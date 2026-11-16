/**
 * @file node.cpp
 * @brief 协调传感器输入、估计处理、会话切换和 ROS 发布。
 */
#include "node.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

namespace zlio::ros {
namespace {
/// 保存同一后验时刻的新旧里程计坐标系变换及其源代次。
void setResetBoundary(localization_interfaces::srv::ResetFrontend::Response& response,
                      std::uint64_t source_epoch, const State& previous, const State& current) {
    if (previous.time != current.time)
        throw std::logic_error("warm reset estimates must share the same timestamp");
    const auto stamp = makeTimeMessage(previous.time);
    const Eigen::Quaterniond rotation =
        (previous.pose.rotation * current.pose.rotation.conjugate()).normalized();
    const Vec3 translation = previous.pose.position - rotation * current.pose.position;
    response.has_boundary = true;
    response.source_epoch = source_epoch;
    response.boundary_stamp = stamp;
    response.old_from_new.translation.x = translation.x();
    response.old_from_new.translation.y = translation.y();
    response.old_from_new.translation.z = translation.z();
    response.old_from_new.rotation.x = rotation.x();
    response.old_from_new.rotation.y = rotation.y();
    response.old_from_new.rotation.z = rotation.z();
    response.old_from_new.rotation.w = rotation.w();
}
}  // namespace

/// 创建里程计对象和 ROS 传输接口。
OdometryNode::OdometryNode() : Node("zlio") {
    loadParameters();
    auto prepared = prepareInput();
    odometry_ = std::move(prepared.odometry);
    predictor_ = std::move(prepared.predictor);
    queue_ = std::move(prepared.queue);
    cloud_format_.maximum_points = input_limits_.maximum_lidar_points;
    decoder_ = std::make_unique<PointCloudDecoder>(cloud_format_);
    setupPublishers();
    setupServices();
    recovery_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] { recoverInput(); });
    publishSession();
    if (!armed_)
        RCLCPP_INFO(get_logger(), "waiting for %s with data=true",
                    start_service_->get_service_name());
    startInput();
}

/// 停止输入并等待估计线程退出。
OdometryNode::~OdometryNode() {
    requestStop();
    if (worker_.joinable())
        worker_.join();
}

/// 构造使用相同算法配置、历史容量和输出分辨率的替代会话资源。
OdometryNode::PreparedInput OdometryNode::prepareInput() const {
    PreparedInput prepared;
    prepared.odometry = std::make_unique<Odometry>(config_, input_limits_);
    prepared.odometry->setOutputResolution(output_resolution_);
    prepared.predictor = std::make_unique<OutputPredictor>(
        Vec3(0, 0, -config_.gravity), config_.noise, input_limits_.imu_history);
    prepared.queue =
        std::make_unique<InputQueue>(maximum_queue_events_, input_limits_.maximum_lidar_points);
    return prepared;
}

/// 读取算法文件与 ROS 传输参数。
void OdometryNode::loadParameters() {
    const auto path = declare_parameter<std::string>("config_file", "");
    if (path.empty())
        throw std::invalid_argument("config_file is required");
    config_ = loadConfig(path);
    frames_.world = declare_parameter<std::string>("world_frame", "odom");
    frames_.body = declare_parameter<std::string>("body_frame", "imu");
    lidar_topic_ = declare_parameter<std::string>("lidar_topic", "/livox/lidar");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/livox/imu");
    input_type_ = declare_parameter<std::string>("input_type", "pointcloud2");
    cloud_format_.time_field = declare_parameter<std::string>("point_time_field", "offset_time");
    cloud_format_.time_scale = declare_parameter<double>("point_time_scale", 1e-9);
    cloud_format_.absolute_time = declare_parameter<bool>("point_time_absolute", false);
    livox_timebase_ = declare_parameter<bool>("livox_use_timebase", true);
    input_limits_.imu_history.maximum_duration =
        declare_parameter<double>("imu_history_seconds", 30);
    input_limits_.imu_history.maximum_interval =
        declare_parameter<double>("maximum_imu_interval", .1);
    const auto count = [this](const char* name, std::size_t initial) {
        const auto value =
            declare_parameter<std::int64_t>(name, static_cast<std::int64_t>(initial));
        if (value <= 0)
            throw std::invalid_argument(std::string(name) + " must be positive");
        return static_cast<std::size_t>(value);
    };
    input_limits_.imu_history.maximum_samples = count("maximum_imu_samples", 65536);
    input_limits_.maximum_lidar_points = count("maximum_buffered_points", 1000000);
    maximum_queue_events_ = count("maximum_queue_events", maximum_queue_events_);
    output_resolution_ = declare_parameter<double>("output_resolution", .10);
    imu_acceleration_scale_ = declare_parameter<double>("imu_acceleration_scale", 1.);
    if (!std::isfinite(imu_acceleration_scale_) || imu_acceleration_scale_ <= 0)
        throw std::invalid_argument("imu_acceleration_scale must be finite and positive");
    armed_ = !declare_parameter<bool>("wait_for_start", false);
    reset_mode_ = declare_parameter<std::string>("reset_mode", "warm");
    coordinated_reset_ = declare_parameter<bool>("coordinated_reset", false);
    if (reset_mode_ != "warm" && reset_mode_ != "cold")
        throw std::invalid_argument("reset_mode must be warm or cold");
}

/// 创建里程计和坐标变换发布器。
void OdometryNode::setupPublishers() {
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry", rclcpp::QoS(200));
    posterior_pub_ = create_publisher<nav_msgs::msg::Odometry>("lidar_odometry", rclcpp::QoS(100));
    frame_pub_ = create_publisher<localization_interfaces::msg::FrontendFrame>("frontend/frame",
                                                                               rclcpp::QoS(64));
    prediction_pub_ = create_publisher<localization_interfaces::msg::FrontendOdometry>(
        "frontend/odometry", rclcpp::QoS(200));
    session_pub_ = create_publisher<localization_interfaces::msg::FrontendSession>(
        "frontend/session", rclcpp::QoS(1).transient_local());
    if (declare_parameter<bool>("publish_tf", true))
        broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

/// 创建输入启用、停止和幂等重置服务。
void OdometryNode::setupServices() {
    start_service_ = create_service<std_srvs::srv::SetBool>(
        "/localization_start", [this](std_srvs::srv::SetBool::Request::ConstSharedPtr request,
                                      std_srvs::srv::SetBool::Response::SharedPtr response) {
            if (!request->data) {
                response->success = false;
                response->message = "start request requires data=true";
                return;
            }
            if (queue_->failed() ||
                finished_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                response->success = false;
                response->message = "odometry is stopped; reset is required before starting";
                return;
            }
            response->success = true;
            response->message = armed_ ? "sensor input is already enabled" : "sensor input enabled";
            if (!armed_)
                RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
            armed_ = true;
        });
    reset_service_ = create_service<localization_interfaces::srv::ResetFrontend>(
        "/zlio/reset_internal",
        [this](localization_interfaces::srv::ResetFrontend::Request::SharedPtr request,
               localization_interfaces::srv::ResetFrontend::Response::SharedPtr response) {
            reset(*request, *response);
        });
    stop_service_ = create_service<std_srvs::srv::Trigger>(
        "stop", [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
                       std_srvs::srv::Trigger::Response::SharedPtr response) {
            requestStop();
            finished_future_.wait();
            response->success = !queue_->failed();
            response->message =
                queue_->failed() ? "odometry stopped after an error" : "odometry stopped";
        });
}

/// 按配置建立传感器订阅。
void OdometryNode::setupSubscriptions() {
    // 传感器与 reset 共用默认互斥回调组；估计线程独立运行。
    const auto epoch = epoch_;
    rclcpp::SubscriptionOptions imu_options, cloud_options;
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::SensorDataQoS().keep_last(4000),
        [this, epoch](sensor_msgs::msg::Imu::ConstSharedPtr message) {
            if (armed_ && epoch == epoch_)
                onImu(message);
        },
        imu_options);
    if (input_type_ == "pointcloud2") {
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic_, rclcpp::SensorDataQoS().keep_last(200),
            [this, epoch](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
                if (armed_ && epoch == epoch_)
                    onCloud(message);
            },
            cloud_options);
    } else if (input_type_ == "livox") {
        livox_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
            lidar_topic_, rclcpp::SensorDataQoS().keep_last(200),
            [this, epoch](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr message) {
                if (armed_ && epoch == epoch_)
                    onLivox(message);
            },
            cloud_options);
    } else
        throw std::invalid_argument("input_type must be pointcloud2 or livox");
}

/// 在新会话中建立输入准入与工作线程，并保证启动失败后仍可完成停止。
void OdometryNode::startInput() {
    finished_ = std::promise<void>();
    finished_future_ = finished_.get_future().share();
    pending_input_restart_ = false;
    allow_input_restart_ = true;
    reset_required_ = false;
    input_failed_ = false;
    failure_reported_ = false;
    try {
        setupSubscriptions();
        worker_ = std::thread([this] { work(); });
    } catch (...) {
        finished_.set_value();
        throw;
    }
}

/// 串行关闭传感器订阅，按指定模式结束队列并汇合估计线程。
void OdometryNode::closeInput(InputQueue::CloseMode mode) {
    imu_sub_.reset();
    cloud_sub_.reset();
    livox_sub_.reset();
    queue_->close(mode);
    if (worker_.joinable())
        worker_.join();
}

/// 接收 IMU 并发布当前预测状态。
void OdometryNode::onImu(sensor_msgs::msg::Imu::ConstSharedPtr message) {
    const InputQueue::Admission input(*queue_);
    if (!input)
        return;
    try {
        Imu sample{messageTime(message->header.stamp),
                   Vec3(message->angular_velocity.x, message->angular_velocity.y,
                        message->angular_velocity.z),
                   imu_acceleration_scale_ * Vec3(message->linear_acceleration.x,
                                                  message->linear_acceleration.y,
                                                  message->linear_acceleration.z)};
        std::optional<Estimate> output;
        {
            std::lock_guard<std::mutex> lock(prediction_mutex_);
            output = predictor_->addImu(sample);
        }
        if (output)
            publish(*output, sample.angular_velocity);
        queue_->push(sample);
    } catch (const InputDiscontinuity& error) {
        interruptInput(std::string("IMU input interrupted: ") + error.what());
    } catch (const std::exception& error) {
        fail(std::string("IMU input failed: ") + error.what());
    }
}

/// 解码点云并加入输入通道。
void OdometryNode::onCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
    const InputQueue::Admission input(*queue_);
    if (!input)
        return;
    try {
        queue_->push(decoder_->decode(*message));
    } catch (const std::exception& error) {
        fail(std::string("PointCloud2 input failed: ") + error.what());
    }
}

/// 解码有效 Livox 回波并加入输入通道。
void OdometryNode::onLivox(livox_ros_driver2::msg::CustomMsg::ConstSharedPtr message) {
    const InputQueue::Admission input(*queue_);
    if (!input)
        return;
    try {
        if (message->point_num != message->points.size())
            throw std::invalid_argument("Livox point_num does not match the points array size");
        if (message->points.size() > input_limits_.maximum_lidar_points)
            throw std::length_error("Livox packet point limit exceeded");
        const Time begin = livox_timebase_ ? static_cast<Time>(message->timebase)
                                           : messageTime(message->header.stamp);
        Scan scan;
        scan.reserve(message->points.size());
        for (const auto& point : message->points) {
            if ((point.tag & 0x30) != 0 && (point.tag & 0x30) != 0x10)
                continue;
            scan.push_back({Vec3(point.x, point.y, point.z), begin + point.offset_time, 0});
        }
        queue_->push(std::move(scan));
    } catch (const std::exception& error) {
        fail(std::string("Livox input failed: ") + error.what());
    }
}

/// 关闭新的输入准入并排空已接收事件。
void OdometryNode::requestStop() {
    allow_input_restart_ = false;
    pending_input_restart_ = false;
    queue_->close();
}

/// 锁存普通输入失败，取消待处理事件并禁止自动重试。
void OdometryNode::markInputFailed() {
    allow_input_restart_ = false;
    pending_input_restart_ = false;
    queue_->close(InputQueue::CloseMode::Cancel);
    input_failed_ = true;
}

/// 取消待处理输入并报告运行错误。
void OdometryNode::fail(const std::string& reason) {
    markInputFailed();
    RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
}

/// 取消旧输入并合并重复中断，由串行恢复回调等待所有准入退出。
void OdometryNode::interruptInput(const std::string& reason) {
    queue_->close(InputQueue::CloseMode::Cancel);
    if (allow_input_restart_ && !pending_input_restart_.exchange(true))
        RCLCPP_WARN(get_logger(), "%s", reason.c_str());
}

/// 发布输入故障；可恢复中断在启动前重收数据，启动后切换或协调代次。
void OdometryNode::recoverInput() {
    if (input_failed_) {
        if (!failure_reported_) {
            reset_required_ = false;
            publishSession();
            failure_reported_ = true;
        }
        return;
    }
    if (!pending_input_restart_ || !allow_input_restart_)
        return;
    try {
        closeInput(InputQueue::CloseMode::Cancel);
        pending_input_restart_ = false;
        if (!allow_input_restart_)
            return;
        if (!has_posterior_) {
            auto prepared = prepareInput();
            odometry_ = std::move(prepared.odometry);
            predictor_ = std::move(prepared.predictor);
            queue_ = std::move(prepared.queue);
            sequence_ = 0;
            startInput();
            publishSession();
        } else if (coordinated_reset_) {
            reset_required_ = true;
            publishSession();
        } else {
            if (epoch_ == std::numeric_limits<std::uint64_t>::max())
                throw std::overflow_error("front-end epoch exhausted during input recovery");
            localization_interfaces::srv::ResetFrontend::Request request;
            request.requester_instance_id = reset_cache_.frontendInstanceId();
            request.target_epoch = epoch_ + 1;
            request.transaction_id = request.target_epoch;
            localization_interfaces::srv::ResetFrontend::Response response;
            reset(request, response);
            if (response.outcome != response.COMPLETED)
                fail("automatic input recovery failed: " + response.message);
        }
    } catch (const std::exception& error) {
        fail(std::string("input recovery failed: ") + error.what());
    }
}

/// 将最新后验提交给实时输出预测器。
void OdometryNode::commit(const Estimate& estimate) {
    std::lock_guard<std::mutex> lock(prediction_mutex_);
    if (!predictor_->correct(estimate, odometry_->gravity()))
        throw std::runtime_error("posterior commit time did not advance");
}

/// 处理可用观测并发布后验结果。
void OdometryNode::collect(bool flush) {
    if (odometry_->initialized() && !has_posterior_) {
        commit(odometry_->estimate());
        has_posterior_ = true;
    }
    const auto results = odometry_->processAvailable(flush);
    if (!results.empty())
        commit(results.back().estimate);
    for (const auto& result : results) {
        localization_interfaces::msg::FrontendFrame message;
        message.instance_id = reset_cache_.frontendInstanceId();
        message.epoch = epoch_;
        message.sequence = ++sequence_;
        message.begin = static_cast<builtin_interfaces::msg::Time>(rclcpp::Time(result.begin));
        message.odometry = makeOdometryMessage(result.estimate, result.angular_velocity, frames_,
                                               config_.noise.gyro_sample_variance);
        message.matches = static_cast<std::uint32_t>(result.matches);
        message.lidar_updated = result.lidar_updated;
        message.cloud = makePointCloudMessage(result.cloud, message.odometry.header, frames_.body);
        posterior_pub_->publish(message.odometry);
        frame_pub_->publish(message);
    }
    if (const auto watermark = odometry_->posteriorWatermark()) {
        std::lock_guard<std::mutex> lock(prediction_mutex_);
        predictor_->discardBefore(*watermark);
    }
}

/// 按输入顺序运行核心估计并完成停止排空。
void OdometryNode::work() {
    try {
        bool received_input = false;
        while (auto event = queue_->take()) {
            received_input = true;
            if (const auto* imu = std::get_if<Imu>(&*event))
                odometry_->addImu(*imu);
            else
                odometry_->addScan(std::move(std::get<Scan>(*event)));
            collect();
        }
        if (!queue_->failed())
            collect(true);
        // 等待启动或尚无数据时，正常停止不属于初始化失败。
        if (!queue_->failed() && received_input && !odometry_->initialized())
            fail("initialization incomplete: " + odometry_->initializationStatus().reason);
    } catch (const InputDiscontinuity& error) {
        interruptInput(std::string("odometry input interrupted: ") + error.what());
    } catch (const std::exception& error) {
        fail(std::string("odometry processing failed: ") + error.what());
    }
    queue_->finish();
    finished_.set_value();
}

/// 发布主里程计及对应坐标变换。
void OdometryNode::publish(const Estimate& estimate, const Vec3& angular_velocity) {
    const auto odometry = makeOdometryMessage(estimate, angular_velocity, frames_,
                                              config_.noise.gyro_sample_variance);
    odom_pub_->publish(odometry);
    localization_interfaces::msg::FrontendOdometry message;
    message.instance_id = reset_cache_.frontendInstanceId();
    message.epoch = epoch_;
    message.odometry = odometry;
    prediction_pub_->publish(message);
    if (broadcaster_) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header = odometry.header;
        transform.child_frame_id = frames_.body;
        transform.transform.translation.x = odometry.pose.pose.position.x;
        transform.transform.translation.y = odometry.pose.pose.position.y;
        transform.transform.translation.z = odometry.pose.pose.position.z;
        transform.transform.rotation = odometry.pose.pose.orientation;
        broadcaster_->sendTransform(transform);
    }
}

/// 发布当前会话身份，通知失败不改变已经完成的重置终态。
void OdometryNode::publishSession() {
    localization_interfaces::msg::FrontendSession message;
    message.instance_id = reset_cache_.frontendInstanceId();
    message.epoch = epoch_;
    message.reset_required = reset_required_;
    message.failed = input_failed_;
    try {
        session_pub_->publish(message);
    } catch (const std::exception& error) {
        // 通知失败不能把已经完成的幂等 reset 改写为另一个终态。
        RCLCPP_ERROR(get_logger(), "session notification failed: %s", error.what());
    }
}

/// 执行幂等重置；先构造替代对象，再关闭旧准入、清空队列并切换代次。
void OdometryNode::reset(const localization_interfaces::srv::ResetFrontend::Request& request,
                         localization_interfaces::srv::ResetFrontend::Response& response) {
    using Cache = localization_interfaces::ResetTransactionCache;
    using Response = localization_interfaces::srv::ResetFrontend::Response;
    if (reset_cache_.begin(request, &response) != Cache::BeginResult::Execute)
        return;
    bool touched = false;
    try {
        if (request.target_epoch <= epoch_) {
            reset_cache_.finish(request, Response::REJECTED, "target_epoch must advance",
                                &response);
            return;
        }
        auto prepared = prepareInput();
        const bool stopped =
            finished_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
        touched = true;
        closeInput(InputQueue::CloseMode::Drain);
        const Estimate previous = odometry_->estimate();
        const std::uint64_t source_epoch = epoch_;
        Response result;
        result.outcome = Response::COMPLETED;
        // 停止或失败期间没有完整输入；仅为持续运行且双方历史完整的会话保留运动状态。
        const bool warm = reset_mode_ == "warm" && !stopped && !queue_->failed() &&
                          predictor_->canReset(previous.state.time) && odometry_->resetWarm();
        if (warm) {
            predictor_->reset(odometry_->estimate(), odometry_->gravity());
            setResetBoundary(result, source_epoch, previous.state, odometry_->estimate().state);
        } else {
            odometry_ = std::move(prepared.odometry);
            predictor_ = std::move(prepared.predictor);
        }
        queue_ = std::move(prepared.queue);
        epoch_ = request.target_epoch;
        sequence_ = 0;
        has_posterior_ = warm;
        startInput();
        result.message =
            warm ? "ZLIO warm reset; velocity, gravity and biases preserved in the new odom frame"
                 : "ZLIO cold-start session activated";
        reset_cache_.finish(request, std::move(result), &response);
        publishSession();
    } catch (const std::exception& e) {
        if (touched) {
            epoch_ = request.target_epoch;
            reset_required_ = false;
            markInputFailed();
        }
        reset_cache_.finish(request, touched ? Response::INDETERMINATE : Response::REJECTED,
                            e.what(), &response);
        if (touched)
            publishSession();
    }
}

}  // namespace zlio::ros
