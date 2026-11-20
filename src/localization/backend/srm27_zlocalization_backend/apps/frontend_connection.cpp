/**
 * @file frontend_connection.cpp
 * @brief 接入前端会话并持有幂等重置的 ROS 传输生命周期。
 */
#include "apps/frontend_connection.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <optional>

#include "apps/ros.hpp"
#include "backend/localization.hpp"

namespace backend::ros {
namespace {
/// 校验成功重置的同刻坐标边界，拒绝无效时间、非有限平移和非单位旋转。
std::optional<FrontendResetBoundary> decodeResetBoundary(
    const localization_interfaces::srv::ResetFrontend::Response& response) {
    const auto& transform = response.old_from_new;
    const Eigen::Vector3d translation(transform.translation.x, transform.translation.y,
                                      transform.translation.z);
    const Eigen::Quaterniond rotation(transform.rotation.w, transform.rotation.x,
                                      transform.rotation.y, transform.rotation.z);
    if (!validStamp(response.boundary_stamp) || response.source_epoch >= response.target_epoch ||
        !translation.allFinite() || !rotation.coeffs().allFinite() ||
        std::abs(rotation.squaredNorm() - 1.) > 1e-6)
        return std::nullopt;
    FrontendResetBoundary boundary;
    boundary.source_epoch = response.source_epoch;
    boundary.stamp = Timestamp::fromNanoseconds(
        static_cast<std::int64_t>(response.boundary_stamp.sec) * 1000000000LL +
        response.boundary_stamp.nanosec);
    boundary.old_from_new.topLeftCorner<3, 3>() = rotation.normalized().toRotationMatrix();
    boundary.old_from_new.topRightCorner<3, 1>() = translation;
    return boundary;
}

/// 将前端服务响应字段转换为业务回复，未知结果按不确定终态处理。
FrontendResetReply decodeResetReply(
    const localization_interfaces::srv::ResetFrontend::Response& response,
    const rclcpp::Logger& logger) {
    FrontendResetReply reply;
    reply.transaction.requester_instance_id = response.requester_instance_id;
    reply.transaction.transaction_id = response.transaction_id;
    reply.transaction.target_epoch = response.target_epoch;
    reply.frontend_instance_id = response.frontend_instance_id;
    reply.reset_generation = response.reset_generation;
    switch (response.outcome) {
        case localization_interfaces::srv::ResetFrontend::Response::REJECTED:
            reply.outcome = FrontendResetOutcome::Rejected;
            break;
        case localization_interfaces::srv::ResetFrontend::Response::COMPLETED:
            reply.outcome = FrontendResetOutcome::Completed;
            break;
        case localization_interfaces::srv::ResetFrontend::Response::IN_PROGRESS:
            reply.outcome = FrontendResetOutcome::InProgress;
            break;
        case localization_interfaces::srv::ResetFrontend::Response::INDETERMINATE:
        default:
            reply.outcome = FrontendResetOutcome::Indeterminate;
            break;
    }
    if (reply.outcome == FrontendResetOutcome::Completed && response.has_boundary) {
        reply.boundary = decodeResetBoundary(response);
        if (!reply.boundary) {
            reply.outcome = FrontendResetOutcome::Indeterminate;
            RCLCPP_ERROR(logger,
                         "front-end reset returned an invalid coordinate boundary; "
                         "treating the result as indeterminate");
        }
    }
    return reply;
}
}  // namespace
/// 绑定节点和统一失败边界；控制器在装配完成后显式绑定。
FrontendConnection::FrontendConnection(rclcpp::Node& node, const std::atomic<bool>& failed,
                                       FailureHandler failure, Boundary boundary)
    : node_(node), failed_(failed), failure_(std::move(failure)), boundary_(std::move(boundary)) {
}

/// 保存接线信息，尚不创建服务客户端。
void FrontendConnection::configure(bool atomic_input, std::string service) {
    atomic_input_ = atomic_input;
    service_ = std::move(service);
}

/// 绑定控制器，并取得初始化完成后的输入代次。
void FrontendConnection::bind(Localization& controller) {
    controller_ = &controller;
    epoch_.store(controller.activeEpoch());
}

/// 在节点的服务装配阶段创建前端重置客户端。
void FrontendConnection::connect() {
    reset_client_ = node_.create_client<localization_interfaces::srv::ResetFrontend>(service_);
}

/// 回收传输登记，并重试尚未完成的身份确认或输入中断恢复。
void FrontendConnection::maintain(double timeout_seconds) {
    if (reset_client_)
        reset_request_.expire(*reset_client_, timeout_seconds);
    if (atomic_input_) {
        const auto expected_epoch = epoch_.load();
        applySessionAction(session_.pendingAction(), expected_epoch);
    }
}

/// 仲裁前端身份和故障通知，只准入活动且未中断的会话。
bool FrontendConnection::acceptIdentity(std::uint64_t instance, std::uint64_t epoch,
                                        std::optional<std::uint64_t> sequence, bool reset_required,
                                        bool failed) {
    const auto expected_epoch = epoch_.load();
    const auto admission = session_.observe(instance, epoch, sequence, reset_required, failed);
    applySessionAction(admission, expected_epoch);
    return admission == FrontendSession::Admission::Accept;
}

/// 隔离普通处理失败，仅对可恢复中断或身份变化推进重置事务。
void FrontendConnection::applySessionAction(FrontendSession::Admission action,
                                            std::uint64_t expected_epoch) {
    if (!controller_)
        return;
    if (action == FrontendSession::Admission::InputFailure) {
        controller_->notifyFrontendFailure(expected_epoch);
    } else if (action == FrontendSession::Admission::InputDiscontinuity) {
        controller_->notifyFrontendDiscontinuity(expected_epoch);
    } else if (action == FrontendSession::Admission::ResetRequired) {
        RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                             "ZLIO session changed; awaiting coordinated reset boundary");
        controller_->requestCoordinatedReset();
    }
}

/// 返回前端重置客户端是否已经具备可用服务连接。
bool FrontendConnection::frontendResetReady() const {
    return reset_client_ && reset_client_->service_is_ready();
}

/// 以固定业务身份发送重置请求，并将每次回复交回对应身份校验。
bool FrontendConnection::sendFrontendReset(const FrontendResetTransaction& transaction) {
    if (!reset_client_ || !reset_client_->service_is_ready()) {
        RCLCPP_ERROR_STREAM(node_.get_logger(),
                            "front-end reset service unavailable: " << service_);
        return false;
    }
    auto request = std::make_shared<localization_interfaces::srv::ResetFrontend::Request>();
    request->requester_instance_id = transaction.requester_instance_id;
    request->transaction_id = transaction.transaction_id;
    request->target_epoch = transaction.target_epoch;
    std::optional<FrontendSession::ResetAttempt> attempt;
    if (atomic_input_) {
        attempt = session_.prepareReset(transaction);
        if (!attempt)
            return false;
        request->target_epoch = attempt->target_epoch;
    }
    try {
        reset_request_.send(
            *reset_client_, request,
            // 解码本次传输回复，并按其捕获的会话身份执行控制器提交。
            [this, transaction, attempt](
                rclcpp::Client<localization_interfaces::srv::ResetFrontend>::SharedFuture future) {
                if (failed_.load())
                    return;
                try {
                    FrontendResetReply reply;
                    reply.transaction = transaction;
                    try {
                        const auto& response = future.get();
                        reply = decodeResetReply(*response, node_.get_logger());
                    } catch (const std::bad_alloc&) {
                        throw;
                    } catch (const std::exception& error) {
                        RCLCPP_ERROR_STREAM(node_.get_logger(),
                                            "front-end reset transport failed; the "
                                            "same transaction "
                                            "will be retried: "
                                                << error.what());
                        return;
                    }
                    // legacy 输入不保证与重置响应使用同一里程计坐标，不能声明运动连续。
                    if (!atomic_input_)
                        reply.boundary.reset();
                    if (controller_)
                        controller_->onFrontendResetResponse(reply, [this, attempt](auto& value) {
                            return !attempt || session_.translateReply(value, *attempt);
                        });
                } catch (...) {
                    failure_("front-end reset response", std::current_exception());
                }
            });
        if (attempt)
            session_.markSent(*attempt);
    } catch (const std::bad_alloc&) {
        throw;
    } catch (const std::exception& error) {
        if (atomic_input_)
            session_.cancelUnsent(transaction);
        RCLCPP_ERROR_STREAM(node_.get_logger(), "front-end reset request failed: " << error.what());
        return false;
    }
    return true;
}

/// 先确认会话并清除旧故障，再公布新订阅代次并重建输入和输出水位。
void FrontendConnection::onFrontendResetBoundary() {
    const auto next_epoch = controller_ ? controller_->activeEpoch() : 0;
    if (atomic_input_)
        session_.commitBoundary(next_epoch);
    epoch_.store(next_epoch);
    boundary_();
}
}  // namespace backend::ros
