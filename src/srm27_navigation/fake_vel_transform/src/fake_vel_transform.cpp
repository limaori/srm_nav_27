// Copyright 2025 Lihan Chen
// Copyright 2026 SRM
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "fake_vel_transform/fake_vel_transform.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace fake_vel_transform
{

namespace
{
/// \brief legacy 模式判定"速度是否为零"的阈值（与改造前一致）。
constexpr double EPSILON = 1e-5;
/// \brief legacy 模式判定"导航控制器是否活跃"的超时（与改造前一致）。
constexpr double CONTROLLER_TIMEOUT = 0.5;
/// \brief 角度换算常数（不依赖 M_PI 的平台定义）。
constexpr double RAD_TO_DEG = 57.295779513082320876;

/// \brief 单调时钟（秒），只用于超时判断。
double steadyNow()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::steady_clock::now().time_since_epoch())
           .count() *
         1.0e-9;
}

/// \brief 速度的所有分量是否有限。
bool allFinite(const geometry_msgs::msg::Twist & _msg)
{
  return std::isfinite(_msg.linear.x) && std::isfinite(_msg.linear.y) &&
         std::isfinite(_msg.linear.z) && std::isfinite(_msg.angular.x) &&
         std::isfinite(_msg.angular.y) && std::isfinite(_msg.angular.z);
}

std::string formatDouble(const double _value)
{
  std::ostringstream stream;
  stream.precision(6);
  stream << _value;
  return stream.str();
}

}  // namespace

FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  current_robot_base_angle_ = 0.0;
  last_controller_activate_time_ = this->get_clock()->now();
  last_odom_steady_ = steadyNow();

  this->declare_parameter<std::string>("transform_mode", "legacy");
  this->declare_parameter<std::string>("robot_base_frame", "gimbal_link");
  this->declare_parameter<std::string>("fake_robot_base_frame", "gimbal_link_fake");
  this->declare_parameter<std::string>("odom_topic", "odom");
  this->declare_parameter<std::string>("local_plan_topic", "local_plan");
  this->declare_parameter<std::string>("cmd_spin_topic", "cmd_spin");
  this->declare_parameter<std::string>("input_cmd_vel_topic", "");
  this->declare_parameter<std::string>("output_cmd_vel_topic", "");
  this->declare_parameter<double>("odom_timeout", 0.25);
  this->declare_parameter<double>("diagnostics_period", 0.5);
  this->declare_parameter<float>("init_spin_speed", 0.0);

  std::string mode_text;
  this->get_parameter("transform_mode", mode_text);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("fake_robot_base_frame", fake_robot_base_frame_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("local_plan_topic", local_plan_topic_);
  this->get_parameter("cmd_spin_topic", cmd_spin_topic_);
  this->get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
  this->get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
  this->get_parameter("odom_timeout", odom_timeout_);
  this->get_parameter("diagnostics_period", diagnostics_period_);
  this->get_parameter("init_spin_speed", spin_speed_);

  if (mode_text == "nav_se2") {
    mode_ = Mode::kNavSe2;
  } else if (mode_text == "legacy") {
    mode_ = Mode::kLegacy;
  } else {
    RCLCPP_FATAL(
      get_logger(), "fake_vel_transform: 不支持的 transform_mode=%s（可选 legacy / nav_se2）",
      mode_text.c_str());
    throw std::invalid_argument("unsupported transform_mode: " + mode_text);
  }
  if (!(odom_timeout_ > 0.0) || !std::isfinite(odom_timeout_)) {
    RCLCPP_FATAL(get_logger(), "fake_vel_transform: odom_timeout 必须为正数");
    throw std::invalid_argument("odom_timeout must be positive");
  }
  if (!(diagnostics_period_ > 0.0) || !std::isfinite(diagnostics_period_)) {
    RCLCPP_FATAL(get_logger(), "fake_vel_transform: diagnostics_period 必须为正数");
    throw std::invalid_argument("diagnostics_period must be positive");
  }

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // 输入/输出都保持 KEEP_LAST=1：fakeyaw 链路只允许"最新一条"生效，
  // 避免历史命令被延迟转换后按过期 yaw 执行（方案 §4.2）。
  rclcpp::QoS cmd_qos(rclcpp::KeepLast(1));
  cmd_qos.reliable();

  cmd_vel_chassis_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 1);
  diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "diagnostics", rclcpp::QoS(10));

  cmd_spin_sub_ = this->create_subscription<example_interfaces::msg::Float32>(
    cmd_spin_topic_, 1, std::bind(&FakeVelTransform::cmdSpinCallback, this, std::placeholders::_1));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic_, cmd_qos,
    std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));

  // odom 订阅：
  //  * nav_se2：用**普通订阅**（不经 message_filters）best_effort + KEEP_LAST=1。转换必须
  //    与"最新一条里程计"尽量同步，任何中间缓冲都会让 yaw 变旧 —— 自转时表现为
  //    `cmd_vel_nav_body` 用了几十毫秒前的 yaw，看起来就像"坐标转换错了"。
  //    best_effort 订阅既能收 best_effort（Point-LIO/GICP/SLAM），也能收 reliable
  //    （仿真真值适配器）发布者。
  //  * legacy：沿用改造前的默认 QoS 与 message_filters 路径（行为逐字保留）。
  if (mode_ == Mode::kNavSe2) {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(1).best_effort(),
      std::bind(&FakeVelTransform::odometryCallback, this, std::placeholders::_1));
  } else {
    odom_sub_filter_.subscribe(this, odom_topic_);
    odom_sub_filter_.registerCallback(
      std::bind(&FakeVelTransform::odometryCallback, this, std::placeholders::_1));
  }

  if (mode_ == Mode::kNavSe2) {
    diagnostics_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(diagnostics_period_),
      std::bind(&FakeVelTransform::publishDiagnostics, this));
    if (std::abs(spin_speed_) > 1e-9) {
      RCLCPP_WARN(
        get_logger(),
        "fake_vel_transform: nav_se2 模式忽略 init_spin_speed=%.3f；导航拥有 wz，"
        "叠加自转会造成双重旋转（方案 §3.1）。",
        static_cast<double>(spin_speed_));
      spin_speed_ = 0.0F;
    }
    RCLCPP_INFO(
      get_logger(),
      "FakeVelTransform 已启动 (nav_se2): TF %s -> %s, odom=%s, 速度 [%s] -> [%s], "
      "odom 超时 %.3f s, wz 逐条透传，无自转叠加。",
      robot_base_frame_.c_str(), fake_robot_base_frame_.c_str(), odom_topic_.c_str(),
      input_cmd_vel_topic_.c_str(), output_cmd_vel_topic_.c_str(), odom_timeout_);
  } else {
    local_plan_sub_filter_.subscribe(this, local_plan_topic_);
    local_plan_sub_filter_.registerCallback(
      std::bind(&FakeVelTransform::localPlanCallback, this, std::placeholders::_1));
    // In Navigation2 Humble release, the velocity is published by the controller without
    // timestamped. We consider the velocity is published at the same time as local_plan.
    // Therefore, we use ApproximateTime policy to synchronize `cmd_vel` and `odometry`.
    sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(100), odom_sub_filter_, local_plan_sub_filter_);
    sync_->registerCallback(
      std::bind(&FakeVelTransform::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

    // 50Hz Timer to send transform from `robot_base_frame` to `fake_robot_base_frame`
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&FakeVelTransform::publishLegacyTransform, this));
    RCLCPP_INFO(
      get_logger(),
      "Start FakeVelTransform! (legacy): TF %s -> %s, odom=%s, local_plan=%s, "
      "速度 [%s] -> [%s], cmd_spin=%s（沿用自转叠加）",
      robot_base_frame_.c_str(), fake_robot_base_frame_.c_str(), odom_topic_.c_str(),
      local_plan_topic_.c_str(), input_cmd_vel_topic_.c_str(), output_cmd_vel_topic_.c_str(),
      cmd_spin_topic_.c_str());
  }
}

void FakeVelTransform::cmdSpinCallback(const example_interfaces::msg::Float32::SharedPtr msg)
{
  if (mode_ == Mode::kNavSe2) {
    // 迁移模式：自转归导航所有。收到非零 cmd_spin 只报警、只计数，绝不影响输出。
    if (std::abs(msg->data) > 1e-9) {
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        non_zero_spin_count_++;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "fake_vel_transform: nav_se2 模式下收到非零 cmd_spin=%.3f，已忽略"
        "（叠加会造成角速度双重叠加，方案 §3.1）。",
        static_cast<double>(msg->data));
    }
    return;
  }
  spin_speed_ = msg->data;
}

void FakeVelTransform::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  const auto & orientation = msg->pose.pose.orientation;
  const bool usable = isUsableOrientation(
    orientation.x, orientation.y, orientation.z, orientation.w);

  if (mode_ == Mode::kLegacy) {
    // 与改造前一致：控制器不活跃时用里程计 yaw；活跃时由 odom/local_plan 同步回调更新。
    if ((this->get_clock()->now() - last_controller_activate_time_).seconds() > CONTROLLER_TIMEOUT) {
      if (usable) {
        current_robot_base_angle_ = tf2::getYaw(orientation);
      }
    }
    return;
  }

  bool publish_tf = false;
  double yaw = 0.0;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_odom_steady_ = steadyNow();
    last_odom_age_ = 0.0;
    if (usable) {
      yaw = tf2::getYaw(orientation);
      if (std::isfinite(yaw)) {
        latest_yaw_ = yaw;
        has_yaw_ = true;
        yaw_usable_ = true;
        publish_tf = true;
      } else {
        yaw_usable_ = false;
        nonfinite_yaw_count_++;
      }
    } else {
      // 姿态不可用：立即停止发布有效速度，而不是沿用上一条好样本的 yaw。
      yaw_usable_ = false;
      nonfinite_yaw_count_++;
    }
  }

  if (!publish_tf) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "fake_vel_transform: 里程计姿态不可用（非有限四元数），已停止更新 %s TF 并输出零速。",
      fake_robot_base_frame_.c_str());
    return;
  }

  // TF 用里程计消息自带的时间戳发布：与速度转换用的是同一条样本的 yaw（方案 §4.2）。
  publishFakeTf(rclcpp::Time(msg->header.stamp));
}

void FakeVelTransform::publishFakeTf(const rclcpp::Time & _stamp)
{
  double yaw = 0.0;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!has_yaw_ || !yaw_usable_) {
      stale_tf_suppressed_count_++;
      return;
    }
    yaw = latest_yaw_;
    tf_publish_count_++;
  }

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = _stamp;
  transform.header.frame_id = robot_base_frame_;
  transform.child_frame_id = fake_robot_base_frame_;
  // fake 系与 odom 对齐：base_link -> base_link_fake 的 yaw 是真实底盘 yaw 的逆。
  const QuaternionZ quaternion = quaternionFromYaw(fakeFrameYaw(yaw));
  transform.transform.rotation.x = quaternion.x;
  transform.transform.rotation.y = quaternion.y;
  transform.transform.rotation.z = quaternion.z;
  transform.transform.rotation.w = quaternion.w;
  // 原点重合：平移必须严格为零。
  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 0.0;
  tf_broadcaster_->sendTransform(transform);
}

void FakeVelTransform::publishZeroVelocity()
{
  cmd_vel_chassis_pub_->publish(geometry_msgs::msg::Twist());
  std::lock_guard<std::mutex> lock(state_mutex_);
  zero_output_count_++;
  last_output_wz_ = 0.0;
}

void FakeVelTransform::publishTransformedVelocity(const geometry_msgs::msg::Twist & _cmd)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    input_count_++;
  }
  if (!allFinite(_cmd)) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      nonfinite_input_count_++;
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "fake_vel_transform: [%s] 收到含 NaN/Inf 的速度，已输出零速。", input_cmd_vel_topic_.c_str());
    publishZeroVelocity();
    return;
  }

  double yaw = 0.0;
  bool usable = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const double age = steadyNow() - last_odom_steady_;
    last_odom_age_ = age;
    if (!has_yaw_) {
      no_odom_count_++;
    } else if (!yaw_usable_) {
      nonfinite_yaw_count_++;
    } else if (age > odom_timeout_) {
      stale_odom_count_++;
    } else {
      yaw = latest_yaw_;
      usable = true;
    }
  }

  if (!usable) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "fake_vel_transform: 没有可用的底盘 yaw（odom=%s，超时 %.3f s），已输出零速。",
      odom_topic_.c_str(), odom_timeout_);
    publishZeroVelocity();
    return;
  }

  const PlanarVelocity v_fake{_cmd.linear.x, _cmd.linear.y, _cmd.angular.z};
  const PlanarVelocity v_base = fakeToRobotBase(v_fake, yaw);

  geometry_msgs::msg::Twist out;
  out.linear.x = v_base.vx;
  out.linear.y = v_base.vy;
  out.linear.z = _cmd.linear.z;
  out.angular = _cmd.angular;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const double wz_error = std::abs(v_base.wz - v_fake.wz);
    wz_error_max_ = std::max(wz_error_max_, wz_error);
    wz_error_sum_ += wz_error;
    wz_error_count_++;
    output_count_++;
    last_output_wz_ = out.angular.z;
  }
  cmd_vel_chassis_pub_->publish(out);
}

void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (mode_ == Mode::kNavSe2) {
    publishTransformedVelocity(*msg);
    return;
  }

  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  const bool is_zero_vel = std::abs(msg->linear.x) < EPSILON && std::abs(msg->linear.y) < EPSILON &&
                           std::abs(msg->angular.z) < EPSILON;
  if (
    is_zero_vel ||
    (this->get_clock()->now() - last_controller_activate_time_).seconds() > CONTROLLER_TIMEOUT) {
    // If received velocity cannot be synchronized, publish it directly
    auto aft_tf_vel = transformVelocity(*msg, current_robot_base_angle_);
    cmd_vel_chassis_pub_->publish(aft_tf_vel);
  } else {
    latest_cmd_vel_ = msg;
  }
}

void FakeVelTransform::localPlanCallback(const nav_msgs::msg::Path::ConstSharedPtr & /*msg*/)
{
  // Consider nav2_controller_server is activated when receiving local_plan
  last_controller_activate_time_ = this->get_clock()->now();
}

void FakeVelTransform::syncCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg,
  const nav_msgs::msg::Path::ConstSharedPtr & /*local_plan_msg*/)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  geometry_msgs::msg::Twist::SharedPtr current_cmd_vel;
  {
    if (!latest_cmd_vel_) {
      return;
    }
    current_cmd_vel = latest_cmd_vel_;
  }

  current_robot_base_angle_ = tf2::getYaw(odom_msg->pose.pose.orientation);
  float yaw_diff = current_robot_base_angle_;
  geometry_msgs::msg::Twist aft_tf_vel = transformVelocity(*current_cmd_vel, yaw_diff);

  cmd_vel_chassis_pub_->publish(aft_tf_vel);
}

void FakeVelTransform::publishLegacyTransform()
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = robot_base_frame_;
  t.child_frame_id = fake_robot_base_frame_;
  tf2::Quaternion q;
  q.setRPY(0, 0, -current_robot_base_angle_);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);
}

geometry_msgs::msg::Twist FakeVelTransform::transformVelocity(
  const geometry_msgs::msg::Twist & twist, double yaw)
{
  geometry_msgs::msg::Twist aft_tf_vel;
  // legacy 模式：wz 由 cmd_spin 链路唯一提供（导航 wz 在 mux 里被丢弃）。
  aft_tf_vel.angular.z = twist.angular.z + spin_speed_;
  const PlanarVelocity v_base = fakeToRobotBase(
    PlanarVelocity{twist.linear.x, twist.linear.y, twist.angular.z}, yaw);
  aft_tf_vel.linear.x = v_base.vx;
  aft_tf_vel.linear.y = v_base.vy;
  return aft_tf_vel;
}

void FakeVelTransform::publishDiagnostics()
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = this->get_clock()->now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "fake_vel_transform";
  status.hardware_id = this->get_namespace();

  const double odom_age = steadyNow() - last_odom_steady_;
  const bool yaw_ok = has_yaw_ && yaw_usable_ && odom_age <= odom_timeout_;
  if (!has_yaw_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "尚未收到里程计，速度输出为零";
  } else if (!yaw_usable_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "里程计姿态不可用（非有限四元数），速度输出为零";
  } else if (odom_age > odom_timeout_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "里程计超时，速度输出为零";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "fakeyaw 转换正常";
  }

  const auto add = [&status](const std::string & _key, const std::string & _value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = _key;
    kv.value = _value;
    status.values.push_back(kv);
  };

  add("mode", "nav_se2");
  add("robot_base_frame", robot_base_frame_);
  add("fake_robot_base_frame", fake_robot_base_frame_);
  add("yaw_deg", formatDouble(latest_yaw_ * RAD_TO_DEG));
  add("yaw_valid", yaw_ok ? "true" : "false");
  add("odom_age", formatDouble(odom_age));
  add("odom_timeout", formatDouble(odom_timeout_));
  add("input_count", std::to_string(input_count_));
  add("output_count", std::to_string(output_count_));
  add("zero_output_count", std::to_string(zero_output_count_));
  add("no_odom_count", std::to_string(no_odom_count_));
  add("stale_odom_count", std::to_string(stale_odom_count_));
  add("nonfinite_yaw_count", std::to_string(nonfinite_yaw_count_));
  add("nonfinite_input_count", std::to_string(nonfinite_input_count_));
  add("tf_publish_count", std::to_string(tf_publish_count_));
  add("stale_tf_suppressed_count", std::to_string(stale_tf_suppressed_count_));
  add("non_zero_spin_count", std::to_string(non_zero_spin_count_));
  add("last_output_wz", formatDouble(last_output_wz_));
  add("wz_error_max", formatDouble(wz_error_max_));
  add(
    "wz_error_mean",
    formatDouble(wz_error_count_ > 0 ? wz_error_sum_ / static_cast<double>(wz_error_count_) : 0.0));

  array.status.push_back(status);
  diagnostics_pub_->publish(array);
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)
