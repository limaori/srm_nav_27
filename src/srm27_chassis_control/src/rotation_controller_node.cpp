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

/// \brief rotation_controller：独立仿真自转速度的接收与输出。
///
/// 只订阅 rotation_cmd 并发布 rotation_velocity，只使用 angular.z，
/// 不发布也不修改任何导航相关话题，因此不会覆盖导航状态。
///
/// 守护线程用单调时钟检测自转请求超时（默认 0.5 s）并清零；正常变速可配置
/// 角加速度上限，停止命令与超时清零优先级最高，直接归零。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "srm27_chassis_control/rotation_waveform.hpp"
#include "srm27_chassis_control/twist_watchdog.hpp"

using srm27_chassis_control::TwistWatchdog;
using srm27_chassis_control::applyAngularAccelLimit;

class RotationControllerNode : public rclcpp::Node
{
public:
  RotationControllerNode()
  : rclcpp::Node("rotation_controller")
  {
    this->input_topic_ = this->declare_parameter<std::string>(
      "input_topic", "rotation_cmd");
    this->output_topic_ = this->declare_parameter<std::string>(
      "output_topic", "rotation_velocity");
    const double publishRate =
      this->declare_parameter<double>("publish_rate", 200.0);
    this->request_timeout_ = this->declare_parameter<double>(
      "request_timeout", 0.5);
    this->max_accel_ = this->declare_parameter<double>("max_angular_accel", 0.0);
    this->wz_max_ = this->declare_parameter<double>("wz_max", 2.0);
    this->diagnostics_period_ = std::chrono::duration<double>(
      this->declare_parameter<double>("diagnostics_period", 0.5));

    this->watchdog_.setTimeout(this->request_timeout_);
    this->publish_period_ = std::chrono::duration<double>(
      publishRate > 0.0 ? 1.0 / publishRate : 0.005);

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();
    this->subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      this->input_topic_, qos,
      [this](const geometry_msgs::msg::Twist::SharedPtr _msg) {
        this->onRotationCommand(*_msg);
      });
    this->publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>(this->output_topic_, qos);
    this->diagnostics_pub_ =
      this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "diagnostics", rclcpp::QoS(10));
    this->clear_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/clear_rotation",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> _response) {
        {
          std::lock_guard<std::mutex> lock(this->mutex_);
          this->target_wz_ = 0.0;
          this->output_wz_ = 0.0;
          this->watchdog_.reset();
        }
        _response->success = true;
        _response->message = "自转命令已清空并清零输出";
      });

    this->guard_thread_ = std::thread([this]() {this->guardLoop();});

    RCLCPP_INFO(
      this->get_logger(),
      "rotation_controller 已启动: [%s] -> [%s], 请求超时 %.3f s, "
      "角加速度上限 %.3f rad/s^2, 输出上限 %.3f rad/s",
      this->input_topic_.c_str(), this->output_topic_.c_str(),
      this->request_timeout_, this->max_accel_, this->wz_max_);
  }

  ~RotationControllerNode() override
  {
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->shutdown_ = true;
      this->target_wz_ = 0.0;
      this->output_wz_ = 0.0;
    }
    this->condition_.notify_all();
    if (this->guard_thread_.joinable()) {
      this->guard_thread_.join();
    }
    this->publisher_->publish(geometry_msgs::msg::Twist());
  }

private:
  void onRotationCommand(const geometry_msgs::msg::Twist & _msg)
  {
    const auto now = std::chrono::steady_clock::now();
    const bool finite = std::isfinite(_msg.angular.z);
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      if (!finite) {
        this->target_wz_ = 0.0;
        this->non_finite_count_++;
      } else {
        this->target_wz_ = _msg.angular.z;
      }
      this->watchdog_.markInput(now);
    }
    if (!finite) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "[%s] 收到含 NaN/Inf 的自转命令，已按零速处理。",
        this->input_topic_.c_str());
    }
  }

  void guardLoop()
  {
    std::unique_lock<std::mutex> lock(this->mutex_);
    auto nextDiagnostics = std::chrono::steady_clock::now();
    auto lastUpdate = std::chrono::steady_clock::now();
    while (!this->shutdown_) {
      this->condition_.wait_for(lock, this->publish_period_);
      if (this->shutdown_) {
        break;
      }
      const auto now = std::chrono::steady_clock::now();
      const double dt = std::chrono::duration<double>(now - lastUpdate).count();
      lastUpdate = now;

      // 超时清零优先：输入中断后不允许继续自转。
      const bool expired = this->watchdog_.expired(now);
      double target = expired ? 0.0 : this->target_wz_;
      if (expired && !this->timeout_active_) {
        this->timeout_active_ = true;
        RCLCPP_WARN(
          this->get_logger(),
          "[%s] 超过 %.3f s 未收到自转请求，自转速度已清零。",
          this->input_topic_.c_str(), this->request_timeout_);
      } else if (!expired) {
        this->timeout_active_ = false;
      }

      if (std::isfinite(target)) {
        target = std::max(-this->wz_max_, std::min(this->wz_max_, target));
      } else {
        target = 0.0;
      }

      double limited = applyAngularAccelLimit(
        target, this->output_wz_, dt, this->max_accel_);
      if (!std::isfinite(limited)) {
        limited = 0.0;
      }
      this->output_wz_ = limited;

      const auto stamp = this->now();
      const double output = limited;
      const double targetCopy = target;
      lock.unlock();

      geometry_msgs::msg::Twist command;
      command.angular.z = output;
      this->publisher_->publish(command);

      if (now >= nextDiagnostics) {
        nextDiagnostics = now + std::chrono::duration_cast<
          std::chrono::steady_clock::duration>(this->diagnostics_period_);
        this->publishDiagnostics(expired, targetCopy, output, stamp);
      }

      lock.lock();
    }
  }

  void publishDiagnostics(
    bool _expired, double _target, double _output, const rclcpp::Time & _stamp)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = _stamp;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "srm27_chassis_control: rotation_controller";
    status.hardware_id = this->get_namespace();
    if (_expired) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "自转请求超时，输出已清零";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "自转输出正常";
    }
    const auto addKeyValue = [&status](
      const std::string & _key, const std::string & _value) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = _key;
        kv.value = _value;
        status.values.push_back(kv);
      };
    addKeyValue("target_wz", std::to_string(_target));
    addKeyValue("output_wz", std::to_string(_output));
    addKeyValue("request_timeout", std::to_string(this->request_timeout_));
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      addKeyValue("non_finite_count", std::to_string(this->non_finite_count_));
    }
    array.status.push_back(status);
    this->diagnostics_pub_->publish(array);
  }

  std::string input_topic_;
  std::string output_topic_;
  double request_timeout_{0.5};
  double max_accel_{0.0};
  double wz_max_{2.0};
  std::chrono::duration<double> publish_period_{0.005};
  std::chrono::duration<double> diagnostics_period_{0.5};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_service_;

  TwistWatchdog watchdog_{0.5};

  std::mutex mutex_;
  std::condition_variable condition_;
  double target_wz_{0.0};
  double output_wz_{0.0};
  bool timeout_active_{false};
  bool shutdown_{false};
  uint64_t non_finite_count_{0};
  std::thread guard_thread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RotationControllerNode>());
  rclcpp::shutdown();
  return 0;
}
