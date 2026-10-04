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

/// \brief srm_cmd_mux：把导航平移速度与独立自转速度合成为仿真底盘速度。
///
/// 输入：
///   cmd_vel_nav        导航平移速度（geometry_msgs/msg/Twist，车体系）
///   rotation_velocity  独立自转速度（只使用 angular.z）
/// 输出：
///   cmd_vel_sim        最终执行命令（mux 是唯一发布者）
///
/// 规则见实施方案 3.3：导航输入的 angular.z 一律丢弃（并统计非零次数），
/// 自转角速度只来自 rotation_velocity；平移按向量模长限幅；两路输入各自
/// 由守护线程用单调时钟检测超时并清零。

#include <atomic>
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

#include "srm27_chassis_control/twist_watchdog.hpp"
#include "srm27_chassis_control/velocity_mix.hpp"

using srm27_chassis_control::MixInput;
using srm27_chassis_control::MixLimits;
using srm27_chassis_control::TwistWatchdog;

namespace
{
bool allFinite(const geometry_msgs::msg::Twist & _msg)
{
  return std::isfinite(_msg.linear.x) && std::isfinite(_msg.linear.y) &&
         std::isfinite(_msg.angular.z);
}
}  // namespace

class CmdMuxNode : public rclcpp::Node
{
public:
  CmdMuxNode()
  : rclcpp::Node("srm_cmd_mux")
  {
    this->nav_topic_ = this->declare_parameter<std::string>(
      "nav_topic", "cmd_vel_nav");
    this->rotation_topic_ = this->declare_parameter<std::string>(
      "rotation_topic", "rotation_velocity");
    this->output_topic_ = this->declare_parameter<std::string>(
      "output_topic", "cmd_vel_sim");
    const double publishRate =
      this->declare_parameter<double>("publish_rate", 200.0);
    this->nav_timeout_ = this->declare_parameter<double>("nav_timeout", 0.3);
    this->rotation_timeout_ = this->declare_parameter<double>("rotation_timeout", 0.1);
    this->limits_.vx_max = this->declare_parameter<double>("vx_max", 0.5);
    this->limits_.vy_max = this->declare_parameter<double>("vy_max", 0.5);
    this->limits_.v_max = this->declare_parameter<double>("v_max", 0.5);
    this->limits_.wz_max = this->declare_parameter<double>("wz_max", 2.0);
    this->diagnostics_period_ = std::chrono::duration<double>(
      this->declare_parameter<double>("diagnostics_period", 0.5));

    this->nav_watchdog_.setTimeout(this->nav_timeout_);
    this->rotation_watchdog_.setTimeout(this->rotation_timeout_);

    this->publish_period_ = std::chrono::duration<double>(
      publishRate > 0.0 ? 1.0 / publishRate : 0.005);

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();
    this->nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      this->nav_topic_, qos,
      [this](const geometry_msgs::msg::Twist::SharedPtr _msg) {
        this->onNavCommand(*_msg);
      });
    this->rotation_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      this->rotation_topic_, qos,
      [this](const geometry_msgs::msg::Twist::SharedPtr _msg) {
        this->onRotationVelocity(*_msg);
      });
    this->output_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>(this->output_topic_, qos);
    this->diagnostics_pub_ =
      this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "diagnostics", rclcpp::QoS(10));

    this->stop_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/stop_all",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> _response) {
        this->stopAll();
        _response->success = true;
        _response->message = "已停止导航与自转两路输入，保持零输出直到 resume_all";
      });
    this->resume_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/resume_all",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> _response) {
        this->resumeAll();
        _response->success = true;
        _response->message = "已重新启用两路输入";
      });

    this->guard_thread_ = std::thread([this]() {this->guardLoop();});

    RCLCPP_INFO(
      this->get_logger(),
      "srm_cmd_mux 已启动: [%s] + [%s] -> [%s], 平移上限 %.3f m/s (v_max %.3f), "
      "自转上限 %.3f rad/s, 导航超时 %.3f s, 自转超时 %.3f s",
      this->nav_topic_.c_str(), this->rotation_topic_.c_str(),
      this->output_topic_.c_str(), this->limits_.vx_max, this->limits_.v_max,
      this->limits_.wz_max, this->nav_timeout_, this->rotation_timeout_);
  }

  ~CmdMuxNode() override
  {
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->shutdown_ = true;
      this->nav_ = geometry_msgs::msg::Twist();
      this->rotation_wz_ = 0.0;
    }
    this->condition_.notify_all();
    if (this->guard_thread_.joinable()) {
      this->guard_thread_.join();
    }
    // 退出前明确写一次零速度，避免仿真里残留最后一次非零命令。
    this->output_pub_->publish(geometry_msgs::msg::Twist());
  }

private:
  void onNavCommand(const geometry_msgs::msg::Twist & _msg)
  {
    const auto now = std::chrono::steady_clock::now();
    bool rejected = false;
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      if (this->stopped_) {
        // 急停/stop_all 期间仍然计时，但输出保持为零。
        this->nav_watchdog_.markInput(now);
        return;
      }
      if (!allFinite(_msg)) {
        rejected = true;
        this->nav_ = geometry_msgs::msg::Twist();
        this->non_finite_nav_count_++;
      } else {
        this->nav_ = _msg;
        if (std::abs(_msg.angular.z) > 1e-9) {
          // 导航链路约定 wz 必须为 0；这里丢弃并统计，避免它混进最终自转输出。
          this->dropped_nav_yaw_count_++;
        }
      }
      this->nav_watchdog_.markInput(now);
    }
    if (rejected) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "[%s] 收到含 NaN/Inf 的导航速度，已按零速处理。",
        this->nav_topic_.c_str());
    }
  }

  void onRotationVelocity(const geometry_msgs::msg::Twist & _msg)
  {
    const auto now = std::chrono::steady_clock::now();
    bool rejected = false;
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      if (this->stopped_) {
        this->rotation_watchdog_.markInput(now);
        return;
      }
      if (!allFinite(_msg)) {
        rejected = true;
        this->rotation_wz_ = 0.0;
        this->non_finite_rotation_count_++;
      } else {
        // 自转输入只取 angular.z，其余分量不参与合成。
        this->rotation_wz_ = _msg.angular.z;
      }
      this->rotation_watchdog_.markInput(now);
    }
    if (rejected) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "[%s] 收到含 NaN/Inf 的自转速度，已按零速处理。",
        this->rotation_topic_.c_str());
    }
  }

  void stopAll()
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->stopped_ = true;
    this->nav_ = geometry_msgs::msg::Twist();
    this->rotation_wz_ = 0.0;
    this->nav_watchdog_.reset();
    this->rotation_watchdog_.reset();
    RCLCPP_WARN(this->get_logger(), "收到 stop_all：两路输入已清零并保持零输出。");
  }

  void resumeAll()
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->stopped_ = false;
    this->nav_ = geometry_msgs::msg::Twist();
    this->rotation_wz_ = 0.0;
    this->nav_watchdog_.reset();
    this->rotation_watchdog_.reset();
    RCLCPP_INFO(this->get_logger(), "收到 resume_all：等待两路输入重新下发速度。");
  }

  void guardLoop()
  {
    std::unique_lock<std::mutex> lock(this->mutex_);
    auto nextDiagnostics = std::chrono::steady_clock::now();
    while (!this->shutdown_) {
      this->condition_.wait_for(lock, this->publish_period_);
      if (this->shutdown_) {
        break;
      }
      const auto now = std::chrono::steady_clock::now();

      MixInput input;
      bool navExpired = false;
      bool rotationExpired = false;
      const bool stopped = this->stopped_;
      if (!this->stopped_) {
        navExpired = this->nav_watchdog_.expired(now);
        rotationExpired = this->rotation_watchdog_.expired(now);
        if (!navExpired) {
          input.nav_vx = this->nav_.linear.x;
          input.nav_vy = this->nav_.linear.y;
          // 导航输入的 angular.z 按约定丢弃，绝不进入合成结果。
        }
        if (!rotationExpired) {
          input.rotation_wz = this->rotation_wz_;
        }
      }
      const auto mixed = srm27_chassis_control::mixVelocity(input, this->limits_);

      if (navExpired && !this->nav_timeout_active_) {
        this->nav_timeout_active_ = true;
        RCLCPP_WARN(
          this->get_logger(), "[%s] 超过 %.3f s 未收到导航速度，平移已清零。",
          this->nav_topic_.c_str(), this->nav_timeout_);
      } else if (!navExpired) {
        this->nav_timeout_active_ = false;
      }
      if (rotationExpired && !this->rotation_timeout_active_) {
        this->rotation_timeout_active_ = true;
        RCLCPP_WARN(
          this->get_logger(), "[%s] 超过 %.3f s 未收到自转速度，自转已清零。",
          this->rotation_topic_.c_str(), this->rotation_timeout_);
      } else if (!rotationExpired) {
        this->rotation_timeout_active_ = false;
      }

      const auto stamp = this->now();
      lock.unlock();

      geometry_msgs::msg::Twist command;
      command.linear.x = mixed.vx;
      command.linear.y = mixed.vy;
      command.angular.z = mixed.wz;
      this->output_pub_->publish(command);

      if (now >= nextDiagnostics) {
        nextDiagnostics = now + std::chrono::duration_cast<
          std::chrono::steady_clock::duration>(this->diagnostics_period_);
        this->publishDiagnostics(
          mixed, navExpired, rotationExpired, stopped, stamp);
      }

      lock.lock();
    }
  }

  void publishDiagnostics(
    const srm27_chassis_control::MixOutput & _mixed, bool _navExpired,
    bool _rotationExpired, bool _stopped, const rclcpp::Time & _stamp)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = _stamp;

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "srm27_chassis_control: cmd_mux";
    status.hardware_id = this->get_namespace();
    if (_stopped) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "stop_all 生效中，输出保持为零";
    } else if (_navExpired && _rotationExpired) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "两路输入均超时，输出为零";
    } else if (_navExpired) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "导航输入超时，仅保留自转";
    } else if (_rotationExpired) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "自转输入超时，仅保留平移";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "速度合成正常";
    }

    const auto addKeyValue = [&status](
      const std::string & _key, const std::string & _value) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = _key;
        kv.value = _value;
        status.values.push_back(kv);
      };
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      addKeyValue("nav_vx", std::to_string(this->nav_.linear.x));
      addKeyValue("nav_vy", std::to_string(this->nav_.linear.y));
      addKeyValue("nav_wz_dropped_count", std::to_string(this->dropped_nav_yaw_count_));
      addKeyValue("non_finite_nav_count", std::to_string(this->non_finite_nav_count_));
      addKeyValue(
        "non_finite_rotation_count",
        std::to_string(this->non_finite_rotation_count_));
      addKeyValue("stopped", this->stopped_ ? "true" : "false");
    }
    addKeyValue("rotation_wz", std::to_string(_mixed.wz));
    addKeyValue("mixed_vx", std::to_string(_mixed.vx));
    addKeyValue("mixed_vy", std::to_string(_mixed.vy));
    addKeyValue("translation_scale", std::to_string(_mixed.scale));
    addKeyValue("clamped", _mixed.clamped ? "true" : "false");
    addKeyValue("nav_timeout", std::to_string(this->nav_timeout_));
    addKeyValue("rotation_timeout", std::to_string(this->rotation_timeout_));

    array.status.push_back(status);
    this->diagnostics_pub_->publish(array);
  }

  std::string nav_topic_;
  std::string rotation_topic_;
  std::string output_topic_;
  double nav_timeout_{0.3};
  double rotation_timeout_{0.1};
  MixLimits limits_;
  std::chrono::duration<double> publish_period_{0.005};
  std::chrono::duration<double> diagnostics_period_{0.5};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr rotation_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_service_;

  TwistWatchdog nav_watchdog_{0.3};
  TwistWatchdog rotation_watchdog_{0.1};

  std::mutex mutex_;
  std::condition_variable condition_;
  geometry_msgs::msg::Twist nav_;
  double rotation_wz_{0.0};
  bool stopped_{false};
  bool shutdown_{false};
  bool nav_timeout_active_{false};
  bool rotation_timeout_active_{false};
  uint64_t dropped_nav_yaw_count_{0};
  uint64_t non_finite_nav_count_{0};
  uint64_t non_finite_rotation_count_{0};
  std::thread guard_thread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdMuxNode>());
  rclcpp::shutdown();
  return 0;
}
