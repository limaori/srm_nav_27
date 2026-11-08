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

/// \brief srm_cmd_mux：导航速度的唯一出口（两路合成 + 限幅 + 超时清零 + 急停）。
///
/// 输入（2026-11-08 起两路，方案 §4.3）：
///   <nav_topic>    导航速度（geometry_msgs/msg/Twist，云台系），**只取 linear.x/linear.y**；
///                  它的 angular.z 一律忽略并计数告警（正常应为 0）。
///   <spin_topic>   决策层底盘自转（geometry_msgs/msg/Twist），**只取 angular.z**；
///                  停发 spin_timeout 即视为停转（wz 清零）。
/// 输出：
///   <output_topic> 最终执行命令（mux 是唯一发布者）
///
/// `spin_topic` 留空 = **单输入**（旧行为）：导航同时拥有平移与 wz，用于仿真链与回退。
///
/// 两路各自计时、互不影响：导航停了决策层还能让底盘原地转，决策层停发导航照常走。
/// `wz` 还带变化率限制（`wz_ramp`，默认与 MINCO 的 max_angular_accel 同档 4.5 rad/s²），
/// 避免决策层的阶跃请求在起停时冲击底盘；`stop_all` 例外 —— 它立即清零、不做斜率限制。
///
/// 限幅（vx_max / vy_max / v_max / wz_max）仍由 config/{real,simulation}/speed_limits.yaml
/// 在启动时合并进本包的参数（唯一来源）。

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
    this->spin_topic_ = this->declare_parameter<std::string>(
      "spin_topic", "rotation_cmd");
    this->output_topic_ = this->declare_parameter<std::string>(
      "output_topic", "cmd_vel_sim");
    const double publishRate =
      this->declare_parameter<double>("publish_rate", 200.0);
    this->nav_timeout_ = this->declare_parameter<double>("nav_timeout", 0.3);
    this->spin_timeout_ = this->declare_parameter<double>("spin_timeout", 0.25);
    this->limits_.vx_max = this->declare_parameter<double>("vx_max", 0.5);
    this->limits_.vy_max = this->declare_parameter<double>("vy_max", 0.5);
    this->limits_.v_max = this->declare_parameter<double>("v_max", 0.5);
    this->limits_.wz_max = this->declare_parameter<double>("wz_max", 2.0);
    // 与 speed_limits.yaml 的 FollowPath.limits.max_angular_accel 同档（4.5）。
    this->limits_.wz_ramp = this->declare_parameter<double>("wz_ramp", 4.5);
    this->diagnostics_period_ = std::chrono::duration<double>(
      this->declare_parameter<double>("diagnostics_period", 0.5));

    if (!(this->spin_timeout_ > 0.0) || !std::isfinite(this->spin_timeout_)) {
      RCLCPP_FATAL(get_logger(), "spin_timeout 必须为正的有限值");
      throw std::invalid_argument("spin_timeout must be finite and positive");
    }
    if (!(this->limits_.wz_ramp >= 0.0) || !std::isfinite(this->limits_.wz_ramp)) {
      RCLCPP_FATAL(get_logger(), "wz_ramp 必须是非负的有限值（0 = 不做斜率限制）");
      throw std::invalid_argument("wz_ramp must be finite and non-negative");
    }

    this->nav_watchdog_.setTimeout(this->nav_timeout_);
    this->spin_watchdog_.setTimeout(this->spin_timeout_);
    this->dual_input_ = !this->spin_topic_.empty();

    this->publish_period_ = std::chrono::duration<double>(
      publishRate > 0.0 ? 1.0 / publishRate : 0.005);

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();
    this->nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      this->nav_topic_, qos,
      [this](const geometry_msgs::msg::Twist::SharedPtr _msg) {
        this->onNavCommand(*_msg);
      });
    if (this->dual_input_) {
      this->spin_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        this->spin_topic_, qos,
        [this](const geometry_msgs::msg::Twist::SharedPtr _msg) {
          this->onSpinCommand(*_msg);
        });
    }
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
        _response->message = "已停止导航与自转输入，保持零输出直到 resume_all";
      });
    this->resume_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/resume_all",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> _response) {
        this->resumeAll();
        _response->success = true;
        _response->message = "已重新启用导航与自转输入";
      });

    this->guard_thread_ = std::thread([this]() {this->guardLoop();});

    if (this->dual_input_) {
      RCLCPP_INFO(
        this->get_logger(),
        "srm_cmd_mux 已启动（双输入）: [%s] 平移 -> [%s]，[%s] wz -> [%s]\n"
        "  平移上限 %.3f m/s (v_max %.3f)，wz 上限 %.3f rad/s，wz 斜率 %.2f rad/s²\n"
        "  导航超时 %.3f s（平移清零），自转超时 %.3f s（wz 清零）",
        this->nav_topic_.c_str(), this->output_topic_.c_str(), this->spin_topic_.c_str(),
        this->output_topic_.c_str(), this->limits_.vx_max, this->limits_.v_max,
        this->limits_.wz_max, this->limits_.wz_ramp, this->nav_timeout_, this->spin_timeout_);
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "srm_cmd_mux 已启动（单输入）: [%s] -> [%s]（平移与 wz 都来自导航）, "
        "平移上限 %.3f m/s (v_max %.3f), wz 上限 %.3f rad/s, 导航超时 %.3f s",
        this->nav_topic_.c_str(), this->output_topic_.c_str(),
        this->limits_.vx_max, this->limits_.v_max,
        this->limits_.wz_max, this->nav_timeout_);
    }
  }

  ~CmdMuxNode() override
  {
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->shutdown_ = true;
      this->nav_ = geometry_msgs::msg::Twist();
      this->spin_wz_ = 0.0;
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
        // 平移原样保留；wz 在双输入模式下由 mixVelocity 忽略（这里先存下来，
        // 由守护线程统一判定并计数，避免在回调线程里做告警限流）。
        this->nav_ = _msg;
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

  /// \brief 决策层自转输入：只取 `angular.z`（方案 §4.3 / §8.2）。
  void onSpinCommand(const geometry_msgs::msg::Twist & _msg)
  {
    const auto now = std::chrono::steady_clock::now();
    const double wz = _msg.angular.z;
    bool rejected = false;
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      // 与导航一致：stop_all 期间仍然计时，但输出保持为零。
      this->spin_watchdog_.markInput(now);
      this->spin_input_count_++;
      if (std::isfinite(wz)) {
        this->spin_wz_ = wz;
      } else {
        this->spin_wz_ = 0.0;
        this->non_finite_spin_count_++;
        rejected = true;
      }
    }
    if (rejected) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "[%s] 收到含 NaN/Inf 的自转请求，已按零处理。",
        this->spin_topic_.c_str());
    }
  }

  void stopAll()
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->stopped_ = true;
    this->nav_ = geometry_msgs::msg::Twist();
    this->spin_wz_ = 0.0;
    this->nav_watchdog_.reset();
    this->spin_watchdog_.reset();
    // 急停不做斜率限制：输出立即归零。
    this->wz_limited_ = 0.0;
    this->wz_limited_valid_ = true;
    RCLCPP_WARN(
      this->get_logger(), "收到 stop_all：导航与自转输入已清零并保持零输出。");
  }

  void resumeAll()
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->stopped_ = false;
    this->nav_ = geometry_msgs::msg::Twist();
    this->spin_wz_ = 0.0;
    this->nav_watchdog_.reset();
    this->spin_watchdog_.reset();
    RCLCPP_INFO(this->get_logger(), "收到 resume_all：等待导航/自转重新下发速度。");
  }

  void guardLoop()
  {
    std::unique_lock<std::mutex> lock(this->mutex_);
    auto nextDiagnostics = std::chrono::steady_clock::now();
    auto lastOutput = std::chrono::steady_clock::now();
    while (!this->shutdown_) {
      this->condition_.wait_for(lock, this->publish_period_);
      if (this->shutdown_) {
        break;
      }
      const auto now = std::chrono::steady_clock::now();

      MixInput input;
      bool navExpired = false;
      bool spinExpired = false;
      const bool stopped = this->stopped_;
      if (!this->stopped_) {
        navExpired = this->nav_watchdog_.expired(now);
        if (!navExpired) {
          input.nav_vx = this->nav_.linear.x;
          input.nav_vy = this->nav_.linear.y;
          input.nav_wz = this->nav_.angular.z;
        }
        if (this->dual_input_) {
          input.spin_enabled = true;
          spinExpired = this->spin_watchdog_.expired(now);
          input.spin_valid = !spinExpired;
          input.spin_wz = spinExpired ? 0.0 : this->spin_wz_;
        }
      } else if (this->dual_input_) {
        input.spin_enabled = true;
        input.spin_valid = false;
      }

      auto mixed = srm27_chassis_control::mixVelocity(input, this->limits_);

      // ---- wz 变化率限制（方案 §4.3：避免决策层的阶跃请求冲击底盘）----
      double wzOut = mixed.wz;
      const double dt = std::chrono::duration<double>(now - lastOutput).count();
      lastOutput = now;
      if (stopped) {
        // 急停/退出：立即归零，不做斜率限制。
        this->wz_limited_ = 0.0;
        this->wz_limited_valid_ = true;
        wzOut = 0.0;
        mixed.wz = 0.0;
        this->wz_ramp_active_ = false;
      } else if (this->limits_.wz_ramp > 0.0) {
        if (!this->wz_limited_valid_) {
          // 第一拍以"当前请求"为起点，避免启动瞬间从一个凭空的速度开始爬。
          this->wz_limited_ = mixed.wz;
          this->wz_limited_valid_ = true;
        } else {
          this->wz_limited_ = srm27_chassis_control::slewLimit(
            mixed.wz, this->wz_limited_, this->limits_.wz_ramp, dt);
        }
        wzOut = this->wz_limited_;
        this->wz_ramp_active_ = std::abs(wzOut - mixed.wz) > 1.0e-9;
      } else {
        this->wz_limited_ = mixed.wz;
        this->wz_limited_valid_ = true;
        this->wz_ramp_active_ = false;
      }

      if (mixed.wz_clamped) {
        if (!this->wz_clamp_active_) {
          this->wz_clamp_active_ = true;
          this->nav_wz_clamped_count_++;
          RCLCPP_WARN(
            this->get_logger(),
            "wz 被限幅到 %.3f rad/s（wz_max=%.3f，来源=%s）",
            mixed.wz, this->limits_.wz_max,
            mixed.wz_from_spin ? this->spin_topic_.c_str() : this->nav_topic_.c_str());
        }
      } else {
        this->wz_clamp_active_ = false;
      }

      // 导航漏出 wz：双输入模式下必须忽略并告警（方案 §4.3）。
      if (mixed.nav_wz_ignored) {
        if (!this->nav_wz_ignored_active_) {
          this->nav_wz_ignored_active_ = true;
          this->nav_wz_ignored_count_++;
          RCLCPP_WARN(
            this->get_logger(),
            "导航输入带了非零 angular.z=%.3f，已忽略（底盘 wz 只认 [%s]；"
            "导航侧应为 xy_only，见方案 §4.2）",
            input.nav_wz, this->spin_topic_.c_str());
        }
      } else {
        this->nav_wz_ignored_active_ = false;
      }

      if (navExpired && !this->nav_timeout_active_) {
        this->nav_timeout_active_ = true;
        RCLCPP_WARN(
          this->get_logger(), "[%s] 超过 %.3f s 未收到导航速度，平移已清零。",
          this->nav_topic_.c_str(), this->nav_timeout_);
      } else if (!navExpired) {
        this->nav_timeout_active_ = false;
      }
      if (this->dual_input_) {
        if (spinExpired && !this->spin_timeout_active_) {
          this->spin_timeout_active_ = true;
          this->spin_timeout_count_++;
          RCLCPP_WARN(
            this->get_logger(), "[%s] 超过 %.3f s 未收到自转请求，底盘停转（wz=0）。",
            this->spin_topic_.c_str(), this->spin_timeout_);
        } else if (!spinExpired) {
          this->spin_timeout_active_ = false;
        }
      }

      const auto stamp = this->now();
      const uint64_t spinCount = this->spin_input_count_;
      const double spinWzInput = this->spin_wz_;
      lock.unlock();

      geometry_msgs::msg::Twist command;
      command.linear.x = mixed.vx;
      command.linear.y = mixed.vy;
      command.angular.z = wzOut;
      this->output_pub_->publish(command);

      if (now >= nextDiagnostics) {
        nextDiagnostics = now + std::chrono::duration_cast<
          std::chrono::steady_clock::duration>(this->diagnostics_period_);
        this->publishDiagnostics(
          mixed, wzOut, navExpired, spinExpired, stopped, spinCount, spinWzInput, stamp);
      }

      lock.lock();
    }
  }

  void publishDiagnostics(
    const srm27_chassis_control::MixOutput & _mixed, double _wzOut, bool _navExpired,
    bool _spinExpired, bool _stopped, uint64_t _spinCount, double _spinWzInput,
    const rclcpp::Time & _stamp)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = _stamp;

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "srm27_chassis_control: cmd_mux";
    status.hardware_id = this->get_namespace();
    if (_stopped) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "stop_all 生效中，输出保持为零";
    } else if (_navExpired && this->dual_input_ && _spinExpired) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "导航与自转输入都超时，平移与 wz 均已清零";
    } else if (_navExpired) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = this->dual_input_ ?
        "导航输入超时，平移已清零（自转仍由 rotation_cmd 提供）" :
        "导航输入超时，平移与 wz 均已清零";
    } else if (this->dual_input_ && _spinExpired) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "自转输入超时（wz=0，底盘停转），导航照常输出平移";
    } else if (this->dual_input_ && this->nav_wz_ignored_active_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "导航漏出 wz，已忽略（底盘 wz 只认自转输入）";
    } else if (_mixed.wz_clamped) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "wz 被 wz_max 限幅";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = this->dual_input_ ?
        "平移来自导航、wz 来自决策层" : "导航同时拥有平移与 wz（单输入）";
    }

    const auto addKeyValue = [&status](
      const std::string & _key, const std::string & _value) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = _key;
        kv.value = _value;
        status.values.push_back(kv);
      };
    if (this->dual_input_) {
      addKeyValue("wz_source", "spin_input");
    } else {
      // 单输入回退时保留旧值，便于旧脚本对照。
      addKeyValue("wz_source", "navigation");
    }
    addKeyValue("dual_input", this->dual_input_ ? "true" : "false");
    addKeyValue("nav_topic", this->nav_topic_);
    addKeyValue("spin_topic", this->spin_topic_);
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      addKeyValue("nav_vx", std::to_string(this->nav_.linear.x));
      addKeyValue("nav_vy", std::to_string(this->nav_.linear.y));
      addKeyValue("nav_wz_input", std::to_string(this->nav_.angular.z));
      addKeyValue("nav_wz_clamped_count", std::to_string(this->nav_wz_clamped_count_));
      addKeyValue("non_finite_nav_count", std::to_string(this->non_finite_nav_count_));
      addKeyValue("stopped", this->stopped_ ? "true" : "false");
    }
    if (this->dual_input_) {
      // 决策层自转输入的频率/超时/漏出统计（方案 §4.3 要求 diagnostics 里能看到）。
      const double spin_freq = (_stamp - this->last_diagnostics_stamp_).seconds() > 0.0 ?
        static_cast<double>(_spinCount - this->last_spin_count_) /
        (_stamp - this->last_diagnostics_stamp_).seconds() :
        0.0;
      this->last_spin_count_ = _spinCount;
      this->last_diagnostics_stamp_ = _stamp;
      addKeyValue("spin_wz_input", std::to_string(_spinWzInput));
      addKeyValue("spin_freq_hz", std::to_string(spin_freq));
      addKeyValue("spin_valid", _spinExpired ? "false" : "true");
      addKeyValue("spin_timeout", std::to_string(this->spin_timeout_));
      addKeyValue("spin_timeout_count", std::to_string(this->spin_timeout_count_));
      addKeyValue("spin_input_count", std::to_string(_spinCount));
      addKeyValue("non_finite_spin_count", std::to_string(this->non_finite_spin_count_));
      addKeyValue("nav_wz_ignored_count", std::to_string(this->nav_wz_ignored_count_));
    }
    addKeyValue("mixed_vx", std::to_string(_mixed.vx));
    addKeyValue("mixed_vy", std::to_string(_mixed.vy));
    addKeyValue("mixed_wz", std::to_string(_mixed.wz));
    addKeyValue("output_wz", std::to_string(_wzOut));
    addKeyValue("wz_clamped", _mixed.wz_clamped ? "true" : "false");
    addKeyValue("wz_ramp", std::to_string(this->limits_.wz_ramp));
    addKeyValue("wz_ramp_active", this->wz_ramp_active_ ? "true" : "false");
    addKeyValue("translation_scale", std::to_string(_mixed.scale));
    addKeyValue("clamped", _mixed.clamped ? "true" : "false");
    addKeyValue("wz_max", std::to_string(this->limits_.wz_max));
    addKeyValue("nav_timeout", std::to_string(this->nav_timeout_));

    array.status.push_back(status);
    this->diagnostics_pub_->publish(array);
  }

  std::string nav_topic_;
  std::string spin_topic_;
  std::string output_topic_;
  /// \brief 是否启用决策层自转输入（`spin_topic` 非空）。
  bool dual_input_{false};
  double nav_timeout_{0.3};
  double spin_timeout_{0.25};
  MixLimits limits_;
  std::chrono::duration<double> publish_period_{0.005};
  std::chrono::duration<double> diagnostics_period_{0.5};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr spin_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_service_;

  TwistWatchdog nav_watchdog_{0.3};
  TwistWatchdog spin_watchdog_{0.25};

  std::mutex mutex_;
  std::condition_variable condition_;
  geometry_msgs::msg::Twist nav_;
  double spin_wz_{0.0};
  bool stopped_{false};
  bool shutdown_{false};
  bool nav_timeout_active_{false};
  bool spin_timeout_active_{false};
  bool wz_clamp_active_{false};
  bool nav_wz_ignored_active_{false};
  bool wz_ramp_active_{false};
  /// \brief `wz` 斜率限制的状态（上一拍输出的 wz）。
  double wz_limited_{0.0};
  bool wz_limited_valid_{false};
  uint64_t nav_wz_clamped_count_{0};
  uint64_t non_finite_nav_count_{0};
  uint64_t nav_wz_ignored_count_{0};
  uint64_t spin_input_count_{0};
  uint64_t non_finite_spin_count_{0};
  uint64_t spin_timeout_count_{0};
  uint64_t last_spin_count_{0};
  rclcpp::Time last_diagnostics_stamp_{0, 0, RCL_ROS_TIME};
  std::thread guard_thread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdMuxNode>());
  rclcpp::shutdown();
  return 0;
}
