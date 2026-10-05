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

/// \brief rotation_test_sender：独立自转测试发送器。
///
/// 恒速、周期（正弦/方波）与停止三种模式由 launch / YAML 参数决定，速度统一用
/// geometry_msgs/msg/Twist 的 angular.z 发布，不新增消息类型，也不把波形参数
/// 塞进 Twist 的其他分量。
///
/// 波形时间取仿真时间，从"启用"或"波形参数改变"时开始计算；周期性发布本身
/// 不重置相位，因此换向与变速的周期只由 period 决定。
/// 拒绝非有限速度、负幅度和周期模式下的非正周期。

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "srm27_chassis_control/rotation_waveform.hpp"

namespace
{
const char * kWaveformParameters[] = {
  "rotation_mode", "angular_speed", "offset", "amplitude",
  "period", "phase", "sine_wave",
};
}  // namespace

class RotationTestSenderNode : public rclcpp::Node
{
public:
  RotationTestSenderNode()
  : rclcpp::Node("rotation_test_sender")
  {
    this->output_topic_ =
      this->declare_parameter<std::string>("output_topic", "rotation_cmd");
    const double publishRate =
      this->declare_parameter<double>("publish_rate", 200.0);
    this->enabled_ = this->declare_parameter<bool>("enabled", true);
    this->diagnostics_period_ = std::chrono::duration<double>(
      this->declare_parameter<double>("diagnostics_period", 0.5));

    this->readWaveformParameters();

    std::string error;
    if (!srm27_chassis_control::validateWaveform(this->params_, &error)) {
      RCLCPP_ERROR(
        this->get_logger(), "自转波形参数非法：%s，发送器保持零输出。",
        error.c_str());
      this->valid_ = false;
    }

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();
    this->publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>(this->output_topic_, qos);
    this->diagnostics_pub_ =
      this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "diagnostics", rclcpp::QoS(10));

    this->enable_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/enable",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> _response) {
        const std::string message = this->setEnabled(true);
        _response->success = this->valid_;
        _response->message = message;
      });
    this->disable_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/disable",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> _response) {
        const std::string message = this->setEnabled(false);
        _response->success = true;
        _response->message = message;
      });
    this->reset_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/reset_waveform",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> _response) {
        {
          std::lock_guard<std::mutex> lock(this->mutex_);
          this->waveform_epoch_ = this->now();
          this->has_epoch_ = true;
        }
        _response->success = true;
        _response->message = "波形相位已从头开始";
      });

    this->parameter_callback_ =
      this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & _parameters) {
        return this->onParameterChange(_parameters);
      });

    this->timer_ = this->create_wall_timer(
      std::chrono::duration<double>(
        publishRate > 0.0 ? 1.0 / publishRate : 0.005),
      [this]() {this->onTimer();});

    RCLCPP_INFO(
      this->get_logger(),
      "rotation_test_sender 已启动: 模式 [%s] -> [%s], 启用状态 %s",
      srm27_chassis_control::toString(this->params_.mode).c_str(),
      this->output_topic_.c_str(), this->enabled_ ? "true" : "false");
  }

private:
  void readWaveformParameters()
  {
    const auto modeText =
      this->declare_parameter<std::string>("rotation_mode", "stop");
    srm27_chassis_control::RotationMode mode =
      srm27_chassis_control::RotationMode::kStop;
    if (!srm27_chassis_control::parseRotationMode(modeText, mode)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "rotation_mode '%s' 无法识别，按 stop 处理（可选 stop/constant/periodic）。",
        modeText.c_str());
      this->valid_ = false;
    }
    this->params_.mode = mode;
    this->params_.angular_speed =
      this->declare_parameter<double>("angular_speed", 0.0);
    this->params_.offset = this->declare_parameter<double>("offset", 0.0);
    this->params_.amplitude = this->declare_parameter<double>("amplitude", 0.0);
    this->params_.period = this->declare_parameter<double>("period", 4.0);
    this->params_.phase = this->declare_parameter<double>("phase", 0.0);
    this->params_.sine_wave = this->declare_parameter<bool>("sine_wave", true);
  }

  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & _parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    bool touchesWaveform = false;
    for (const auto & parameter : _parameters) {
      for (const auto * name : kWaveformParameters) {
        if (parameter.get_name() == name) {
          touchesWaveform = true;
        }
      }
    }

    if (touchesWaveform) {
      // 先在副本上应用并校验，避免把非法参数写进运行状态。
      auto candidate = this->params_;
      {
        std::lock_guard<std::mutex> lock(this->mutex_);
        candidate = this->params_;
      }
      for (const auto & parameter : _parameters) {
        const auto & name = parameter.get_name();
        if (name == "rotation_mode") {
          srm27_chassis_control::RotationMode mode;
          if (!srm27_chassis_control::parseRotationMode(
              parameter.as_string(), mode))
          {
            result.successful = false;
            result.reason = "rotation_mode 必须是 stop/constant/periodic";
            return result;
          }
          candidate.mode = mode;
        } else if (name == "angular_speed") {
          candidate.angular_speed = parameter.as_double();
        } else if (name == "offset") {
          candidate.offset = parameter.as_double();
        } else if (name == "amplitude") {
          candidate.amplitude = parameter.as_double();
        } else if (name == "period") {
          candidate.period = parameter.as_double();
        } else if (name == "phase") {
          candidate.phase = parameter.as_double();
        } else if (name == "sine_wave") {
          candidate.sine_wave = parameter.as_bool();
        }
      }

      std::string error;
      if (!srm27_chassis_control::validateWaveform(candidate, &error)) {
        result.successful = false;
        result.reason = error;
        return result;
      }

      {
        std::lock_guard<std::mutex> lock(this->mutex_);
        this->params_ = candidate;
        // 波形参数改变后从当前仿真时间重新开始计算相位。
        this->waveform_epoch_ = this->now();
        this->has_epoch_ = true;
      }
      this->valid_ = true;
      RCLCPP_INFO(
        this->get_logger(), "自转波形参数已更新: 模式 [%s]，相位重新开始。",
        srm27_chassis_control::toString(candidate.mode).c_str());
    }
    return result;
  }

  std::string setEnabled(bool _enabled)
  {
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      if (_enabled && !this->valid_) {
        return "波形参数非法，拒绝启用";
      }
      const bool wasEnabled = this->enabled_;
      this->enabled_ = _enabled;
      // "先清空旧命令，由新的启用操作开始输出"：重新启用时相位从零开始，
      // 且不沿用暂停前的任何输出。
      this->waveform_epoch_ = this->now();
      this->has_epoch_ = true;
      this->last_output_ = 0.0;
      RCLCPP_INFO(
        this->get_logger(), "%s 自转发送器（之前为 %s）。",
        _enabled ? "启用" : "暂停", wasEnabled ? "启用" : "暂停");
    }
    if (!_enabled) {
      // 暂停时停止发布：接收侧的超时守护线程会把自转速度清零。
      geometry_msgs::msg::Twist zero;
      this->publisher_->publish(zero);
    }
    return _enabled ? "已启用自转发送器，相位从零开始" : "已暂停自转发送器";
  }

  void onTimer()
  {
    srm27_chassis_control::RotationWaveformParams params;
    bool enabled = false;
    bool valid = false;
    rclcpp::Time epoch;
    bool hasEpoch = false;
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      params = this->params_;
      enabled = this->enabled_;
      valid = this->valid_;
      epoch = this->waveform_epoch_;
      hasEpoch = this->has_epoch_;
    }

    if (!enabled || !valid) {
      return;
    }
    if (!hasEpoch) {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->waveform_epoch_ = this->now();
      this->has_epoch_ = true;
      return;
    }

    // 使用仿真时间：周期性发布本身不重置相位，暂停时相位自然冻结。
    const double t = (this->now() - epoch).seconds();
    const double wz = srm27_chassis_control::computeRotationVelocity(params, t);

    geometry_msgs::msg::Twist command;
    command.angular.z = std::isfinite(wz) ? wz : 0.0;
    this->publisher_->publish(command);

    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->last_output_ = command.angular.z;
      this->last_phase_time_ = t;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now >= this->next_diagnostics_) {
      this->next_diagnostics_ = now + std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(this->diagnostics_period_);
      this->publishDiagnostics(params, t, command.angular.z, enabled);
    }
  }

  void publishDiagnostics(
    const srm27_chassis_control::RotationWaveformParams & _params, double _t,
    double _wz, bool _enabled)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = this->now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "srm27_chassis_control: rotation_test_sender";
    status.hardware_id = this->get_namespace();
    if (!this->valid_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "波形参数非法，未输出自转速度";
    } else if (!_enabled) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "发送器已暂停";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "自转波形输出正常";
    }
    const auto addKeyValue = [&status](
      const std::string & _key, const std::string & _value) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = _key;
        kv.value = _value;
        status.values.push_back(kv);
      };
    addKeyValue("mode", srm27_chassis_control::toString(_params.mode));
    addKeyValue("angular_speed", std::to_string(_params.angular_speed));
    addKeyValue("offset", std::to_string(_params.offset));
    addKeyValue("amplitude", std::to_string(_params.amplitude));
    addKeyValue("period", std::to_string(_params.period));
    addKeyValue("phase", std::to_string(_params.phase));
    addKeyValue("sine_wave", _params.sine_wave ? "true" : "false");
    addKeyValue("waveform_time", std::to_string(_t));
    addKeyValue("rotation_wz", std::to_string(_wz));
    array.status.push_back(status);
    this->diagnostics_pub_->publish(array);
  }

  std::string output_topic_;
  srm27_chassis_control::RotationWaveformParams params_;
  bool enabled_{true};
  bool valid_{true};
  std::chrono::duration<double> diagnostics_period_{0.5};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    parameter_callback_;

  std::mutex mutex_;
  rclcpp::Time waveform_epoch_{0, 0, RCL_ROS_TIME};
  bool has_epoch_{false};
  double last_output_{0.0};
  double last_phase_time_{0.0};
  std::chrono::steady_clock::time_point next_diagnostics_{
    std::chrono::steady_clock::now()};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RotationTestSenderNode>());
  rclcpp::shutdown();
  return 0;
}
