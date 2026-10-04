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

/// \brief SRM 仿真速度执行适配器。
///
/// 订阅 ROS 侧的最终速度命令 cmd_vel_sim（geometry_msgs/msg/Twist，车体系），
/// 校验后转换为 Gazebo Transport 的 Twist 发给 SRM 自有底盘速度执行插件。
///
/// 职责边界：
///   * 只做校验、转发和超时清零；不合成速度、不做限幅、不加轮速 PID。
///   * 收到非有限值时拒绝该条命令并按零速处理，绝不把 NaN/Inf 传给物理引擎。
///   * 守护线程用单调时钟检测输入中断，超时后清零并持续发布零速，
///     保证 mux 退出或崩溃后仿真底盘不会沿旧速度继续运动。

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

#include <ignition/msgs/twist.pb.h>
#include <ignition/transport/Node.hh>

namespace
{
bool allFinite(const geometry_msgs::msg::Twist &_msg)
{
  return std::isfinite(_msg.linear.x) && std::isfinite(_msg.linear.y) &&
         std::isfinite(_msg.linear.z) && std::isfinite(_msg.angular.x) &&
         std::isfinite(_msg.angular.y) && std::isfinite(_msg.angular.z);
}
}  // namespace

class SrmVelocityAdapter : public rclcpp::Node
{
public:
  SrmVelocityAdapter()
  : rclcpp::Node("srm_velocity_adapter")
  {
    this->declare_parameter<std::string>("robot_name", "");
    this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel_sim");
    this->declare_parameter<std::string>("gz_command_topic", "cmd_vel");
    this->declare_parameter<double>("command_timeout", 0.1);
    this->declare_parameter<double>("publish_rate", 200.0);

    std::string robotName = this->get_parameter("robot_name").as_string();
    if (robotName.empty())
    {
      robotName = this->get_namespace();
      while (!robotName.empty() && robotName.front() == '/')
      {
        robotName.erase(robotName.begin());
      }
      const auto slash = robotName.find('/');
      if (slash != std::string::npos)
      {
        robotName = robotName.substr(0, slash);
      }
    }
    if (robotName.empty())
    {
      RCLCPP_ERROR(
          this->get_logger(),
          "robot_name 为空且无法从命名空间推导，速度适配器不会发布任何命令。");
    }
    this->robotName_ = robotName;

    const auto gzTopic = this->get_parameter("gz_command_topic").as_string();
    this->gzTopic_ = robotName + "/" + gzTopic;
    this->commandTimeout_ = this->get_parameter("command_timeout").as_double();
    double publishRate = this->get_parameter("publish_rate").as_double();
    if (publishRate <= 0.0)
    {
      publishRate = 200.0;
    }
    this->publishPeriod_ =
        std::chrono::duration<double>(1.0 / publishRate);

    this->gzPub_ = this->gzNode_.Advertise<ignition::msgs::Twist>(
        this->gzTopic_);

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();
    const auto topic = this->get_parameter("cmd_vel_topic").as_string();
    this->subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        topic, qos,
        [this](const geometry_msgs::msg::Twist::SharedPtr _msg) {
          this->onCommand(*_msg);
        });

    this->diagnosticsPub_ =
        this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "diagnostics", rclcpp::QoS(10));

    this->lastRxSteady_ = std::chrono::steady_clock::now();
    this->watchdogThread_ = std::thread([this]() { this->watchdogLoop(); });

    RCLCPP_INFO(
        this->get_logger(),
        "SRM 速度适配器已启动: 订阅 [%s] -> 发布 Gazebo Transport [%s], "
        "命令超时 %.3f s, 发布频率 %.0f Hz",
        topic.c_str(), this->gzTopic_.c_str(), this->commandTimeout_,
        publishRate);
  }

  ~SrmVelocityAdapter() override
  {
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->shutdown_ = true;
      this->latest_ = geometry_msgs::msg::Twist();
      this->received_ = false;
    }
    this->condition_.notify_all();
    if (this->watchdogThread_.joinable())
    {
      this->watchdogThread_.join();
    }
    // 退出前明确把零速写下去，避免仿真里残留最后一次非零命令。
    this->publish(this->latest_);
  }

private:
  void onCommand(const geometry_msgs::msg::Twist &_msg)
  {
    if (!allFinite(_msg))
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "收到含 NaN/Inf 的速度命令，已按零速处理。");
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->latest_ = geometry_msgs::msg::Twist();
      this->received_ = true;
      this->lastRxSteady_ = std::chrono::steady_clock::now();
      this->nonFiniteCount_++;
      this->condition_.notify_all();
      return;
    }
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->latest_ = _msg;
    this->received_ = true;
    this->lastRxSteady_ = std::chrono::steady_clock::now();
    this->commandCount_++;
    this->condition_.notify_all();
  }

  void watchdogLoop()
  {
    std::unique_lock<std::mutex> lock(this->mutex_);
    while (!this->shutdown_)
    {
      this->condition_.wait_for(lock, this->publishPeriod_);
      if (this->shutdown_)
      {
        break;
      }
      const auto now = std::chrono::steady_clock::now();
      const std::chrono::duration<double> sinceRx = now - this->lastRxSteady_;
      // 单调时钟计时：输入中断（mux 退出、崩溃或暂停）后清零并持续发零速。
      const bool timedOut = this->received_ &&
          sinceRx.count() > this->commandTimeout_;
      auto command = this->latest_;
      if (timedOut)
      {
        command = geometry_msgs::msg::Twist();
        if (!this->timeoutActive_)
        {
          this->timeoutActive_ = true;
          RCLCPP_WARN(
              this->get_logger(),
              "速度命令超过 %.3f s 未更新，已清零底盘速度。",
              this->commandTimeout_);
        }
      }
      else
      {
        this->timeoutActive_ = false;
      }
      lock.unlock();
      this->publish(command);
      this->publishDiagnostics(timedOut);
      lock.lock();
    }
  }

  void publish(const geometry_msgs::msg::Twist &_command)
  {
    if (this->robotName_.empty())
    {
      return;
    }
    ignition::msgs::Twist msg;
    msg.mutable_linear()->set_x(_command.linear.x);
    msg.mutable_linear()->set_y(_command.linear.y);
    msg.mutable_linear()->set_z(_command.linear.z);
    msg.mutable_angular()->set_x(_command.angular.x);
    msg.mutable_angular()->set_y(_command.angular.y);
    msg.mutable_angular()->set_z(_command.angular.z);
    this->gzPub_.Publish(msg);
  }

  void publishDiagnostics(bool _timedOut)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = this->now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "srm27_gazebo_simulator: velocity_adapter";
    status.hardware_id = this->robotName_;
    if (this->robotName_.empty())
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "robot_name 未配置";
    }
    else if (_timedOut)
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "输入超时，速度已清零";
    }
    else
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "速度转发正常";
    }
    auto addKeyValue = [&status](const std::string &_key,
                                 const std::string &_value) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = _key;
        kv.value = _value;
        status.values.push_back(kv);
      };
    addKeyValue("gz_topic", this->gzTopic_);
    addKeyValue("command_timeout", std::to_string(this->commandTimeout_));
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      addKeyValue("commands", std::to_string(this->commandCount_));
      addKeyValue("non_finite_commands", std::to_string(this->nonFiniteCount_));
    }
    array.status.push_back(status);
    this->diagnosticsPub_->publish(array);
  }

  ignition::transport::Node gzNode_;
  ignition::transport::Node::Publisher gzPub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnosticsPub_;

  std::string robotName_;
  std::string gzTopic_;
  double commandTimeout_{0.1};
  std::chrono::duration<double> publishPeriod_{0.005};

  std::mutex mutex_;
  std::condition_variable condition_;
  geometry_msgs::msg::Twist latest_;
  bool received_{false};
  bool timeoutActive_{false};
  bool shutdown_{false};
  std::chrono::steady_clock::time_point lastRxSteady_;
  uint64_t commandCount_{0};
  uint64_t nonFiniteCount_{0};
  std::thread watchdogThread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SrmVelocityAdapter>());
  rclcpp::shutdown();
  return 0;
}
