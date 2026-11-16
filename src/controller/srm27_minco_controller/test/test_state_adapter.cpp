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

#include <gtest/gtest.h>
#include <tf2/utils.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

#include "srm27_minco_controller/state_adapter.hpp"

/// \file
/// \brief 状态适配层的集成测试（真实节点 + 真实话题 + 真实 TF 查询）。
///
/// 覆盖方案 §2.3 与 §5.2 的关键契约：
///  * 差分速度使用**消息采样时间差**，不是回调到达时间差（仿真倍速/暂停/回放）；
///  * `twist` 表达在 `child_frame_id` 下，进入 odom 系前必须显式旋转；
///  * 重复/倒退时间戳、过大间隔、位姿跳变都不产生速度尖峰；
///  * TF 查询失败时**不会**退化成单位变换伪装成有效定位。

namespace
{

/// \brief 在全部用例之前初始化 rclcpp、之后关闭，保证全局 context 有效。
///
/// 本文件使用 gtest_main，因此通过全局测试环境而不是自定义 main 来初始化。
class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
  void TearDown() override
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

const bool kRclcppEnvironmentRegistered = []() {
  ::testing::AddGlobalTestEnvironment(new RclcppEnvironment());
  return true;
}();

double steadyNow()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::steady_clock::now().time_since_epoch())
           .count() *
         1.0e-9;
}

rclcpp::Time stampFromSeconds(double _seconds)
{
  return rclcpp::Time(static_cast<std::int64_t>(std::llround(_seconds * 1.0e9)), RCL_ROS_TIME);
}

/// \brief 测试夹具：一个真实 LifecycleNode + 单线程执行器 + 里程计发布者。
class StateAdapterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("use_sim_time", false)});
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("state_adapter_test", options);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>("odometry", 10);

    // 不使用后台 spin 线程：由测试线程显式 spin_some()，时序完全确定，
    // 也不会出现“取消后 executor 不返回”的挂死风险。
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
    // 注意：不依赖 get_subscription_count()。DDS 发现与匹配计数在不同 RMW 上时序不同，
    // 真正的就绪信号是适配层收到了采样，由 waitForSample() 负责等待。
  }

  void TearDown() override
  {
    adapter_.reset();
    publisher_.reset();
    executor_.reset();
    node_.reset();
  }

  /// \brief 建立适配层；`velocity_filter_alpha = 1` 表示关闭低通，便于精确断言。
  bool configureAdapter(
    const std::string & _planning_frame = "odom", const std::string & _odom_topic = "odometry")
  {
    srm27_minco_controller::StateAdapter::Config config;
    config.odometry_topic = _odom_topic;
    config.base_frame = "base_link";
    config.planning_frame = _planning_frame;
    config.timeout = 0.10;
    config.max_sample_gap = 0.20;
    config.max_position_jump = 0.75;
    config.max_yaw_jump = 1.20;
    config.twist_in_child_frame = true;
    config.use_tf = true;
    config.velocity_filter_alpha = 1.0;
    return adapter_.configure(node_, tf_buffer_, config, &reason_);
  }

  /// \brief 发布一条里程计消息（不等待接收，用于乱序采样）。
  void publishRaw(
    double _stamp, double _x, double _y, double _yaw, double _vx_child = 0.0,
    double _vy_child = 0.0, double _omega = 0.0)
  {
    nav_msgs::msg::Odometry message;
    message.header.stamp = stampFromSeconds(_stamp);
    message.header.frame_id = "odom";
    message.child_frame_id = "base_link";
    message.pose.pose.position.x = _x;
    message.pose.pose.position.y = _y;
    message.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), _yaw));
    message.twist.twist.linear.x = _vx_child;
    message.twist.twist.linear.y = _vy_child;
    message.twist.twist.angular.z = _omega;
    publisher_->publish(message);
  }

  /// \brief 发布一条里程计消息并等待适配层接收。
  void publish(
    double _stamp, double _x, double _y, double _yaw, double _vx_child = 0.0,
    double _vy_child = 0.0, double _omega = 0.0)
  {
    nav_msgs::msg::Odometry message;
    message.header.stamp = stampFromSeconds(_stamp);
    message.header.frame_id = "odom";
    message.child_frame_id = "base_link";
    message.pose.pose.position.x = _x;
    message.pose.pose.position.y = _y;
    message.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), _yaw));
    message.twist.twist.linear.x = _vx_child;
    message.twist.twist.linear.y = _vy_child;
    message.twist.twist.angular.z = _omega;
    publisher_->publish(message);
    waitForSample(_stamp);
  }

  /// \brief 等待适配层真正收到该时间戳的采样。
  void waitForSample(double _stamp)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      if (std::abs(adapter_.lastSampleStamp() - _stamp) < 1.0e-9) {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ADD_FAILURE() << "等待里程计采样超时: " << _stamp;
  }

  /// \brief 空转执行器一段时间，让已经发布的消息被处理。
  void spinFor(std::chrono::milliseconds _duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + _duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_{};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_{};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_{};
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_{};
  srm27_minco_controller::StateAdapter adapter_{};
  std::string reason_{};
};

}  // namespace

TEST_F(StateAdapterTest, RejectsUseBeforeConfigure)
{
  srm27_minco_controller::StateAdapter adapter;
  srm27_minco_core::State2D state;
  std::string reason;
  EXPECT_FALSE(adapter.stateAt(1.0, steadyNow(), state, &reason));
  EXPECT_FALSE(reason.empty());
  EXPECT_FALSE(state.valid);
}

TEST_F(StateAdapterTest, DifferenceUsesSampleStampNotArrivalTime)
{
  ASSERT_TRUE(configureAdapter()) << reason_;
  // 两条消息的采样时间相差 0.1 s，但**发布间隔**远远大于 0.1 s。
  // 按到达时间差分会把 0.05 m 除以约 0.5 s，得到 ~0.1 m/s（旧实现的错误结果）；
  // 按采样时间差分应得到 0.5 m/s。
  publish(10.000, 0.0, 0.0, 0.0);
  // 故意让“到达时间”远大于“采样时间”，若实现误用到达时间会得到 ~0.1 m/s 而不是 0.5 m/s。
  spinFor(std::chrono::milliseconds(500));
  publish(10.100, 0.05, 0.0, 0.0);

  srm27_minco_core::State2D state;
  ASSERT_TRUE(adapter_.stateAt(10.100, steadyNow(), state, &reason_)) << reason_;
  EXPECT_NEAR(state.velocity.x(), 0.5, 1.0e-6)
    << "差分必须使用消息采样时间差；实测 " << state.velocity.x();
  EXPECT_NEAR(state.velocity.y(), 0.0, 1.0e-9);
  EXPECT_NEAR(state.sample_stamp, 10.100, 1.0e-9);
}

TEST_F(StateAdapterTest, ChildFrameTwistIsRotatedIntoOdom)
{
  ASSERT_TRUE(configureAdapter()) << reason_;
  // 车头朝 +y（yaw = π/2），车体系前向速度 0.5 m/s -> odom 系速度应为 (0, 0.5)。
  publish(20.000, 1.0, 2.0, M_PI / 2.0, 0.5, 0.0, 0.0);

  srm27_minco_core::State2D state;
  ASSERT_TRUE(adapter_.stateAt(20.000, steadyNow(), state, &reason_)) << reason_;
  EXPECT_NEAR(state.velocity.x(), 0.0, 1.0e-9) << "twist 是车体系分量，必须旋转到 odom 系";
  EXPECT_NEAR(state.velocity.y(), 0.5, 1.0e-9);
  EXPECT_NEAR(state.x, 1.0, 1.0e-9);
  EXPECT_NEAR(state.y, 2.0, 1.0e-9);
}

TEST_F(StateAdapterTest, EmptyTwistFallsBackToPoseDifferenceInOdom)
{
  ASSERT_TRUE(configureAdapter()) << reason_;
  publish(30.000, 0.0, 0.0, 0.0);
  publish(30.100, 0.0, -0.03, 0.0);

  srm27_minco_core::State2D state;
  ASSERT_TRUE(adapter_.stateAt(30.100, steadyNow(), state, &reason_)) << reason_;
  // odom 系位姿差分就是车体原点在 odom 系的速度，不需要车体系往返旋转。
  EXPECT_NEAR(state.velocity.x(), 0.0, 1.0e-9);
  EXPECT_NEAR(state.velocity.y(), -0.3, 1.0e-6);
}

TEST_F(StateAdapterTest, StaleSampleIsRejected)
{
  ASSERT_TRUE(configureAdapter()) << reason_;
  publish(40.000, 0.0, 0.0, 0.0);

  srm27_minco_core::State2D state;
  std::string reason;
  EXPECT_FALSE(adapter_.stateAt(41.000, steadyNow(), state, &reason))
    << "超过 state_timeout 的状态必须被判为失效";
  EXPECT_FALSE(reason.empty());
}

TEST_F(StateAdapterTest, PositionJumpIncrementsResetCount)
{
  ASSERT_TRUE(configureAdapter()) << reason_;
  ASSERT_EQ(adapter_.resetCount(), 0u);
  publish(50.000, 0.0, 0.0, 0.0);
  // 0.1 s 内平移 5 m 视为定位重置，而不是真实运动。
  publish(50.100, 5.0, 0.0, 0.0);
  EXPECT_GE(adapter_.resetCount(), 1u) << "位姿跳变必须计入重置计数，控制器据此作废旧轨迹与热启动";
}

TEST_F(StateAdapterTest, OutOfOrderSampleIsDroppedAndDoesNotProduceVelocitySpike)
{
  ASSERT_TRUE(configureAdapter()) << reason_;
  publish(60.000, 0.0, 0.0, 0.0);
  publish(60.100, 0.05, 0.0, 0.0);
  // 乱序（时间倒退）的采样：必须被丢弃，既不能拿它做差分（分母为负会得到方向相反的
  // 速度），也不能让它覆盖更新的位姿。
  publishRaw(60.050, 0.90, 0.0, 0.0);
  // 只空转很短时间：超过 state_timeout(0.1 s) 之后按设计就应当判为状态失效。
  spinFor(std::chrono::milliseconds(50));

  EXPECT_EQ(adapter_.droppedSampleCount(), 1u);
  EXPECT_NEAR(adapter_.lastSampleStamp(), 60.100, 1.0e-9) << "latest 采样不应被更旧的时间戳覆盖";

  srm27_minco_core::State2D state;
  ASSERT_TRUE(adapter_.stateAt(60.100, steadyNow(), state, &reason_)) << reason_;
  EXPECT_NEAR(state.x, 0.05, 1.0e-9) << "位姿必须来自 60.100 的采样";
  EXPECT_NEAR(state.velocity.x(), 0.5, 1.0e-6)
    << "丢弃乱序采样后仍应得到正确速度；实测 " << state.velocity.x();
}

TEST_F(StateAdapterTest, UnusableDifferenceIntervalIsRejected)
{
  ASSERT_TRUE(configureAdapter()) << reason_;
  publish(70.000, 0.0, 0.0, 0.0);
  // 间隔超过 max_sample_gap：没有 twist 时不能给出差分速度，必须明确失效。
  publish(70.900, 0.5, 0.0, 0.0);

  srm27_minco_core::State2D state;
  std::string reason;
  EXPECT_FALSE(adapter_.stateAt(70.900, steadyNow(), state, &reason));
  EXPECT_FALSE(reason.empty());
}

TEST_F(StateAdapterTest, MissingPlanningFrameTransformIsNotSilentlyIgnored)
{
  // planning_frame 与里程计 frame_id 不同且没有 TF：必须是明确失效，
  // 不能用单位变换伪装成有效定位（方案 §2.3 修正项 4）。
  ASSERT_TRUE(configureAdapter("map")) << reason_;
  publish(80.000, 0.0, 0.0, 0.0);

  srm27_minco_core::State2D state;
  std::string reason;
  EXPECT_FALSE(adapter_.stateAt(80.000, steadyNow(), state, &reason));
  EXPECT_NE(reason.find("TF"), std::string::npos) << "失败原因应指出 TF 问题: " << reason;
}
