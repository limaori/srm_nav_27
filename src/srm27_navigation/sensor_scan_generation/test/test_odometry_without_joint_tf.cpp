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

/// \brief 方案 §4.1.1 的回归：**关节角 TF 断流时 /odometry 仍按点云频率输出**。
///
/// 背景：导航的"车"从 base_link 改成 gimbal_yaw 之后，
///   * `lidar -> gimbal_yaw` 是固定外参（gimbal_yaw -> livox_frame -> livox_imu）；
///   * `lidar -> base_link` 要经过 gimbal_yaw_joint，也就是关节角。
/// 旧实现每帧查这两次 TF，任一失败就整帧跳过 —— 关节角一断流，/odometry 跟着停，
/// 导航直接停车（RViz 里底盘消失只是表面现象）。打开 `cache_robot_base_transform`
/// 之后，前者开机查一次并缓存，后者失败只丢 odom->base_link 的 TF。
///
/// 2026-11-08 追加：TF 树反转（URDF gimbal_is_root）之后本节点只发 odom->gimbal_yaw，
/// 关节角那一支（gimbal_yaw->base_link）交给 robot_state_publisher —— 所以第三个用例
/// 专门验证"即使 base_link 的关节 TF 存在，也**不再**发 odom->base_link"。
///
/// 测试环境里**故意只发布 gimbal_yaw -> livox_imu 的静态 TF**，不发布 base_link：
///   * cache_robot_base_transform=true  -> /odometry 照常输出，且没有 odom->base_link；
///   * cache_robot_base_transform=false -> 行为不变（缺 base_link 就整帧跳过，/odometry 停）。

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_scan_generation/sensor_scan_generation.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace
{
using namespace std::chrono_literals;

/// \brief 测试用 QoS：里程计/点云是被测节点的输入，用 BEST_EFFORT 覆盖它的订阅端。
rclcpp::QoS inputQos()
{
  return rclcpp::QoS(10).best_effort();
}

class OdometryWithoutJointTfTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    // 沙箱/CI 里 ~/.ros 可能是只读的，rclcpp::init() 会因为建日志文件失败直接抛异常。
    // 只在调用方没有显式指定时把日志目录指到临时目录。
    if (std::getenv("ROS_LOG_DIR") == nullptr) {
      char template_path[] = "/tmp/sensor_scan_generation_test_XXXXXX";
      char * log_dir = mkdtemp(template_path);
      if (log_dir != nullptr) {
        setenv("ROS_LOG_DIR", log_dir, 0);
      }
    }
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  /// \brief 起被测节点 + 驱动节点，并发布 `gimbal_yaw -> livox_imu` 的静态 TF。
  /// \param _publish_robot_base_tf TF 树反转模式（发 odom->gimbal_yaw，不碰 base_link）
  void start(
    bool _cache_robot_base_transform, bool _publish_robot_base_tf = false,
    bool _with_joint_tf = false)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {
        rclcpp::Parameter("lidar_frame", std::string("livox_imu")),
        rclcpp::Parameter("base_frame", std::string("base_link")),
        rclcpp::Parameter("robot_base_frame", std::string("gimbal_yaw")),
        rclcpp::Parameter("publish_tf", true),
        rclcpp::Parameter("cache_robot_base_transform", _cache_robot_base_transform),
        rclcpp::Parameter("publish_robot_base_tf", _publish_robot_base_tf),
      });
    generation_ =
      std::make_shared<sensor_scan_generation::SensorScanGenerationNode>(options);
    driver_ = std::make_shared<rclcpp::Node>("sensor_scan_generation_test_driver");

    odom_pub_ = driver_->create_publisher<nav_msgs::msg::Odometry>("lidar_odometry", inputQos());
    scan_pub_ =
      driver_->create_publisher<sensor_msgs::msg::PointCloud2>("registered_scan", inputQos());
    odometry_sub_ = driver_->create_subscription<nav_msgs::msg::Odometry>(
      "odometry", rclcpp::QoS(10).reliable(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr _msg) {
        ++odometry_count_;
        last_odometry_child_frame_ = _msg->child_frame_id;
      });
    tf_sub_ = driver_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::QoS(100).reliable(),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr _msg) {
        for (const auto & transform : _msg->transforms) {
          // 只看**本节点**承诺发布的那两条（父系必须是 odom）：测试里自己发的
          // gimbal_yaw -> base_link（模拟 robot_state_publisher 的关节角）要排除掉。
          if (transform.header.frame_id != "odom") {
            continue;
          }
          if (transform.child_frame_id == "base_link") {
            ++odom_base_link_tf_count_;
          }
          if (transform.child_frame_id == "gimbal_yaw") {
            ++odom_gimbal_tf_count_;
          }
        }
      });

    static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(driver_);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(generation_);
    executor_->add_node(driver_);
    spin_thread_ = std::thread([this]() {executor_->spin();});

    // 固定外参：gimbal_yaw <- livox_imu（原点重合，只为让 TF 树连通）。
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = driver_->now();
    transform.header.frame_id = "gimbal_yaw";
    transform.child_frame_id = "livox_imu";
    transform.transform.rotation.w = 1.0;
    static_broadcaster_->sendTransform(transform);

    if (_with_joint_tf) {
      // 关节角那一支（gimbal_yaw -> base_link）由 robot_state_publisher 发；
      // 这里用一个动态广播器模拟它存在，用来验证"树反转后不再发 odom->base_link"。
      joint_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(driver_);
      geometry_msgs::msg::TransformStamped joint;
      joint.header.stamp = driver_->now();
      joint.header.frame_id = "gimbal_yaw";
      joint.child_frame_id = "base_link";
      joint.transform.translation.z = -0.26805;
      joint.transform.rotation.w = 1.0;
      joint_broadcaster_->sendTransform(joint);
    }

    // 等静态 TF 进到被测节点的 tf_buffer（/tf_static 是 transient_local，订阅侧一建连就有，
    // 这里只是让 spin 线程跑起来）。
    std::this_thread::sleep_for(300ms);
  }

  void stop()
  {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    static_broadcaster_.reset();
    joint_broadcaster_.reset();
    odometry_sub_.reset();
    tf_sub_.reset();
    odom_pub_.reset();
    scan_pub_.reset();
    if (executor_) {
      executor_->remove_node(generation_);
      executor_->remove_node(driver_);
    }
    executor_.reset();
    driver_.reset();
    generation_.reset();
  }

  /// \brief 按给定周期发布 `_frames` 帧"里程计 + 点云"（时间戳相同 -> 会被同步器配对）。
  void publishFrames(int _frames, std::chrono::milliseconds _period)
  {
    for (int i = 0; i < _frames; ++i) {
      const rclcpp::Time stamp = driver_->now();
      nav_msgs::msg::Odometry odometry;
      odometry.header.stamp = stamp;
      odometry.header.frame_id = "odom";
      odometry.child_frame_id = "gimbal_yaw";
      odometry.pose.pose.position.x = 0.01 * static_cast<double>(i);
      odometry.pose.pose.orientation.w = 1.0;
      odom_pub_->publish(odometry);

      sensor_msgs::msg::PointCloud2 scan;
      scan.header.stamp = stamp;
      scan.header.frame_id = "odom";
      scan.height = 1;
      scan.width = 0;
      scan.is_dense = true;
      scan_pub_->publish(scan);

      std::this_thread::sleep_for(_period);
    }
  }

  template<typename Predicate>
  static void waitFor(std::chrono::milliseconds _duration, Predicate _done)
  {
    const auto deadline = std::chrono::steady_clock::now() + _duration;
    while (std::chrono::steady_clock::now() < deadline) {
      if (_done()) {
        return;
      }
      std::this_thread::sleep_for(10ms);
    }
  }

  std::shared_ptr<sensor_scan_generation::SensorScanGenerationNode> generation_;
  std::shared_ptr<rclcpp::Node> driver_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> joint_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  std::atomic<int> odometry_count_{0};
  std::atomic<int> odom_base_link_tf_count_{0};
  std::atomic<int> odom_gimbal_tf_count_{0};
  std::string last_odometry_child_frame_;
};

/// \brief 缓存固定外参时：关节角 TF 缺失不影响 /odometry。
TEST_F(OdometryWithoutJointTfTest, OdometryKeepsFlowingWithoutJointTf)
{
  start(true);
  constexpr int kFrames = 20;
  publishFrames(kFrames, 50ms);
  // 同步器可能吞掉第一帧，给回调留点余量。
  waitFor(500ms, [this]() {return odometry_count_.load() >= kFrames - 2;});
  const int received = odometry_count_.load();
  stop();

  EXPECT_GE(received, kFrames / 2)
    << "关节角 TF 断流时 /odometry 也停了（只收到 " << received << "/" << kFrames
    << " 帧）：cache_robot_base_transform 的降级路径没有生效";
  EXPECT_EQ(odom_base_link_tf_count_.load(), 0)
    << "没有 base_link 的 TF 却发布了 odom->base_link";
}

/// \brief 默认（不缓存）：保持既有行为 —— 缺 base_link 就整帧跳过。
TEST_F(OdometryWithoutJointTfTest, LegacyBehaviorStillSkipsWithoutJointTf)
{
  start(false);
  publishFrames(3, 50ms);
  waitFor(1200ms, []() {return false;});
  const int received = odometry_count_.load();
  stop();

  EXPECT_EQ(received, 0)
    << "cache_robot_base_transform=false 时行为必须与改造前一致（整帧跳过），实际输出了 "
    << received << " 条 /odometry";
}

/// \brief 树反转：发 odom->gimbal_yaw，且绝不发 odom->base_link（否则会有两个父节点）。
TEST_F(OdometryWithoutJointTfTest, RobotBaseTfModePublishesGimbalAndNeverBaseLink)
{
  start(true, /*_publish_robot_base_tf=*/true, /*_with_joint_tf=*/true);
  constexpr int kFrames = 20;
  publishFrames(kFrames, 50ms);
  waitFor(500ms, [this]() {return odometry_count_.load() >= kFrames - 2;});
  const int received = odometry_count_.load();
  const int gimbal_tf = odom_gimbal_tf_count_.load();
  const int base_tf = odom_base_link_tf_count_.load();
  stop();

  EXPECT_GE(received, kFrames / 2) << "/odometry 必须照常输出";
  EXPECT_GT(gimbal_tf, 0) << "树反转后必须由本节点发 odom->gimbal_yaw";
  EXPECT_EQ(base_tf, 0)
    << "树反转后不能再发 odom->base_link：base_link 已经是 gimbal_yaw 的子节点，"
       "两个父节点会让整棵 TF 树失效";
}

}  // namespace
