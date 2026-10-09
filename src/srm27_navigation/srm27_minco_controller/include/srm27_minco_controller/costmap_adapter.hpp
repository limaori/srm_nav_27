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

#ifndef SRM27_MINCO_CONTROLLER__COSTMAP_ADAPTER_HPP_
#define SRM27_MINCO_CONTROLLER__COSTMAP_ADAPTER_HPP_

#include <cstdint>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <string>

#include "srm27_minco_core/grid_snapshot.hpp"

namespace srm27_minco_controller
{

/// \brief 把 Nav2 costmap 复制成与 ROS 解耦的二维占用快照。
///
/// 语义（方案 §6.1、§13）：
///  * 只有 `LETHAL_OBSTACLE` (254) 视为**原始障碍**；`NO_INFORMATION` (255) 视为未知，
///    是否不可通行由 `unknown_is_obstacle` 决定（距离场始终把未知当种子，保守）。
///  * 膨胀层产生的 `INSCRIBED_INFLATED_OBSTACLE` (253) 及以下的代价值**不写入原始占用**：
///    机器人半径已经在碰撞校验里单独计入，把它当障碍会让半径被重复计算、通道被人为变窄。
///    如需按“中心不可进入”语义使用，可打开 `treat_inscribed_as_obstacle`，但必须清楚
///    这会重复计入半径。
///  * 互斥锁只覆盖复制 origin/resolution/size/cost 数组，复制完立即释放（方案 §4.4）；
///    EDT、插值、搜索与优化都在副本上运行。
class CostmapAdapter
{
public:
  /// \brief 配置。
  struct Config
  {
    /// \brief 未知区域是否不可通行。
    bool unknown_is_obstacle{true};
    /// \brief 是否把内切膨胀代价 (253) 也当成原始障碍（默认关闭，避免半径重复计入）。
    bool treat_inscribed_as_obstacle{false};
  };

  CostmapAdapter() = default;

  /// \brief 设置数据源与配置。
  bool configure(
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> _costmap_ros, const Config & _config,
    std::string * _reason = nullptr);

  /// \brief 复制一份快照。
  /// \param _stamp 快照对应的 ROS 时间（秒），由调用方传入（适配层不依赖节点时钟）。
  /// \param _snapshot 输出快照。
  /// \param _version 输出地图内容版本；只有内容真正变化时才递增。
  /// \param _reason 失败原因（可为 nullptr）。
  /// \return 成功返回 true。
  bool snapshot(
    double _stamp, srm27_minco_core::GridSnapshot & _snapshot, std::uint64_t & _version,
    std::string * _reason = nullptr);

  /// \brief 当前版本号（最近一次成功快照的版本）。
  std::uint64_t version() const { return version_; }

  /// \brief 最近一次成功快照的时间（ROS 秒）。
  double stamp() const { return stamp_; }

  /// \brief 数据源是否可用。
  bool ready() const { return static_cast<bool>(costmap_ros_); }

private:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_{};
  Config config_{};
  std::uint64_t version_{0};
  double stamp_{0.0};
  std::uint64_t content_hash_{0};
  bool has_hash_{false};
  bool configured_{false};
};

}  // namespace srm27_minco_controller

#endif  // SRM27_MINCO_CONTROLLER__COSTMAP_ADAPTER_HPP_
