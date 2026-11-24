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

#ifndef SRM27_MAP_ADAPTER__COSTMAP_ADAPTER_HPP_
#define SRM27_MAP_ADAPTER__COSTMAP_ADAPTER_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include <nav2_costmap_2d/costmap_2d_ros.hpp>

#include "srm27_map_core/grid_snapshot.hpp"

namespace srm27_map_adapter
{

/// \brief Nav2 代价图 → `srm27_map_core::GridSnapshot` 的映射配置。
///
/// 语义：
///  * `NO_INFORMATION` (255) 一律记为 `kUnknown`，是否不可通行由下游按
///    `unknown_is_obstacle` 决定（距离场始终把未知当种子）。
///  * 代价值 **大于** `occupied_cost_threshold` 记为 `kOccupied`。
///    - `253`（MINCO/controller 默认）：只有 `LETHAL_OBSTACLE` (254) 算原始障碍，
///      内切膨胀 253 与 1..252 都是自由（机器人半径在碰撞校验里单独计入）。
///    - `252`（YAstar / SFC 默认）：253 与 254 都算障碍，用于「中心不可进入」口径。
struct CostmapConfig
{
  int occupied_cost_threshold{253};
};

/// \brief 从 `Costmap2D` 复制一份与 ROS 解耦的快照（供 CostmapSubscriber 场景使用）。
///
/// 持锁只覆盖 origin/resolution/size/cost 的复制；哈希（可选）也基于同一次复制，
/// 避免两次读之间地图被滚动窗口改写。
/// \param _content_hash 非空时输出代价数组的 FNV-1a 哈希（用于判断内容是否真的变化）。
/// \return 成功返回 true；失败时 snapshot 保持原样，reason 给出原因。
bool makeGridSnapshot(
  nav2_costmap_2d::Costmap2D * _costmap, const CostmapConfig & _config, double _stamp,
  srm27_map_core::GridSnapshot & _snapshot, std::uint64_t * _content_hash = nullptr,
  std::string * _reason = nullptr);

/// \brief 把 Nav2 costmap 复制成与 ROS 解耦的二维占用快照。
///
/// 除 `makeGridSnapshot` 的语义外还维护**内容版本号**：滚动地图每帧原点/内容都在动，
/// 若按帧递增版本，下游「地图版本绝对相等」的提交检查永远失败（方案 §4.4）。
/// 版本只在内容哈希变化时递增。
class CostmapAdapter
{
public:
  CostmapAdapter() = default;

  /// \brief 设置数据源与配置。
  bool configure(
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> _costmap_ros, const CostmapConfig & _config,
    std::string * _reason = nullptr);

  /// \brief 复制一份快照并更新内容版本。
  /// \param _stamp 快照对应的 ROS 时间（秒），由调用方传入（适配层不依赖节点时钟）。
  /// \param _snapshot 输出快照。
  /// \param _version 输出地图内容版本；只有内容真正变化时才递增。
  /// \param _reason 失败原因（可为 nullptr）。
  /// \return 成功返回 true。
  bool snapshot(
    double _stamp, srm27_map_core::GridSnapshot & _snapshot, std::uint64_t & _version,
    std::string * _reason = nullptr);

  /// \brief 当前版本号（最近一次成功快照的版本）。
  std::uint64_t version() const { return version_; }

  /// \brief 最近一次成功快照的时间（ROS 秒）。
  double stamp() const { return stamp_; }

  /// \brief 数据源是否可用。
  bool ready() const { return static_cast<bool>(costmap_ros_); }

private:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_{};
  CostmapConfig config_{};
  std::uint64_t version_{0};
  double stamp_{0.0};
  std::uint64_t content_hash_{0};
  bool has_hash_{false};
  bool configured_{false};
};

}  // namespace srm27_map_adapter

#endif  // SRM27_MAP_ADAPTER__COSTMAP_ADAPTER_HPP_
