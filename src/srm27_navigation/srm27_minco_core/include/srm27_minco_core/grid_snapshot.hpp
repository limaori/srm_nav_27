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

#ifndef SRM27_MINCO_CORE__GRID_SNAPSHOT_HPP_
#define SRM27_MINCO_CORE__GRID_SNAPSHOT_HPP_

#include <Eigen/Core>
#include <cstdint>
#include <vector>

namespace srm27_minco_core
{

/// \brief 栅格单元语义。
///
/// 只描述**原始占用**，不包含膨胀代价：
///  * `kFree`：可通行。
///  * `kOccupied`：真实障碍占用（Nav2 `LETHAL_OBSTACLE`）。
///  * `kUnknown`：未知区域。是否不可通行由使用方按 `unknown_is_obstacle` 决定，
///    但距离场总是把未知当作障碍种子，避免把没看过的地方当作空旷（方案 §6.1）。
///
/// 膨胀层（`INSCRIBED_INFLATED_OBSTACLE` 及以下的代价值）**不得**写入本结构：
/// 机器人半径已经在碰撞校验里单独计入，重复计入会让有效通道变窄（方案 §13）。
enum class CellState : std::uint8_t {
  kFree = 0,
  kOccupied = 1,
  kUnknown = 2,
};

/// \brief 与 costmap 解耦的二维占用快照。
///
/// 由 ROS 适配层在持有 costmap 锁期间复制得到；此后所有距离场、搜索与优化都在
/// 本快照的副本上运行，不再持有 costmap 锁（方案 §4.4）。
class GridSnapshot
{
public:
  GridSnapshot() = default;

  /// \brief 建立快照。
  /// \param _origin_x 地图左下角 x（m）。
  /// \param _origin_y 地图左下角 y（m）。
  /// \param _resolution 每格边长（m），必须为正。
  /// \param _size_x 列数（x 方向格数），必须为正。
  /// \param _size_y 行数（y 方向格数），必须为正。
  /// \param _cells 行优先排列的单元，长度必须是 size_x * size_y。
  /// \param _map_version 地图版本号。
  /// \param _stamp 快照对应的 ROS 时间（秒）。
  /// \return 参数不合法时返回 false，对象保持为空。
  bool reset(
    double _origin_x, double _origin_y, double _resolution, int _size_x, int _size_y,
    std::vector<CellState> _cells, std::uint64_t _map_version, double _stamp);

  /// \brief 快照是否可用（尺寸、分辨率与数据长度自洽）。
  bool valid() const { return valid_; }

  int sizeX() const { return size_x_; }
  int sizeY() const { return size_y_; }
  double resolution() const { return resolution_; }
  double originX() const { return origin_x_; }
  double originY() const { return origin_y_; }
  std::uint64_t mapVersion() const { return map_version_; }
  double stamp() const { return stamp_; }

  /// \brief 单元总数。
  std::size_t cellCount() const { return cells_.size(); }

  /// \brief 是否在栅格范围内。
  bool containsCell(int _mx, int _my) const
  {
    return _mx >= 0 && _my >= 0 && _mx < size_x_ && _my < size_y_;
  }

  /// \brief 世界坐标 -> 栅格索引（向下取整）。不做范围检查。
  void worldToCell(double _x, double _y, int & _mx, int & _my) const
  {
    _mx = static_cast<int>(std::floor((_x - origin_x_) / resolution_));
    _my = static_cast<int>(std::floor((_y - origin_y_) / resolution_));
  }

  /// \brief 栅格索引 -> 单元中心的世界坐标。
  void cellToWorld(int _mx, int _my, double & _x, double & _y) const
  {
    _x = origin_x_ + (static_cast<double>(_mx) + 0.5) * resolution_;
    _y = origin_y_ + (static_cast<double>(_my) + 0.5) * resolution_;
  }

  /// \brief 单元状态；越界返回 `kUnknown`（保守）。
  CellState cell(int _mx, int _my) const
  {
    if (!valid_ || !containsCell(_mx, _my)) {
      return CellState::kUnknown;
    }
    return cells_
      [static_cast<std::size_t>(_my) * static_cast<std::size_t>(size_x_) +
       static_cast<std::size_t>(_mx)];
  }

  /// \brief 世界坐标处的单元状态；越界返回 `kUnknown`。
  CellState cellAt(double _x, double _y) const
  {
    if (!valid_) {
      return CellState::kUnknown;
    }
    int mx = 0;
    int my = 0;
    worldToCell(_x, _y, mx, my);
    return cell(mx, my);
  }

  /// \brief 该单元是否被原始障碍占用。
  bool isOccupied(int _mx, int _my) const { return cell(_mx, _my) == CellState::kOccupied; }

  /// \brief 障碍种子掩码：占用或未知都算种子。
  /// \param _unknown_is_seed 未知是否作为距离场种子，正常应为 true。
  std::vector<std::uint8_t> buildSeedMask(bool _unknown_is_seed) const;

  /// \brief 地平线内是否存在任何障碍种子（用于快速跳过无意义的规划）。
  bool hasAnySeed(bool _unknown_is_seed) const;

private:
  bool valid_{false};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double resolution_{0.05};
  int size_x_{0};
  int size_y_{0};
  std::uint64_t map_version_{0};
  double stamp_{0.0};
  std::vector<CellState> cells_;
};

}  // namespace srm27_minco_core

#endif  // SRM27_MINCO_CORE__GRID_SNAPSHOT_HPP_
