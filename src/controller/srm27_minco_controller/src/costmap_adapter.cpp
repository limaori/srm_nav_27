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

#include "srm27_minco_controller/costmap_adapter.hpp"

#include <algorithm>
#include <mutex>
#include <nav2_costmap_2d/cost_values.hpp>
#include <vector>

namespace srm27_minco_controller
{

namespace
{

/// \brief FNV-1a 64 位哈希，用于判断地图内容是否真的变化。
std::uint64_t hashBytes(const unsigned char * _data, std::size_t _size)
{
  std::uint64_t hash = 1469598103934665603ULL;
  for (std::size_t i = 0; i < _size; ++i) {
    hash ^= static_cast<std::uint64_t>(_data[i]);
    hash *= 1099511628211ULL;
  }
  return hash;
}

}  // namespace

bool CostmapAdapter::configure(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> _costmap_ros, const Config & _config,
  std::string * _reason)
{
  const auto fail = [&_reason](const char * _message) {
    if (_reason != nullptr) {
      *_reason = _message;
    }
    return false;
  };

  configured_ = false;
  if (!_costmap_ros) {
    return fail("costmap adapter requires a Costmap2DROS instance");
  }
  costmap_ros_ = std::move(_costmap_ros);
  config_ = _config;
  configured_ = true;
  version_ = 0;
  stamp_ = 0.0;
  content_hash_ = 0;
  has_hash_ = false;
  return true;
}

bool CostmapAdapter::snapshot(
  double _stamp, srm27_minco_core::GridSnapshot & _snapshot, std::uint64_t & _version,
  std::string * _reason)
{
  const auto fail = [&_reason](const char * _message) {
    if (_reason != nullptr) {
      *_reason = _message;
    }
    return false;
  };

  if (!configured_) {
    return fail("costmap adapter not configured");
  }
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  if (costmap == nullptr) {
    return fail("costmap is not available");
  }

  const unsigned int size_x = costmap->getSizeInCellsX();
  const unsigned int size_y = costmap->getSizeInCellsY();
  if (size_x == 0 || size_y == 0) {
    return fail("costmap has zero size");
  }
  const std::size_t count = static_cast<std::size_t>(size_x) * static_cast<std::size_t>(size_y);

  std::vector<unsigned char> raw(count, 0);
  const double origin_x = costmap->getOriginX();
  const double origin_y = costmap->getOriginY();
  const double resolution = costmap->getResolution();

  {
    // 互斥锁只覆盖复制过程，随后立即释放；后续 EDT / 优化都在副本上进行。
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
    const unsigned char * char_map = costmap->getCharMap();
    if (char_map == nullptr) {
      return fail("costmap character map is null");
    }
    std::copy(char_map, char_map + count, raw.begin());
  }

  // 代价值 1..252 是膨胀层产生的梯度代价，**不写入原始占用**。
  std::vector<srm27_minco_core::CellState> cells(count, srm27_minco_core::CellState::kFree);
  for (std::size_t i = 0; i < count; ++i) {
    const unsigned char cost = raw[i];
    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
      cells[i] = srm27_minco_core::CellState::kOccupied;
    } else if (cost == nav2_costmap_2d::NO_INFORMATION) {
      cells[i] = srm27_minco_core::CellState::kUnknown;
    } else if (
      config_.treat_inscribed_as_obstacle && cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      cells[i] = srm27_minco_core::CellState::kOccupied;
    }
  }

  // 版本号只在**内容**变化时递增：滚动地图每帧都在动，若按帧递增会让
  // “地图版本绝对相等”的提交检查永远失败（方案 §4.4）。
  const std::uint64_t hash = hashBytes(raw.data(), raw.size());
  if (!has_hash_ || hash != content_hash_) {
    content_hash_ = hash;
    has_hash_ = true;
    ++version_;
  }
  stamp_ = _stamp;
  _version = version_;

  if (!_snapshot.reset(
        origin_x, origin_y, resolution, static_cast<int>(size_x), static_cast<int>(size_y),
        std::move(cells), version_, _stamp)) {
    return fail("failed to build the grid snapshot");
  }
  return true;
}

}  // namespace srm27_minco_controller
