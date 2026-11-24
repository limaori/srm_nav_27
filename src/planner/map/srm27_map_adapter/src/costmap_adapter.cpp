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

#include "srm27_map_adapter/costmap_adapter.hpp"

#include <mutex>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"

namespace srm27_map_adapter
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

/// \brief 代价值 → 单元语义。未知优先于阈值判断。
srm27_map_core::CellState cellStateFromCost(unsigned char _cost, int _threshold)
{
  if (_cost == nav2_costmap_2d::NO_INFORMATION) {
    return srm27_map_core::CellState::kUnknown;
  }
  if (static_cast<int>(_cost) > _threshold) {
    return srm27_map_core::CellState::kOccupied;
  }
  return srm27_map_core::CellState::kFree;
}

}  // namespace

bool makeGridSnapshot(
  nav2_costmap_2d::Costmap2D * _costmap, const CostmapConfig & _config, double _stamp,
  srm27_map_core::GridSnapshot & _snapshot, std::uint64_t * _content_hash,
  std::string * _reason)
{
  const auto fail = [&_reason](const char * _message) {
    if (_reason != nullptr) {
      *_reason = _message;
    }
    return false;
  };

  if (_costmap == nullptr) {
    return fail("costmap is null");
  }

  unsigned int size_x;
  unsigned int size_y;
  double origin_x;
  double origin_y;
  double resolution;
  std::vector<unsigned char> raw;
  {
    // 滚动窗口可能同时改原点和栅格，必须在同一把锁下读取。
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(_costmap->getMutex()));
    size_x = _costmap->getSizeInCellsX();
    size_y = _costmap->getSizeInCellsY();
    origin_x = _costmap->getOriginX();
    origin_y = _costmap->getOriginY();
    resolution = _costmap->getResolution();
    const unsigned char * char_map = _costmap->getCharMap();
    if (size_x == 0 || size_y == 0) {
      return fail("costmap has zero size");
    }
    if (char_map == nullptr) {
      return fail("costmap character map is null");
    }
    const std::size_t count = static_cast<std::size_t>(size_x) * size_y;
    raw.assign(char_map, char_map + count);
  }

  std::vector<srm27_map_core::CellState> cells(raw.size(), srm27_map_core::CellState::kFree);
  for (std::size_t i = 0; i < raw.size(); ++i) {
    cells[i] = cellStateFromCost(raw[i], _config.occupied_cost_threshold);
  }
  if (_content_hash != nullptr) {
    *_content_hash = hashBytes(raw.data(), raw.size());
  }
  if (!_snapshot.reset(
      origin_x, origin_y, resolution, static_cast<int>(size_x), static_cast<int>(size_y),
      std::move(cells), 0, _stamp))
  {
    return fail("failed to build the grid snapshot");
  }
  return true;
}

bool CostmapAdapter::configure(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> _costmap_ros, const CostmapConfig & _config,
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
  double _stamp, srm27_map_core::GridSnapshot & _snapshot, std::uint64_t & _version,
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

  std::uint64_t hash = 0;
  srm27_map_core::GridSnapshot snapshot;
  if (!makeGridSnapshot(costmap, config_, _stamp, snapshot, &hash, _reason)) {
    return false;
  }

  // 版本号只在**内容**变化时递增：滚动地图每帧都在动，若按帧递增会让
  // “地图版本绝对相等”的提交检查永远失败（方案 §4.4）。
  if (!has_hash_ || hash != content_hash_) {
    content_hash_ = hash;
    has_hash_ = true;
    ++version_;
  }
  stamp_ = _stamp;
  _version = version_;
  _snapshot = std::move(snapshot);
  return true;
}

}  // namespace srm27_map_adapter
