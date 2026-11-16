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

#include "srm27_minco_core/grid_snapshot.hpp"

#include <cmath>

namespace srm27_minco_core
{

bool GridSnapshot::reset(
  double _origin_x, double _origin_y, double _resolution, int _size_x, int _size_y,
  std::vector<CellState> _cells, std::uint64_t _map_version, double _stamp)
{
  // 失败时对象保持为空：不能只把 valid_ 置假而留下旧的尺寸与单元，否则
  // cellCount()/sizeX()/containsCell() 会继续暴露上一次的快照（与头文件契约不一致）。
  valid_ = false;
  size_x_ = 0;
  size_y_ = 0;
  cells_.clear();
  if (
    !std::isfinite(_origin_x) || !std::isfinite(_origin_y) || !std::isfinite(_resolution) ||
    !std::isfinite(_stamp)) {
    return false;
  }
  if (_resolution <= 0.0 || _size_x <= 0 || _size_y <= 0) {
    return false;
  }
  const std::size_t expected =
    static_cast<std::size_t>(_size_x) * static_cast<std::size_t>(_size_y);
  if (_cells.size() != expected) {
    return false;
  }

  origin_x_ = _origin_x;
  origin_y_ = _origin_y;
  resolution_ = _resolution;
  size_x_ = _size_x;
  size_y_ = _size_y;
  map_version_ = _map_version;
  stamp_ = _stamp;
  cells_ = std::move(_cells);
  valid_ = true;
  return true;
}

std::vector<std::uint8_t> GridSnapshot::buildSeedMask(bool _unknown_is_seed) const
{
  std::vector<std::uint8_t> mask;
  if (!valid_) {
    return mask;
  }
  mask.resize(cells_.size(), static_cast<std::uint8_t>(1));
  for (std::size_t i = 0; i < cells_.size(); ++i) {
    const bool seed =
      cells_[i] == CellState::kOccupied || (_unknown_is_seed && cells_[i] == CellState::kUnknown);
    mask[i] = seed ? static_cast<std::uint8_t>(0) : static_cast<std::uint8_t>(1);
  }
  return mask;
}

bool GridSnapshot::hasAnySeed(bool _unknown_is_seed) const
{
  if (!valid_) {
    return false;
  }
  for (const CellState state : cells_) {
    if (state == CellState::kOccupied || (_unknown_is_seed && state == CellState::kUnknown)) {
      return true;
    }
  }
  return false;
}

}  // namespace srm27_minco_core
