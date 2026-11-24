// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.

#include "srm27_map_core/sfc_grid.hpp"

#include <cstddef>
#include <stdexcept>

namespace srm27_map_core
{

SfcGrid makeSfcGrid(const GridSnapshot & snapshot, bool unknown_is_obstacle)
{
  if (!snapshot.valid()) {
    throw std::invalid_argument("makeSfcGrid: invalid snapshot");
  }

  SfcGrid grid;
  grid.size_x = snapshot.sizeX() + 2;   // 四周各补 1 格障碍边界
  grid.size_y = snapshot.sizeY() + 2;
  grid.resolution = static_cast<float>(snapshot.resolution());
  grid.origin_x = static_cast<float>(snapshot.originX() - snapshot.resolution());
  grid.origin_y = static_cast<float>(snapshot.originY() - snapshot.resolution());
  grid.data.assign(
    static_cast<std::size_t>(grid.size_x) * static_cast<std::size_t>(grid.size_y), 0);

  // 先按 0 = 障碍 / 255 = 可通行 生成内部区域，再拷进补边后的栅格。
  const auto inner = snapshot.buildByteMask(255, 0, unknown_is_obstacle);
  const auto w = static_cast<std::size_t>(grid.size_x);
  for (int y = 0; y < snapshot.sizeY(); ++y) {
    for (int x = 0; x < snapshot.sizeX(); ++x) {
      const auto src = static_cast<std::size_t>(y) * static_cast<std::size_t>(snapshot.sizeX()) +
        static_cast<std::size_t>(x);
      const auto dst = static_cast<std::size_t>(y + 1) * w + static_cast<std::size_t>(x + 1);
      grid.data[dst] = inner[src];
    }
  }
  return grid;
}

}  // namespace srm27_map_core
