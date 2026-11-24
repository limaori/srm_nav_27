// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.

#ifndef SRM27_MAP_CORE__SFC_GRID_HPP_
#define SRM27_MAP_CORE__SFC_GRID_HPP_

#include <vector>

#include "srm27_map_core/grid_snapshot.hpp"

namespace srm27_map_core
{

/// SFC（SfcSquare / MinimumSnap）专用栅格。
/// **占据语义与 GridSnapshot 相反**：0 = 障碍，255 = 可通行。
/// 上游 SfcSquare 全文以 `map(y, x) == 0` 判障碍（sfcSquare.cpp:143,163-167），
/// 且 MinimumSnap 的走廊生成只认这一套，故必须单独转换，不能复用 `buildBinaryMask`。
struct SfcGrid
{
  int size_x{0};
  int size_y{0};
  float resolution{0.0f};
  float origin_x{0.0f};
  float origin_y{0.0f};
  /// 行主序 y * size_x + x；0 = 障碍，255 = 可通行。
  std::vector<unsigned char> data;
};

/// 由快照生成 SFC 栅格，并**在四周补一圈障碍边界**（尺寸各 +2，原点各 -1 格）。
/// 原因：SfcSquare::_getBound() 会读 map(top - 1, i) 而不做边界判断
/// （sfcSquare.cpp:163-170 注释称"边界会在地图膨胀时处理"），
/// 若不补边界，靠近地图边缘的路径点会越界读内存。补一圈 0 可让扩张在边界处自然停下。
/// `unknown_is_obstacle` 决定未知格是否编码成 0（障碍）。
SfcGrid makeSfcGrid(const GridSnapshot & snapshot, bool unknown_is_obstacle = true);

}  // namespace srm27_map_core

#endif  // SRM27_MAP_CORE__SFC_GRID_HPP_
