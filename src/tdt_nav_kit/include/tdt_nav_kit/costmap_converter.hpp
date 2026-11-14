// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.
//
// 上游 vendor/ 内的文件适用 MIT，见 vendor/LICENCE 与 THIRD_PARTY_NOTICES.md。

#ifndef TDT_NAV_KIT__COSTMAP_CONVERTER_HPP_
#define TDT_NAV_KIT__COSTMAP_CONVERTER_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

// 前向声明：让本头文件不拉入 ROS 依赖，纯算法单测可以直接包含。
namespace nav2_costmap_2d
{
class Costmap2D;
class Costmap2DROS;
}  // namespace nav2_costmap_2d

namespace tdt_nav_kit
{

struct ConverterConfig
{
  /// 代价 >= 该值视为障碍。默认 252 = nav2_costmap_2d::MAX_NON_OBSTACLE。
  /// Nav2 inflation 层产生 1..252，交给上游 YAstar 的势场处理，不在此判障碍。
  int occupancy_threshold{252};
  /// NO_INFORMATION(255) 的处理方式，默认按障碍。
  bool treat_unknown_as_obstacle{true};
};

struct CostmapSnapshot
{
  int size_x{0};
  int size_y{0};
  float resolution{0.0f};
  float origin_x{0.0f};
  float origin_y{0.0f};
  /// 行主序 y * size_x + x；0 = 可行，1 = 障碍。
  /// 该语义与 YAstar 的 6 参 setMap 重载一致（不要用 3 参 u_char* 重载，那个是 0 = 障碍）。
  std::vector<int8_t> data;
};

/// 纯转换逻辑，不碰 ROS，可无节点单测。
/// char_map 为 Costmap2D::getCharMap() 的原始缓冲，count = size_x * size_y。
std::vector<int8_t> convertCostmapCells(
  const unsigned char * char_map, std::size_t count, const ConverterConfig & cfg);

/// SFC（SfcSquare / MinimumSnap）专用栅格。
/// **占据语义与 CostmapSnapshot 相反**：0 = 障碍，255 = 可通行。
/// 上游 SfcSquare 全文以 `map(y, x) == 0` 判障碍（sfcSquare.cpp:143,163-167），
/// 且 MinimumSnap 的走廊生成只认这一套，故必须单独转换，不能复用 data。
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

/// 同 convertCostmapCells，但输出 SFC 语义（0 / 255）。
std::vector<unsigned char> convertCostmapCellsToSfc(
  const unsigned char * char_map, std::size_t count, const ConverterConfig & cfg);

/// 由快照生成 SFC 栅格，并**在四周补一圈障碍边界**（尺寸各 +2，原点各 -1 格）。
/// 原因：SfcSquare::_getBound() 会读 map(top - 1, i) 而不做边界判断
/// （sfcSquare.cpp:163-170 注释称"边界会在地图膨胀时处理"），
/// 若不补边界，靠近地图边缘的路径点会越界读内存。补一圈 0 可让扩张在边界处自然停下。
SfcGrid makeSfcGrid(const CostmapSnapshot & snap);

/// 直接由 Costmap2D 指针取快照（nav2_core::Smoother 拿到的是 CostmapSubscriber，
/// 不是 Costmap2DROS，故需要这个入口）。
CostmapSnapshot makeSnapshotFromCostmap(
  nav2_costmap_2d::Costmap2D * costmap, const ConverterConfig & cfg);

/// 在 costmap 互斥锁内一次性深拷贝出快照；之后规划只读快照，
/// 避免规划过程中读到正在被更新线程改写的 Costmap 内部缓冲。
/// 失败（空指针 / 零尺寸）抛 std::invalid_argument 或 std::runtime_error。
CostmapSnapshot makeSnapshot(
  nav2_costmap_2d::Costmap2DROS * costmap_ros, const ConverterConfig & cfg);

}  // namespace tdt_nav_kit

#endif  // TDT_NAV_KIT__COSTMAP_CONVERTER_HPP_
