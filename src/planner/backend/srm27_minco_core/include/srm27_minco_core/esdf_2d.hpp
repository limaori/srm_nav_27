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

#ifndef SRM27_MINCO_CORE__ESDF_2D_HPP_
#define SRM27_MINCO_CORE__ESDF_2D_HPP_

#include <Eigen/Core>
#include <cstdint>
#include <vector>

#include "srm27_minco_core/grid_snapshot.hpp"

namespace srm27_minco_core
{

/// \brief 二维距离场查询结果。
struct EsdfQueryResult
{
  /// \brief 查询是否成功（点在栅格范围内、距离场已建立）。
  bool valid{false};
  /// \brief 有符号距离（m）：自由区为正，障碍/未知区为负。
  double distance{0.0};
  /// \brief 距离场梯度 d(distance)/d(p)，单位无量纲（m/m）。
  /// `gradient_valid` 为假时恒为零向量，调用方必须据此跳过梯度项。
  Eigen::Vector2d gradient{Eigen::Vector2d::Zero()};
  /// \brief 梯度是否可用。
  bool gradient_valid{false};
  /// \brief 该点所在的栅格单元语义。
  CellState cell_state{CellState::kUnknown};
  /// \brief 距离场所属地图版本。
  std::uint64_t map_version{0};

  /// \brief 该点是否位于原始障碍/未知单元内部。
  bool insideObstacle() const { return cell_state != CellState::kFree; }
};

/// \brief 由二维占用快照建立的欧氏距离场（ESDF 查询服务）。
///
/// 实现要点（方案 §6.1–§6.2）：
///  * 距离场种子只来自**原始占用**与未知区域，膨胀代价值不参与，避免把机器人半径
///    重复计入通道宽度。
///  * 对外提供米制、带符号、连续可插值的距离查询；内部同时保存“到障碍”和“到自由区”
///    两张精确 EDT，符号由占用语义给出。
///  * 到格中心的距离会被保守化：减去半格对角线，得到到障碍格边界的下界估计。
///  * 插值采用双线性；梯度是同一双线性表达式对位置的解析导数，二者一致。插值可能
///    过冲，因此硬碰撞判定不依赖插值距离放行（另见 TrajectoryValidator）。
class Esdf2D
{
public:
  /// \brief 构建参数。
  struct Config
  {
    /// \brief 未知区域是否作为距离场种子（保守默认 true）。
    bool unknown_is_seed{true};
    /// \brief 是否把距离保守化（减去半格对角线）。
    bool conservative{true};
    /// \brief 不存在任何障碍种子时返回的距离（m），避免出现无穷大破坏优化。
    double no_obstacle_distance{1.0e3};
  };

  Esdf2D() = default;

  /// \brief 由快照建立距离场。
  /// \return 快照非法时返回 false，内部状态被清空。
  bool build(const GridSnapshot & _snapshot, const Config & _config);

  /// \brief 用默认配置建立距离场。
  bool build(const GridSnapshot & _snapshot) { return build(_snapshot, Config()); }

  /// \brief 距离场是否可用。
  bool valid() const { return valid_; }

  /// \brief 是否至少存在一个障碍/未知种子。
  bool hasSeed() const { return has_seed_; }

  /// \brief 对应快照的地图版本。
  std::uint64_t mapVersion() const { return map_version_; }

  /// \brief 对应快照的时间戳（秒）。
  double stamp() const { return stamp_; }

  int sizeX() const { return size_x_; }
  int sizeY() const { return size_y_; }
  double resolution() const { return resolution_; }
  double halfCellDiagonal() const { return half_cell_diagonal_; }

  /// \brief 查询世界坐标处的距离与梯度。
  ///
  /// 地图外返回 `valid = false`；地图内未知区域按种子处理（负距离）。
  EsdfQueryResult query(double _x, double _y) const;

  /// \brief 只取有符号距离；失败时返回 +inf。
  double distanceAt(double _x, double _y) const;

  /// \brief 栅格单元的保守有符号距离（m）；越界返回 -inf。
  double cellDistance(int _mx, int _my) const;

  /// \brief 精确欧氏距离变换（Felzenszwalb–Huttenlocher 两遍一维抛物线变换）。
  ///
  /// \param _width 列数，必须为正。
  /// \param _height 行数，必须为正。
  /// \param _seed 行优先的种子掩码，true 表示种子，长度必须是 width*height。
  /// \param _dist_sq_out 输出**格单位**的距离平方，长度 width*height。
  /// \return 参数非法时返回 false。
  ///
  /// 该算法是公开的精确 EDT；本项目实现时以北京理工大学追梦战队
  /// `navi_minco_bit` 的 `ESDFUtils::computeEDT2D()` 作为对照实现，详见
  /// `srm27_minco_vendor/THIRD_PARTY_NOTICES.md`。
  static bool computeEdt2D(
    int _width, int _height, const std::vector<bool> & _seed, std::vector<double> & _dist_sq_out);

private:
  /// \brief 由格单位的距离平方与符号位计算米制有符号距离。
  double signedFromEdt(double _edt_sq, bool _occupied) const;

  bool valid_{false};
  bool has_seed_{false};
  Config config_{};
  int size_x_{0};
  int size_y_{0};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double resolution_{0.05};
  double half_cell_diagonal_{0.0};
  std::uint64_t map_version_{0};
  double stamp_{0.0};
  /// \brief 每个单元的有符号米制距离，行优先。
  std::vector<double> signed_distance_;
  /// \brief 每个单元的语义，行优先。
  std::vector<CellState> cells_;
};

}  // namespace srm27_minco_core

#endif  // SRM27_MINCO_CORE__ESDF_2D_HPP_
