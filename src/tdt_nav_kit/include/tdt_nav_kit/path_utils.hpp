// Copyright 2026 T-DT
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

#ifndef TDT_NAV_KIT__PATH_UTILS_HPP_
#define TDT_NAV_KIT__PATH_UTILS_HPP_

#include <vector>

#include <Eigen/Core>

namespace tdt_nav_kit
{

/// 平面点列（只保留 xy，与 MinimumSnap 的输入一致）。
using PointVec = std::vector<Eigen::Vector2f>;

/**
 * @brief 按最大段长线性加密点列。
 *
 * 上游 MinimumSnap 的碰撞检查按输入点连线/样条采样判定：输入越稀，样条在拐角
 * 处越容易切进膨胀层。YAstar 的段长可达 5.7 m，实测在 `check_for_collisions`
 * 打开时必定失败并原样返回，因此求解前先降到 0.3 m 量级。
 *
 * @param points     输入点列
 * @param step       目标最大段长（米）；<= 0 表示不处理
 * @param max_points 输出点数上限，超出后截断并把末点固定为原点列末点
 * @return 加密后的点列；输入不足 2 点或 step <= 0 时原样返回
 */
PointVec resamplePath(const PointVec & points, float step, int max_points);

/**
 * @brief 按弧长均匀抽稀点列（把段长放大到 step 量级，保留首末点）。
 *
 * roundCorners 的切角长度被 0.45*相邻段长 夹住：规划器输出段长约 0.2 m 时，
 * cut=0.4 实际只能用 0.09 m，90° 拐角仍是 R≈6 cm 的圆弧。先抽稀到 0.8 m
 * 量级，切角才能真正起作用（实测 κmax 10 → 3.1，净空不变）。
 *
 * @param points     输入点列
 * @param step       目标段长（米）；<= 0 表示不处理
 * @param max_points 输出点数上限
 * @return 抽稀后的点列；输入不足 3 点、step <= 0 或总长不足一个 step 时原样返回
 */
PointVec decimatePath(const PointVec & points, float step, int max_points);

/**
 * @brief 折角圆弧化：用二次贝塞尔在拐点处切角。
 *
 * MinimumSnap 是对输入点做插值的，折线本身的尖角会被原样保留（实测 545° 总转角
 * 的路径上出现 24 rad/m 的瞬时曲率，对应转弯半径 4 cm，底盘无法跟随）。
 * 在拐点两侧各退 `cut` 米，用二次贝塞尔（控制点=原拐点）连接，曲率随切角长度
 * 增大而下降，且曲线始终落在两条相邻线段构成的三角形内——只要原折线无碰撞，
 * 切角一般也不会引入碰撞（最终仍由平滑器的碰撞检查兜底）。
 *
 * @param points       输入点列
 * @param cut          切角长度（米）；<= 0 表示不处理
 * @param min_turn_deg 只有转角大于该值（度）才切角，避免把直行路径切出波动
 * @param sample_step  每段贝塞尔的采样步长（米），用于控制输出点密度
 * @return 切角后的点列
 */
PointVec roundCorners(
  const PointVec & points, float cut, float min_turn_deg, float sample_step);

}  // namespace tdt_nav_kit

#endif  // TDT_NAV_KIT__PATH_UTILS_HPP_
