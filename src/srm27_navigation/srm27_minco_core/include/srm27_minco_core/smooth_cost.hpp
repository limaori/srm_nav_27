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

#ifndef SRM27_MINCO_CORE__SMOOTH_COST_HPP_
#define SRM27_MINCO_CORE__SMOOTH_COST_HPP_

#include <cmath>

namespace srm27_minco_core
{

/// \file
/// \brief 优化目标里使用的平滑代价函数。
///
/// **值与导数成对定义**：任何一处使用都把函数值和解析导数一起取，避免出现
/// “代价平滑、梯度不一致”的实现（方案 §6.4）。

/// \brief 数值稳定的 logistic 函数 `1 / (1 + exp(-z))`。
inline double stableSigmoid(const double _z)
{
  if (_z >= 0.0) {
    return 1.0 / (1.0 + std::exp(-_z));
  }
  const double e = std::exp(_z);
  return e / (1.0 + e);
}

/// \brief softplus：`log(1 + exp(z))`，大 |z| 时避免溢出。
inline double softplus(const double _z)
{
  const double abs_z = std::abs(_z);
  const double positive = _z > 0.0 ? _z : 0.0;
  return positive + std::log1p(std::exp(-abs_z));
}

/// \brief softplus 的逆：`log(expm1(y))`，要求 `y > 0`。
/// \return `y <= 0` 或非有限时返回 -inf 哨兵值（调用方必须避免）。
inline double softplusInverse(const double _y)
{
  if (!(_y > 0.0) || !std::isfinite(_y)) {
    return -40.0;
  }
  if (_y > 30.0) {
    // log(expm1(y)) ~= y - exp(-y)，避免 expm1 溢出。
    return _y - std::exp(-_y);
  }
  return std::log(std::expm1(_y));
}

/// \brief 平滑 hinge（softplus 形式）的值与导数。
///
/// `h <= 0`（未越界）时值趋近 0、导数趋近 0；`h > 0` 时值趋近 `h`、导数趋近 1。
/// \param _h 越界量，单位与代价一致（例如“还差多少米才安全”）。
/// \param _beta 平滑程度，越大越接近硬 hinge；必须为正且有限。
/// \param _value 输出的函数值。
/// \param _derivative 输出的导数 d(value)/dh。
/// \return 参数非法时返回 false，输出为 0。
inline bool softplusHinge(
  const double _h, const double _beta, double & _value, double & _derivative)
{
  _value = 0.0;
  _derivative = 0.0;
  if (!std::isfinite(_h) || !std::isfinite(_beta) || _beta <= 0.0) {
    return false;
  }
  const double z = _beta * _h;
  _value = softplus(z) / _beta;
  _derivative = stableSigmoid(z);
  return true;
}

/// \brief 段时长比例惩罚：`max(0, r-hi)^2 + max(0, lo-r)^2` 的值与导数。
/// \param _ratio `T_i / mean(T)`。
/// \param _low 允许的最小比例。
/// \param _high 允许的最大比例。
/// \param _value 输出的函数值。
/// \param _derivative 输出的 d(value)/d(ratio)。
/// \return 参数非法时返回 false。
inline bool timeRatioPenalty(
  const double _ratio, const double _low, const double _high, double & _value, double & _derivative)
{
  _value = 0.0;
  _derivative = 0.0;
  if (!std::isfinite(_ratio) || !std::isfinite(_low) || !std::isfinite(_high) || _low > _high) {
    return false;
  }
  const double above = _ratio - _high;
  const double below = _low - _ratio;
  if (above > 0.0) {
    _value += above * above;
    _derivative += 2.0 * above;
  }
  if (below > 0.0) {
    _value += below * below;
    _derivative -= 2.0 * below;
  }
  return true;
}

}  // namespace srm27_minco_core

#endif  // SRM27_MINCO_CORE__SMOOTH_COST_HPP_
