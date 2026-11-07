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

#include "tdt_nav_kit/path_utils.hpp"

#include <algorithm>
#include <cmath>

namespace tdt_nav_kit
{

PointVec resamplePath(const PointVec & points, float step, int max_points)
{
  if (step <= 1e-3f || points.size() < 2 || max_points < 2) {
    return points;
  }

  PointVec dense;
  dense.reserve(points.size() * 4);

  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const Eigen::Vector2f a = points[i];
    const Eigen::Vector2f b = points[i + 1];
    const float dist = (b - a).norm();
    dense.push_back(a);

    if (dist > step) {
      const int segments = static_cast<int>(std::ceil(dist / step));
      for (int k = 1; k < segments; ++k) {
        const float t = static_cast<float>(k) / static_cast<float>(segments);
        dense.push_back(a + t * (b - a));
        if (static_cast<int>(dense.size()) >= max_points) {
          break;
        }
      }
    }
    if (static_cast<int>(dense.size()) >= max_points) {
      break;
    }
  }
  dense.push_back(points.back());

  if (static_cast<int>(dense.size()) > max_points) {
    dense.resize(static_cast<std::size_t>(max_points));
    dense.back() = points.back();
  }
  return dense;
}

PointVec roundCorners(
  const PointVec & points, float cut, float min_turn_deg, float sample_step)
{
  if (cut <= 1e-3f || points.size() < 3) {
    return points;
  }

  const float step = std::max(sample_step, 0.02f);
  constexpr float kPi = 3.14159265358979323846f;

  PointVec out;
  out.reserve(points.size() * 3);
  out.push_back(points.front());

  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    const Eigen::Vector2f a = points[i - 1];
    const Eigen::Vector2f b = points[i];
    const Eigen::Vector2f c = points[i + 1];

    const Eigen::Vector2f v1 = b - a;
    const Eigen::Vector2f v2 = c - b;
    const float l1 = v1.norm();
    const float l2 = v2.norm();

    // 重复点：跳过该拐点，避免除零
    if (l1 < 1e-6f || l2 < 1e-6f) {
      continue;
    }

    float cos_ang = v1.dot(v2) / (l1 * l2);
    cos_ang = std::clamp(cos_ang, -1.0f, 1.0f);
    const float turn_deg = std::acos(cos_ang) * 180.0f / kPi;

    if (turn_deg < min_turn_deg) {
      out.push_back(b);
      continue;
    }

    // 切角长度不能超过相邻线段的一半，否则相邻两个拐角会互相吃掉
    const float d = std::min({cut, 0.45f * l1, 0.45f * l2});
    const Eigen::Vector2f p0 = b - v1.normalized() * d;
    const Eigen::Vector2f p2 = b + v2.normalized() * d;

    // 二次贝塞尔 p0 -> b -> p2，按弧长近似决定采样数
    const int n = std::max(
      2, static_cast<int>(std::ceil((2.0f * d) / step)));
    for (int k = 0; k <= n; ++k) {
      const float t = static_cast<float>(k) / static_cast<float>(n);
      const float w0 = (1.0f - t) * (1.0f - t);
      const float w1 = 2.0f * (1.0f - t) * t;
      const float w2 = t * t;
      out.push_back(w0 * p0 + w1 * b + w2 * p2);
    }
  }

  out.push_back(points.back());
  return out;
}

}  // namespace tdt_nav_kit
