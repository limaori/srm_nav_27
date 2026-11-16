// Copyright 2027 SRM
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

#ifndef SRM27_BEHAVIOR__SEMANTIC_MAP_HPP_
#define SRM27_BEHAVIOR__SEMANTIC_MAP_HPP_

#include <yaml-cpp/yaml.h>

#include <string>
#include <utility>
#include <vector>

#include "srm27_behavior/waypoint_utils.hpp"

namespace srm27_behavior
{

struct SemanticZone
{
  std::string code;
  std::string type;
  double max_spin = 5.0;
  std::vector<std::pair<double, double>> polygon;

  bool contains(double x, double y) const
  {
    if (polygon.size() < 3) {
      return false;
    }
    bool inside = false;
    const size_t n = polygon.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
      const double xi = polygon[i].first;
      const double yi = polygon[i].second;
      const double xj = polygon[j].first;
      const double yj = polygon[j].second;
      if (((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
        inside = !inside;
      }
    }
    return inside;
  }
};

class SemanticMap
{
public:
  bool loadFromYaml(const std::string & raw_path)
  {
    zones_.clear();
    const std::string resolved = resolveBehaviorFilePath(raw_path);
    if (resolved.empty()) {
      return false;
    }
    try {
      const YAML::Node root = YAML::LoadFile(resolved);
      frame_id_ = root["frame_id"] ? root["frame_id"].as<std::string>("map") : "map";
      const auto zones_node = root["zones"];
      if (!zones_node || !zones_node.IsSequence()) {
        return false;
      }
      for (const auto & zn : zones_node) {
        SemanticZone zone;
        if (!zn["code"] || !zn["polygon"] || !zn["polygon"].IsSequence()) {
          continue;
        }
        zone.code = zn["code"].as<std::string>();
        zone.type = zn["type"] ? zn["type"].as<std::string>("") : "";
        zone.max_spin = zn["max_spin"] ? zn["max_spin"].as<double>(5.0) : 5.0;
        for (const auto & pt : zn["polygon"]) {
          if (pt.IsSequence() && pt.size() >= 2) {
            zone.polygon.emplace_back(pt[0].as<double>(), pt[1].as<double>());
          }
        }
        if (zone.polygon.size() >= 3) {
          zones_.push_back(std::move(zone));
        }
      }
      return true;
    } catch (const std::exception &) {
      zones_.clear();
      return false;
    }
  }

  const SemanticZone * findZoneAt(
    double x, double y, const std::string & zone_code = "",
    const std::string & zone_type = "") const
  {
    for (const auto & zone : zones_) {
      if (!zone_code.empty() && zone.code != zone_code) {
        continue;
      }
      if (!zone_type.empty() && zone.type != zone_type) {
        continue;
      }
      if (zone.contains(x, y)) {
        return &zone;
      }
    }
    return nullptr;
  }

  const std::string & frameId() const { return frame_id_; }
  size_t size() const { return zones_.size(); }

private:
  std::string frame_id_ = "map";
  std::vector<SemanticZone> zones_;
};

}  // namespace srm27_behavior

#endif  // SRM27_BEHAVIOR__SEMANTIC_MAP_HPP_
