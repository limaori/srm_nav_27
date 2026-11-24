# srm27_map_core

规划地图底座：**纯 C++**（只依赖 Eigen），不依赖 ROS / Nav2 / TF，可离线单测。
把原先散在 `srm27_minco_core`、`srm27_nav_common`、`srm27_nav_smoother` 的
地图/栅格代码收拢成一套规范类型。

| 头文件 | 内容 |
|---|---|
| `srm27_map_core/grid_snapshot.hpp` | 规范栅格 `GridSnapshot`：`CellState`（自由/占据/未知）、分辨率/原点/尺寸、版本与时间戳、世界↔栅格换算、种子掩码、二值/字节编码 |
| `srm27_map_core/esdf_2d.hpp` | `Esdf2D`：由快照建欧氏距离场，双线性插值距离/梯度、保守半格修正 |
| `srm27_map_core/occupancy_geometry.hpp` | 点/连续线段/路径可通行判断、最近自由点（未知是否算障碍由调用方显式传入） |
| `srm27_map_core/sfc_grid.hpp` | SFC 语义栅格视图（0 = 障碍 / 255 = 可通行）与四周补边 |

## 语义边界

- 快照只记录**原始占用**：`kOccupied` 对应 Nav2 `LETHAL_OBSTACLE`，膨胀代价值
  （1..253）不写入；未知区域单独记为 `kUnknown`，是否不可通行由使用方决定。
- `buildBinaryMask()` 输出 YAstar 6 参 `setMap()` 需要的 `0 = 可行 / 1 = 阻挡`；
  `buildByteMask()` 供 SFC 等需要自定义编码的算法使用。两者共用同一未知策略。
- 地图外与非法点一律不可通行；`nearestFreePoint` 只返回地图内格中心。

Nav2 的具体代价值→`CellState` 映射不在这里，而在地图适配层
`srm27_map_adapter`（依赖 `nav2_costmap_2d`），保持本包可离线构建。

## 构建与测试

```bash
colcon build --packages-select srm27_map_core --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON --parallel-workers 2
colcon test --packages-select srm27_map_core --event-handlers console_direct+
```

测试覆盖：快照 reset 失败契约/坐标换算/越界语义/两种编码、ESDF 距离与梯度
（含未知种子）、连续线段穿角与最近自由点、SFC 编码与补边。
