# 2026-11-24 规划地图模块 `src/planner/map`

> 状态：已完成（构建 + 单测 + bringup 回归；实车与 Gazebo 闭环未复测）
> 范围：从 `srm27_minco_core`、`srm27_nav_common`、`srm27_nav_smoother`、
> `srm27_minco_controller` 抽出地图/栅格相关代码，形成 `src/planner/map` 下的两个包
> 未改动：YAstar / MinimumSnap / MINCO 的算法数值行为、Nav2 参数、启动入口

## 1. 背景

地图相关代码此前散在 4 个包，有 3 套栅格语义、2 份重复的 costmap 拷贝：

| 位置 | 内容 | 槽点 |
|---|---|---|
| `srm27_minco_core` | `GridSnapshot`、`CellState`、`Esdf2D` | MINCO 的纯算法包被地图类型"污染"，YAstar 想用只能依赖 MINCO |
| `srm27_nav_common` | `CostmapSnapshot`（`int8_t 0/1`）、`planner_geometry`、`SfcGrid` | 另一套语义；与 `GridSnapshot` 类型重复 |
| `srm27_nav_smoother` | `SfcGrid`（`0=障碍/255=可通行`） | 又一套语义，只有 SFC 需要 |
| `srm27_minco_controller` | `CostmapAdapter`（加锁复制 + 版本号） | 与 nav_common 的 `makeSnapshot` 重复实现同一件事 |

## 2. 目标结构

```text
src/planner/map/
├── srm27_map_core/      # 纯 C++（只依赖 Eigen）：栅格/ESDF/占据几何/SFC 视图
└── srm27_map_adapter/   # 依赖 Nav2：Costmap2D(ROS) → GridSnapshot，带内容版本号
```

依赖方向：

```text
srm27_map_core  ◄── srm27_map_adapter ◄── srm27_nav_planner / srm27_nav_smoother
        ▲                                           │
        ├── srm27_minco_core ──► srm27_minco_controller
        └── srm27_nav_behavior_plugins（脱困行为）
```

`map_core` 不依赖 ROS 与 Nav2，可离线单测；只有 `map_adapter` 认识
`nav2_costmap_2d`。

## 3. 归属变化

| 原位置 | 新位置 |
|---|---|
| `srm27_minco_core/{include,src}/{grid_snapshot,esdf_2d}` | `map/srm27_map_core/`（`namespace srm27_map_core`） |
| `srm27_minco_core/test/test_esdf_2d.cpp` | `map/srm27_map_core/test/` |
| `srm27_nav_common/{include,src}/planner_geometry` 的占据几何 | `map/srm27_map_core/{occupancy_geometry}` |
| `srm27_nav_common/planner_geometry` 的 `StableGoal`、`nav2PathFromYastar` | `srm27_nav_planner/{goal_resolver,yastar_path}`（只有规划器用） |
| `srm27_nav_common/{include,src}/costmap_converter` | 删除；能力并入 `srm27_map_adapter` |
| `srm27_nav_smoother/{include,src}/sfc_grid` | `map/srm27_map_core/sfc_grid`（`GridSnapshot` 入参） |
| `srm27_minco_controller/{include,src}/costmap_adapter` | `map/srm27_map_adapter/costmap_adapter` |

## 4. 代码整合点

1. **统一规范类型**：所有规划/控制侧都用 `srm27_map_core::GridSnapshot`，
   不再有 `CostmapSnapshot` 与 `GridSnapshot` 两套结构。
2. **一套代价值映射**：`srm27_map_adapter::CostmapConfig::occupied_cost_threshold`。
   - `253`（MINCO/controller）：只有 `LETHAL_OBSTACLE`(254) 是原始障碍；
   - `252`（YAstar/SFC）：253/254 都算障碍。
   旧 `CostmapAdapter::Config` 的 `unknown_is_obstacle` / `treat_inscribed_as_obstacle`
   两个开关由「阈值 + 下游未知策略」表达，语义等价且少一个易混淆维度。
3. **一份 costmap 拷贝实现**：`makeGridSnapshot()` 带加锁复制 + 可选内容哈希，
   `CostmapAdapter` 在其上加版本号；controller 与两个插件共用。
4. **编码收敛到快照上**：`buildBinaryMask()`（YAstar 的 `0=可行/1=阻挡`）与
   `buildByteMask(free, blocked, ...)`（SFC 的 `255/0`），未知格策略显式传参，
   不再各自在插件里写转换循环。
5. **未知语义显式化**：`pointIsFree/segmentIsFree/pathIsFree/nearestFreePoint`
   都带 `unknown_is_obstacle` 参数（默认 true）。
6. **`sfc_grid` 入参改为 `GridSnapshot`**，因此 `makeSfcGrid()` 不再需要
   `nav_common` 的类型；SFC 补边逻辑不变。
7. **规划器专用逻辑留在前端包**：`StableGoal`（目标吸附跨重规划稳定）与
   `nav2PathFromYastar`（YAstar 角点→Nav2 格中心）只在 YAstar 插件用，不进地图模块。

## 5. 与 MINCO 的兼容处理

`srm27_minco_core` 的公开头文件（`minco_optimizer.hpp`、`trajectory_validator.hpp`、
`recovery_decider.hpp`）大量使用 `Esdf2D` / `EsdfQueryResult` / `CellState` /
`GridSnapshot`。为把改动面压到最小：

- 三个头文件改 `#include "srm27_map_core/esdf_2d.hpp"`，并在
  `namespace srm27_minco_core` 内加 4 个 `using` 别名；
- `srm27_minco_controller`、`srm27_nav_behavior_plugins` 的调用点改为
  `srm27_map_core::...` 全名（显式表达"这是地图类型"）；
- `srm27_minco_core` 的 CMake 增加 `srm27_map_core` 的 include/link/export 依赖。

## 6. 验证

构建（受限并发，见 `编译注意事项.md`）：

```bash
./scripts/build/build_safe.sh --packages-select srm27_map_core srm27_map_adapter
./scripts/build/build_safe.sh --packages-select srm27_nav_common srm27_minco_core
./scripts/build/build_safe.sh --packages-select srm27_minco_controller
./scripts/build/build_safe.sh --packages-select srm27_nav_planner srm27_nav_smoother \
  srm27_nav_behavior_plugins
```

| 包 | 结果 |
|---|---|
| `srm27_map_core` | **36 tests, 0 failures**（快照坐标/编码、ESDF、占据几何、SFC） |
| `srm27_map_adapter` | **5 tests, 0 failures**（253/252 两档阈值、未知格、内容哈希） |
| `srm27_nav_planner` | **11 tests, 0 failures**（pluginlib + 目标吸附/YAstar 几何） |
| `srm27_nav_smoother` | **15 tests, 0 failures** |
| `srm27_nav_common` | **45 tests, 0 failures**（去掉地图相关用例后） |
| `srm27_nav_behavior_plugins` | **36 tests, 0 failures** |
| `srm27_minco_core` | 12 个 gtest 全通过；`clang_format` 失败为既有问题（包内无 `.clang-format`） |
| `srm27_minco_controller` | 8/9 通过；`test_command_frame` 需可写 `ROS_LOG_DIR`（本机 `~/.ros` 只读），`clang_format` 既有问题 |
| `srm27_nav_bringup` | 9 个 Python 回归脚本全部 PASS |

未覆盖：实车与 Gazebo 闭环；算法数值行为未变，但 MINCO/controller 的类型来源变了，
实车首跑建议核对 `ros2 param get /controller_server FollowPath.plugin` 与
`/planner_server GridBased.plugin`。

## 7. 构建事故记录（重要）

本轮一次 `colcon build --packages-up-to srm27_nav_bringup srm27_navigation` 在
`frontend`(ZLIO) + `srm27_zlocalization_backend`(GTSAM) 并行编译约 25 分钟后
**整机重启**。根因：`--parallel-workers 2` 只限制包级并发，每包内部仍是
`make -j16`，合计 30+ 个 g++ 进程打满 15 GiB 内存 + 1 GiB swap。

处置：新增 `scripts/build/build_safe.sh`（`MAKEFLAGS=-j2` +
`--parallel-workers 1`，可 `JOBS=` / `PKG_WORKERS=` 覆盖），并重写
`编译注意事项.md` 说明两层并发的区别。**验证小改动不要跑全栈构建。**
