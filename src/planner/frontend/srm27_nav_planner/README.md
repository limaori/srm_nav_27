# srm27_nav_planner

轨迹规划**前端**：Nav2 `GlobalPlanner` 插件，把 `srm27_nav_common` 里 vendored 的
T-DT YAstar（分层 Mask 的 A*）包装成全局规划器。

| 项 | 值 |
|---|---|
| 插件 ID | `srm27_nav_planner::YAstarGlobalPlanner` |
| 基类 | `nav2_core::GlobalPlanner`（Humble 两参 `createPlan`） |
| 输出 | `nav_msgs/msg/Path`，默认 `map` 帧；**不输出速度** |
| 地图输入 | `srm27_map_adapter::CostmapAdapter`（带内容版本号）→ `srm27_map_core::GridSnapshot` |
| 上游算法 | YAstar @ `tdt-nav-kit` `814d5ce0`（MIT，源码内置在本包 `vendor/src/YAstar/`） |

插件只读 Nav2 全局代价地图的一次快照（加锁深拷贝），不修改 costmap、行为树、
控制器或底盘链路。

## 启用

**实车**（`srm27_nav_bringup` 的 `nav_srm_real_launch.py`）：

```bash
./scripts/bringup/start_real_nav.sh --yastar-planner --lio --map <地图名> ...
# 等价于 launch 参数 use_yastar_planner:=True；
# 默认仍是 nav2_theta_star_planner/ThetaStarPlanner。
```

**仿真**（`nav_srm_simulation_launch.py`）：

```bash
ros2 launch srm27_nav_bringup nav_srm_simulation_launch.py use_yastar_planner:=True
```

检查：

```bash
ros2 param get /planner_server GridBased.plugin   # srm27_nav_planner::YAstarGlobalPlanner
```

## 参数（`planner_server.GridBased.<key>`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `occupancy_threshold` | `252` | 代价 > 该值视为障碍；253/254 仍是硬障碍 |
| `treat_unknown_as_obstacle` | `true` | `NO_INFORMATION(255)` 是否按障碍 |
| `mask_num_layers` | `4` | 上游 `main.cpp:96` 的 Mask 层数（静态图只用基础层） |
| `cost_inflate_radius` | `1.0` | 离障代价的搜索半径（米），配合上游 `0.5/(0.1+d)` 势场 |
| `simplify_epsilon` | `0.1` | 上游 `main.cpp:133` 的 DP 化简阈值（米） |
| `save_point_max_radius` | `0.4` | 起点不在自由格时的吸附半径（米） |
| `goal_snap_max_radius` | `0.4` | 目标吸附半径（米）；同一目标重规划期间吸附点固定 |
| `orientation_mode` | `identity` | `identity`（w=1）或 `tangent`（相邻点求 yaw） |
| `debug_timing` | `false` | 打印每次规划的分段耗时 |

2026-11-24 起，规划流程直接对齐上游 `main.cpp`：
`setMap -> setMaskNumLayers(4) -> setCostField(1.0, 0.5/(0.1+d)) -> initCostMap()`,
搜索后只做 `simplifyPath(0.1) + simplifyPathHypot()`，不再输出前加密、不再做
简化回退。实车默认注入值（`nav_srm_real_launch.py`）为
`occupancy_threshold=252`、`treat_unknown_as_obstacle=true`、
`save_point_max_radius=goal_snap_max_radius=0.4`、`cost_inflate_radius=1.0`、
`mask_num_layers=4`、`simplify_epsilon=0.1`、`orientation_mode=tangent`。

## 包内结构

```text
srm27_nav_planner/
├── vendor/src/YAstar/          # 上游 MIT 源码（yastar.* 编译，kinodynamicAstar.* 不编译）
├── vendor/LICENCE              # MIT 原文
├── include/srm27_nav_planner/  # 插件与规划器内部工具头（Apache-2.0）
├── src/                        # yastar_global_planner / yastar_path / goal_resolver
├── test/                       # pluginlib、规划器几何、YAstar 纯算法
├── tools/                      # bench_yastar.cpp、bench_planner.py
├── PATCHES.md                  # 上游最小补丁 P1/P2/P4/P5/P8/P9
└── THIRD_PARTY_NOTICES.md      # 来源、commit、sha256、许可边界
```

`vendor/` 只在本包内使用（CMake 用 `PRIVATE` include），安装到
`share/srm27_nav_planner/` 与 `include/srm27_nav_planner/vendor/` 仅为溯源，不导出给下游。

## 行为约定

- 每次规划从 `CostmapAdapter` 取一份加锁快照；`occupancy_threshold` 直接作为适配层阈值
  （`252` 时 253/254 都算障碍，与 SFC 平滑口径一致），未知格按
  `treat_unknown_as_obstacle` 编码进 `buildBinaryMask()`；
- 搜索路径先转到 Nav2 栅格中心，首末端点保留解析后的精确坐标；
- 简化后的路径若切过障碍格，退回未简化的搜索路径；
- 目标格被占据且吸附半径内无自由格时**规划失败**，不会静默换目标；
- 输出是稀疏路径，需要 `SmoothPath`（`srm27_nav_smoother`）加密后才适合
  `transformGlobalPlan` 按局部代价地图裁剪。

## 构建与测试

```bash
colcon build --packages-select srm27_nav_planner --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON --parallel-workers 2
colcon test --packages-select srm27_nav_planner --event-handlers console_direct+
```

`test_pluginlib_load` 验证插件注册与实例化；`test_planner_helpers` 验证 YAstar 角点→
格中心换算、目标吸附稳定性与「搜索+简化保留亚格端点」；`test_yastar_chain` 用独立参考
逐元素校验 `getSDF()` 并断言搜索可复现。地图几何本身在 `srm27_map_core`
（`test_occupancy_geometry`）。
