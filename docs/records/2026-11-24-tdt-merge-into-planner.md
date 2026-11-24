# 2026-11-24 `tdt_nav_kit` 拆包并入 planner 前/后端

> 状态：已完成（离线构建 + 单测 + launch 参数面验证；实车与 Gazebo 闭环未在本轮复测）
>
> **后续变动（2026-11-24，两次）**：
> 1. 地图/栅格部分已再次重构——`srm27_nav_common` 的 `costmap_converter`、
>    `planner_geometry` 与 `srm27_nav_smoother` 的 `sfc_grid` 抽到 `src/planner/map`
>    （`srm27_map_core`、`srm27_map_adapter`），见
>    [2026-11-24-planner-map-module.md](2026-11-24-planner-map-module.md)。
> 2. 上游算法源码不再放在 `srm27_nav_common/vendor`：YAstar 进
>    `srm27_nav_planner/vendor`，MinimumSnap/SfcSquare 进 `srm27_nav_smoother/vendor`，见
>    [2026-11-24-vendor-sources-into-plugins.md](2026-11-24-vendor-sources-into-plugins.md)。
>
> 本文第 3、4 节的路径以当时为准。
> 范围：`src/tdt_nav_kit` → `src/planner/{common,frontend,backend}`，
> 以及 `srm27_nav_bringup` 的规划插件开关、参数注入、脚本与文档
> 未改动：算法实现（YAstar / MinimumSnap / SfcSquare 的数值行为）、控制器、行为树节点、
> 速度链、定位链路

## 1. 背景与目标

原 `src/tdt_nav_kit` 把「全局规划器（前端）」和「路径平滑器（后端）」放在同一个 ROS 包：

- 一个 `libtdt_nav_kit.so` 同时注册 `GlobalPlanner` 与 `Smoother`；
- vendored MIT 源码和本工作区 Apache-2.0 代码混在同一 `include/`、`src/`；
- 规划器与平滑器里各有一份 `Path ↔ 点列 / 朝向` 转换（`toPath` / `writeBack`）；
- 包内自带两份完整 Nav2 参数（约 1445 行）、演示 launch 和一份与 bringup 重复的行为树；
- `srm27_nav_bringup` 的 `package.xml` 故意不声明它，`--packages-up-to` / rosdep 带不上。

本轮按仓库「前端 / 后端 / 共用底座」的分层拆成三个包，并把装配资产收进
`srm27_nav_bringup`，消除重复配置与重复代码。

## 2. 目标结构

```text
src/planner/
├── common/
│   └── srm27_nav_common/          # 共用底座：vendored 算法 + Costmap/Path 转换（不注册插件）
├── frontend/
│   └── srm27_nav_planner/         # Nav2 GlobalPlanner 插件（YAstar）
└── backend/
    └── srm27_nav_smoother/        # Nav2 Smoother 插件（MinimumSnap + SfcSquare）
```

依赖方向：

```text
srm27_nav_planner ──► srm27_nav_common ◄── srm27_nav_smoother
```

前端与后端之间**没有**直接依赖；vendored 算法只存在于 `srm27_nav_common`，
两个插件包只保留 Apache-2.0 的包装层。

## 3. 文件归属

| 原路径（`src/tdt_nav_kit/`） | 新路径 |
|---|---|
| `vendor/**`、`vendor/LICENCE` | `common/srm27_nav_common/vendor/**` |
| `THIRD_PARTY_NOTICES.md`、`PATCHES.md` | `common/srm27_nav_common/` |
| `src/costmap_converter.cpp`、`path_utils.*`、`planner_geometry.*` | `common/srm27_nav_common/src|include/srm27_nav_common/` |
| `src/tdt_global_planner.cpp`、`include/.../tdt_global_planner.hpp` | `frontend/srm27_nav_planner/src/yastar_global_planner.cpp`、`include/srm27_nav_planner/yastar_global_planner.hpp` |
| `src/tdt_smoother.cpp`、`include/.../tdt_smoother.hpp` | `backend/srm27_nav_smoother/src/minimum_snap_smoother.cpp`、`include/srm27_nav_smoother/minimum_snap_smoother.hpp` |
| `src/sfc_visualization.*` | `backend/srm27_nav_smoother/` |
| `costmap_converter.*` 中的 `SfcGrid` / `makeSfcGrid` / `convertCostmapCellsToSfc` | 新增 `backend/srm27_nav_smoother/{include,src}/sfc_grid.*` |
| —（新拆出） | `common/srm27_nav_common/{include,src}/path_conversion.*` |
| `test/test_costmap_converter`、`test_path_utils`、`test_planner_geometry`、`test_yastar_chain`、`test_sfc_origin`、`test_sfc_bound_frame` | `common/srm27_nav_common/test/` |
| `test/test_pluginlib_load` | 拆成 `srm27_nav_planner` 与 `srm27_nav_smoother` 各一份 |
| `test/test_sfc_grid`、`test_sfc_markers` | `backend/srm27_nav_smoother/test/` |
| `tools/bench_yastar.cpp`、`bench_planner.py` | `common/srm27_nav_common/tools/` |
| `config/nav2_params_srm_tdt*.yaml` | **删除**：两段插件参数改为 `srm27_nav_bringup` 的 launch 注入 |
| `launch/tdt_nav_simulation.launch.py` | **删除**：能力变成仿真入口的两个开关（§5） |
| `behavior_trees/navigate_to_pose_w_smoothing.xml` | **删除**：bringup 的 `nav2_bt/` 是唯一版本（更安全，无 BackUp） |

## 4. 命名与插件 ID

| 项 | 旧 | 新 |
|---|---|---|
| 前端包 | `tdt_nav_kit`（与后端同包） | `srm27_nav_planner` |
| 后端包 | `tdt_nav_kit`（与前端同包） | `srm27_nav_smoother` |
| 共用底座 | — | `srm27_nav_common` |
| 全局规划器插件 | `tdt_nav_kit::TdtGlobalPlanner` | `srm27_nav_planner::YAstarGlobalPlanner` |
| 平滑器插件 | `tdt_nav_kit::TdtSmoother` | `srm27_nav_smoother::MinimumSnapSmoother` |
| 平滑器实例名 | `tdt_smoother` | `minimum_snap_smoother` |
| 实车 launch 开关 | `use_tdt_planner` | `use_yastar_planner` |
| 实车脚本开关 | `--tdt-planner` | `--yastar-planner`（`--tdt-planner` 保留为过渡别名） |
| 仿真入口开关 | `tdt_nav_simulation.launch.py` | `use_yastar_planner:=True` / `use_minisnap_smoother:=True`（或脚本 `--yastar-planner` / `--minisnap-smoother`） |
| Marker 命名空间 | `tdt_sfc` | `sfc_corridor` |

**兼容性**：插件 ID 是硬改名，不提供 pluginlib 别名；仓库内的参数文件、launch、
maps 实验配置已全部更新。仓库外若仍有 `plugin: tdt_nav_kit::TdtSmoother/…` 的
私有参数文件，需要同步改名。命令行 `--tdt-planner` 仍可用。

T-DT 的出处、MIT 许可与补丁记录保留在 `srm27_nav_common/THIRD_PARTY_NOTICES.md`、
`PATCHES.md` 和源码注释里；代码标识符不再带 `tdt`。

## 5. 装配层整合（`srm27_nav_bringup`）

### 实车（`nav_srm_real_launch.py`）

- `use_tdt_planner` → `use_yastar_planner`，注入插件改为
  `srm27_nav_planner::YAstarGlobalPlanner`；
- 平滑器实例/插件 ID 随参数文件更新为 `minimum_snap_smoother` /
  `srm27_nav_smoother::MinimumSnapSmoother`（实车默认仍是 MinimumSnap）；
- `package.xml` 增加 `srm27_nav_planner`、`srm27_nav_smoother` 的 `exec_depend`，
  `--packages-up-to srm27_nav_bringup` 现在会带上它们。

### 仿真（`nav_srm_simulation_launch.py`）

新增两个默认关闭的注入开关（都不打开时链路与改动前逐字一致）：

| 开关 | 注入内容 |
|---|---|
| `use_yastar_planner` | `planner_server.GridBased` 换 `srm27_nav_planner::YAstarGlobalPlanner`（含 `path_resample_step=0.20`、`goal_snap_max_radius=0.4`、`cost_inflate_radius=1.0` 等实车同源参数） |
| `use_minisnap_smoother` | `smoother_server` 换 `minimum_snap_smoother` / `srm27_nav_smoother::MinimumSnapSmoother`（路径加密 0.30 m、切角 0.40 m、`backend=path`、SFC 可视化开启）；同时把两份 `default_nav_*_bt_xml` 切到带 `SmoothPath` 的版本（否则平滑器根本不会被调用） |

不新增整份参数副本：`config/simulation/current/nav2_params_srm_fakeyaw.yaml` 仍是
唯一来源，注入在 `RewrittenYaml.perform()` 里只替换这两段。

### 脚本

- `start_real_nav.sh`：`--yastar-planner` / `--no-yastar-planner`；`--tdt-planner` 作为
  过渡别名；包检查列表增加 `srm27_nav_common`、`srm27_nav_smoother`
  （`--yastar-planner` 时再查 `srm27_nav_planner`）；
- `start_sim_nav.sh`：`--yastar-planner` / `--no-yastar-planner`、
  `--minisnap-smoother` / `--no-minisnap-smoother`；
- 为满足 `test_cli_parity` 的「仿真参数实车必须能解析」约定，实车脚本也接受
  `--minisnap-smoother` / `--no-minisnap-smoother`：只提示，不切换链路（实车默认已是
  MinimumSnap，且没有 SimpleSmoother 的正式实车参数文件）。

## 6. 代码层面的整合

| 改动 | 原因 |
|---|---|
| 新增 `path_conversion`（`pointsFromPath` / `toPath` / `writePath` / `parseOrientationMode`） | 原规划器 `toPath` 与平滑器 `writeBack` 是同一份逻辑的两份拷贝，`tangent` 朝向算法也重复；合并后两边共用一套 |
| `SfcGrid` / `makeSfcGrid` 从 common 移到后端包 | 只有平滑器使用 SFC 语义；common 只保留两者都用的 `CostmapSnapshot` |
| vendored 算法统一放 common，插件的 `CMakeLists` 只编译包装层 | MIT 与 Apache-2.0 的边界变成包边界；`package.xml` 的 license 也分得清 |
| common 导出 vendor 头目录（`include/srm27_nav_common/vendor/{YAstar,MinimumSnapOsqp}`） | 两个插件仍按上游习惯 `#include "yastar.hpp"` / `"minimumSnap.hpp"`，不需要改 vendored 源码 |
| 删除未声明的死参数 `corner_round_input_step` | 实车参数文件里写了 0.80，但插件从未 `declare_parameter`，属于静默无效配置；实现它属于改行为，本轮只删除 |
| `test_pluginlib_load` 拆成两份，各自断言 manifest 属于本包 | 拆分后不再共用一个共享库，旧断言 `SharesLibraryWithPlanner` 失去意义 |
| 新增 `test_path_conversion`（5 个用例） | 覆盖新抽出的公共转换层：朝向、header 保留、空路径 |

## 7. 同步修改的引用

- `src/bringup/srm27_nav_bringup/`：
  `config/real/current/nav2_params_srm_minco_fakeyaw.yaml`、
  `launch/nav_srm_real_launch.py`、`launch/nav_srm_simulation_launch.py`、
  `package.xml`、`test/test_fakeyaw_real_wiring.py`、
  `test/test_fakeyaw_launch_wiring.py`、`test/test_cli_parity.py`；
- `src/bringup/srm27_navigation/package.xml`（元包依赖）；
- `scripts/bringup/start_real_nav.sh`、`start_sim_nav.sh`；
- `maps/{219_1121,227_1113pro,227_1116_remap}/simulation/`：
  4 份实验参数、场景生成/验证脚本、manifest；顺带修掉了指向
  `/home/srm/nav_ws/srm_nav_27_tdt/...` 的失效行为树绝对路径；
- 文档：根 `README.md`、`docs/README.md`、`docs/architecture/launch_matrix.md`、
  `docs/architecture/package_inventory.md`；原包 README 归档为
  `docs/archive/2026-11-24-tdt-nav-kit-README.md`。

## 8. 验证

### 8.1 构建

```bash
colcon build --packages-select srm27_nav_common srm27_nav_planner srm27_nav_smoother \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  --parallel-workers 2
# 3 packages finished
colcon build --packages-select srm27_nav_bringup --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
```

构建期修正：`minimumSnap.hpp` 直接 `#include <OsqpEigen/OsqpEigen.h>`，而 OsqpEigen
没有 rosdep key、`ament_export_dependencies` 也不会把它的头目录带过去，因此
`srm27_nav_smoother` 的 CMake 显式 `find_package(OsqpEigen REQUIRED)` 并链接
`OsqpEigen::OsqpEigen`。原有 `ament_add_gtest` 的 rpath 循环警告与改动前相同，非新增。

### 8.2 单测

| 包 | 结果 |
|---|---|
| `srm27_nav_common` | **47 tests, 0 errors, 0 failures**（含新增 `test_path_conversion`） |
| `srm27_nav_planner` | **3 tests, 0 failures**（pluginlib 注册/实例化/描述文件归属） |
| `srm27_nav_smoother` | **16 tests, 0 failures**（SFC 栅格、Marker、pluginlib） |

来源：`colcon test --packages-select …` + `colcon test-result --test-result-base build/<pkg> --all`。

### 8.3 装配层回归

`src/bringup/srm27_nav_bringup/test/test_*.py` 8 个脚本全部 PASS（含新增的
`test_optional_yastar_and_minisnap_injection`）。

> 本机沙箱里 `~/.ros/log` 不可写，直接 `colcon test` 会以
> `OSError(30, 'Read-only file system')` 失败；把 `ROS_LOG_DIR` / `ROS_HOME` / `TMPDIR`
> 指到工作区内即可通过。这不是改动引入的失败。

`ros2 launch srm27_nav_bringup nav_srm_simulation_launch.py --show-args` 能看到
`use_yastar_planner` / `use_minisnap_smoother`；实车入口能看到 `use_yastar_planner`。

### 8.4 未覆盖

- 实车与 Gazebo 闭环导航未在本轮复测；算法数值行为未改，但插件包名/ID 变了，
  实车首跑请先 `ros2 param get /planner_server GridBased.plugin` 与
  `/smoother_server smoother_plugins` 核对；
- 原 TDT 演示使用的老地图 `rmuc_2025` + Omni 控制器组合没有继续维护；
  新入口用当前仿真链路（MINCO + fakeyaw）复现。

## 9. 回退

- 需要回到旧插件名时：`git revert` 本记录对应提交；
- 只回退运行链路：实车不加 `--yastar-planner`（ThetaStar），仿真默认即为
  ThetaStar + SimpleSmoother；MinimumSnap 平滑器与规划器解耦，可单独选。
