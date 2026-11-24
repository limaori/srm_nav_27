# srm_nav_27

深圳北理莫斯科大学北极熊战队哨兵机器人 ROS 2 Humble 工作空间。当前主线是
**MINCO/MPC + fakeyaw 云台系导航**：实车与仿真共用同一条速度链，只在最后一段执行端
（串口 / Gazebo）不同。

- 唯一入口表：[`docs/architecture/launch_matrix.md`](docs/architecture/launch_matrix.md)
- 包清点：[`docs/architecture/package_inventory.md`](docs/architecture/package_inventory.md)
- 文档索引：[`docs/README.md`](docs/README.md)
- 工作区清理记录：[`docs/records/2026-11-14-workspace-cleanup.md`](docs/records/2026-11-14-workspace-cleanup.md)

## 当前架构

```text
srm27_behavior（行为树 / 决策）
        │ NavigateToPose、rotation_cmd（底盘 wz）
        ▼
Nav2 + MINCO/MPC（导航参考系 gimbal_yaw_fake，只出平移）
        │ cmd_vel_nav_fake
fake_vel_transform（按云台/底盘夹角把平移转到车体系）
        │ cmd_vel_nav_body
srm_cmd_mux ◄── rotation_cmd（决策层底盘自转，只取 angular.z）
        │
        ├── 实车：cmd_vel_chassis → srm27_nav_protocol → 串口 → C 板
        └── 仿真：cmd_vel_sim → 速度适配器 → Gazebo 底盘插件

定位/感知：Livox MID-360 → Point-LIO 或 ZLIO → loam_interface / zlio_bridge
          → sensor_scan_generation（odom→gimbal_yaw）→ 代价地图 / Nav2

map→odom：静态 TF（--lio）| small_gicp（--reloc）| slam_toolbox（--slam）
          | srm27_zlocalization_backend（--zloc-backend，实验、默认关闭）
```

关键点：

- 导航的“车”是**云台**：`odom → gimbal_yaw` 只依赖固定外参；`base_link` 是底盘。
- 导航默认 `yaw_mode=xy_only`，底盘 `wz` 由决策层 `rotation_cmd` 在 mux 合成，不经过 MINCO。
- `fake_vel_transform` **不是历史残留**，当前实车/仿真链路都依赖它；
  旧步兵 `gimbal_yaw_fake` / `MecanumDrive2` 执行链才是已删除部分。
- 速度限幅的唯一来源是 `srm27_nav_bringup/config/{real,simulation}/current/speed_limits.yaml`。

## 包分层

| 模块 | 路径 | 说明 |
|---|---|---|
| 接口 | `src/interfaces` | `localization_interfaces`、`rm_decision_interfaces` |
| 感知定位 | `src/localization` | frontend: `frontend`(ZLIO)、`point_lio`、`livox_ros_driver2`、`loam_interface`、`pointcloud_to_laserscan`；backend: `small_gicp_relocalization`、`sensor_scan_generation`、`srm27_localization_adapter` |
| 轨迹规划 | `src/planner` | map: `srm27_map_core`（栅格/ESDF/占据几何/SFC，纯 C++）、`srm27_map_adapter`（Nav2 代价图→快照）；common: `srm27_nav_common`（YAstar/MinimumSnap vendored 源码 + Path 转换）；frontend: `terrain_analysis`、`terrain_analysis_ext`、`srm27_costmap_plugins`、`srm27_nav_behavior_plugins`、`srm27_nav_planner`（YAstar 全局规划器）；backend: `srm27_minco_core`、`srm27_minco_vendor`、`srm27_qpoases_vendor`、`srm27_nav_smoother`（MinimumSnap 平滑器） |
| 控制器 | `src/controller` | `srm27_minco_controller`、`srm27_omni_pid_controller`、`fake_vel_transform`、`srm27_chassis_control`、`srm27_nav_protocol`、`srm27_teleop_twist_joy` |
| 决策 | `src/decision` | common: `srm27_behavior_core`（航点/语义区/自转/姿态共享状态）；frontend: `srm27_behavior_plugins`（BehaviorTree 动作/条件/控制/装饰器插件）；backend: `srm27_behavior`（执行服务端、裁判输入、决策树/参数/launch） |
| 仿真 | `src/simulation` | `srm27_gazebo_simulator`、`srm27_robot_description`、`ign_sim_pointcloud_tool` |
| 装配 | `src/bringup` | `srm27_nav_bringup`、`srm27_nav_tools`、`srm27_navigation` 元包 |
| 依赖 | `src/dependencies` | `BehaviorTree.ROS2` |

> 七个业务模块：感知定位前端/后端、轨迹规划前端/后端、控制器、决策、仿真；
> `interfaces`、`bringup`、`dependencies` 是支撑目录。ROS 包名没有改，只调整了目录归属。
> 每个包的输入/输出与路径见 [ROS 包清点](docs/architecture/package_inventory.md)。
>
> 2026-11-24：原 `src/tdt_nav_kit` 拆包并入轨迹规划——全局规划器进
> `srm27_nav_planner`（前端），MinimumSnap 平滑器进 `srm27_nav_smoother`（后端），
> vendored 算法与共用转换层进 `srm27_nav_common`（planner/common）。
> 插件 ID：`srm27_nav_planner::YAstarGlobalPlanner`、
> `srm27_nav_smoother::MinimumSnapSmoother`。开关：实车 `--yastar-planner`，
> 仿真 `--yastar-planner` / `--minisnap-smoother`。
>
> 2026-11-24：决策模块按同一框架拆成 `srm27_behavior_core`（common）、
> `srm27_behavior_plugins`（frontend）、`srm27_behavior`（backend）；服务端运行时
> 从 `srm27_behavior_plugins/bt_plugins` 加载插件，启动入口 `ros2 launch srm27_behavior ...`
> 与 `start_*_nav.sh --behavior` 均保持不变。
>
> 2026-11-24：地图部分单独成 `src/planner/map`——`srm27_map_core`（GridSnapshot/ESDF/
> 占据几何/SFC，纯 C++）与 `srm27_map_adapter`（Nav2 costmap→快照）。YAstar、
> MINCO/controller、MinimumSnap 平滑器共用同一套栅格语义与同一份 costmap 拷贝实现。
>
> 2026-11-24：上游算法源码不再集中——YAstar 进 `srm27_nav_planner/vendor/`，
> MinimumSnap/SfcSquare 进 `srm27_nav_smoother/vendor/`（各自 MIT 许可与补丁登记），
> `srm27_nav_common` 只保留共享的路径工具。

## 四个正式入口

| 用途 | 命令 | launch |
|---|---|---|
| 实车导航 | `./scripts/bringup/start_real_nav.sh` | `nav_srm_real_launch.py`（别名 `real_nav_srm_real_launch.py`） |
| 仿真导航 | `./scripts/bringup/start_sim_nav.sh` | `nav_srm_simulation_launch.py`（别名 `sim_nav_srm_simulation_launch.py`） |
| 实车建图 | `./scripts/bringup/start_real_slam.sh` | `real_mapping_launch.py`（别名 `real_mapping_entry_launch.py`） |
| 仿真建图 | `./scripts/bringup/start_sim_slam.sh` | `sim_slam_launch.py`（别名 `sim_slam_entry_launch.py`） |

旧路径 `./scripts/start_*.sh` 仍可用（`script → scripts` 兼容链接）。
完整默认值、参数文件、速度出口与 launch 状态见
[`docs/architecture/launch_matrix.md`](docs/architecture/launch_matrix.md)。

常用示例：

```bash
# 实车：默认 zlio + 静态 map->odom；GICP 重定位用 --reloc（需先验 PCD）
./scripts/bringup/start_real_nav.sh -m 227_1113
./scripts/bringup/start_real_nav.sh -m 227_1113 --reloc --prior-pcd 227_1113

# 实车：Omni 回退链
./scripts/bringup/start_real_nav.sh --controller omni

# 仿真：默认 Gazebo 真值里程计；接 LIO 用 --lio
./scripts/bringup/start_sim_nav.sh
./scripts/bringup/start_sim_nav.sh --lio --lio-frontend zlio

# 规划插件（可选）：YAstar 全局规划器 / MinimumSnap 平滑器
./scripts/bringup/start_real_nav.sh --yastar-planner
./scripts/bringup/start_sim_nav.sh --yastar-planner --minisnap-smoother

# 建图
./scripts/bringup/start_real_slam.sh
./scripts/bringup/start_real_slam.sh --save 227_1114
./scripts/bringup/start_sim_slam.sh
./scripts/bringup/start_sim_slam.sh --collect --name rmuc_2025
```

## 定位与控制模式

实车导航（五者互斥，区别是 `map→odom` 由谁发布）：

| 模式 | 命令 | 说明 |
|---|---|---|
| LIO + 静态 TF | `--lio`（未显式指定且前端为 `zlio` 时的默认） | ZLIO/Point-LIO 出里程计，`map→odom` 静态；不需要先验 PCD |
| GICP 重定位 | `--reloc`（默认前端 `point_lio` 时） | small_gicp 与 `maps/pcd/<site>_mapframe.pcd` 配准 |
| SLAM | `--slam` | `slam_toolbox` 发布 `map→odom`，边建图边导航 |
| 静态地图 | `--static` | 只加载栅格图 + 静态 TF；仅当外部提供 `odom→base_link` 时可用 |
| 定位后端 | `--zloc-backend --zloc-prior --zloc-map <目录>` | Z-Localization backend 用先验地图包发布 `map→odom`；**实验模式，默认关闭** |

仿真导航：

| 模式 | 命令 | 说明 |
|---|---|---|
| Gazebo 真值 | 默认 | 不接 LIO，验证控制/规划链路 |
| ZLIO | `--lio`（默认前端） | `frontend` + `zlio_bridge` |
| Point-LIO | `--lio --lio-frontend point_lio` | 回退前端 |
| GICP | `--gicp --prior-pcd <pcd>` | 先验点云重定位 |
| 定位后端 | `--lio --zloc-backend --zloc-prior --zloc-map <目录>` | backend 用先验地图包发布 `map→odom`（实验，默认关闭） |

backend 模式的 TF 所有权、输入契约与 reset 语义见
[`docs/architecture/localization_backend.md`](docs/architecture/localization_backend.md)。
它需要 GTSAM 4.2.x，本机没有 root 时用 `./scripts/setup/install_gtsam_4_2.sh` 装到用户目录。

控制器：

- **MINCO/MPC**：默认、当前唯一正式控制器（`srm27_minco_controller::MincoMpcController`）。
- **OmniPID**：实车 `--controller omni` 的正式回退；仿真入口已拒绝 Omni（参数文件是旧
  `base_link` 链，无法在云台系/fakeyaw 链上工作）。

## 构建

```bash
source /opt/ros/humble/setup.bash
# 首次：拉取 dependencies.repos 里的外部依赖（如果还没拉）
# vcs import src < dependencies.repos

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`srm27_zlocalization_backend`（Z-Localization backend 迁入版）额外依赖 GTSAM 4.2.x。
系统里没有时用仓库脚本装到用户目录，不需要 root：

```bash
./scripts/setup/install_gtsam_4_2.sh     # 默认 ~/.local/gtsam-4.2.0
# 家目录不可写时装到工作区（本机现有安装就在这，backend 的 build 缓存也指着它）：
GTSAM_PREFIX=$PWD/.deps/gtsam-4.2.0 ./scripts/setup/install_gtsam_4_2.sh
source scripts/setup/env_srm27.sh        # ROS + 本工作区 + GTSAM（自动探测前缀）
colcon build --packages-select srm27_zlocalization_backend
```

它默认**不参与**导航（`--zloc-backend` 之外的入口行为不变）；包缺失时默认链路照常启动。

只改导航 bringup 时可增量构建：

```bash
colcon build --packages-select srm27_nav_bringup --symlink-install
colcon test --packages-select srm27_nav_bringup
```

测试：

```bash
bash tests/smoke/test_lib_path_guard.sh
colcon test --packages-select srm27_nav_protocol
```

包内测试仍在各包 `test/` 下（MINCO core/controller、nav_plugins、fake_vel_transform、
chassis_control、nav_bringup、nav_protocol 等）。

## 地图与数据目录

地图（版本化）：

```text
maps/
├── <site>/
│   ├── <site>.yaml        # Nav2 栅格图
│   ├── <site>.pgm
│   ├── <site>.posegraph   # slam_toolbox 位姿图
│   └── <site>.data
└── pcd/
    ├── <site>.pcd           # Point-LIO 原始（camera_init 系）
    └── <site>_mapframe.pcd  # 转到地图系，GICP 用这份
```

运行数据（**不进 Git**，默认在工作区外）：

```text
~/srm_nav_27_data/           # SRM27_DATA_DIR 可覆盖
├── bags/                    # 建图录包 mapping_bag_*
├── diagnostics/             # 建图/运行日志
├── generated_pcd/point_lio/ # Point-LIO 累积点云 scans.pcd
├── experiments/             # 实验原始数据（按日期/实验名）
├── plots/                   # 最终图表
└── archive/                 # 已归档的一次性材料
```

- `SRM27_DATA_DIR`：运行数据根目录。
- `SRM27_PCD_DIR`：Point-LIO 点云输出目录，默认 `$SRM27_DATA_DIR/generated_pcd/point_lio`。
- 源码包 `point_lio/PCD/` 只保留说明文件；不要再往里面堆 PCD。
- 同一实验长期只保留：一份原始数据 + 一份最终图表 + 一份结论说明 + 元数据。

## 路径约定

- **脚本**：只使用 `scripts/<group>/<name>`；旧的 `script/` 和 `scripts/<name>` 快捷链接
  已删除。例如 `./scripts/bringup/start_real_nav.sh`。
- **launch**：根目录 `<name>_launch.py` 仍是 canonical，旧
  `ros2 launch srm27_nav_bringup <name>_launch.py` 不变；分组目录放唯一前缀名的别名
  （`real_nav_srm_real_launch.py`、`sim_slam_entry_launch.py` 等）。
- **config**：`config/{real,simulation}/<name>` 仍保留指向 `current|legacy/<name>` 的
  兼容符号链接；参数内容只有一份。

新代码、新文档请使用上述分组路径。
