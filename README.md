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
```

关键点：

- 导航的“车”是**云台**：`odom → gimbal_yaw` 只依赖固定外参；`base_link` 是底盘。
- 导航默认 `yaw_mode=xy_only`，底盘 `wz` 由决策层 `rotation_cmd` 在 mux 合成，不经过 MINCO。
- `fake_vel_transform` **不是历史残留**，当前实车/仿真链路都依赖它；
  旧步兵 `gimbal_yaw_fake` / `MecanumDrive2` 执行链才是已删除部分。
- 速度限幅的唯一来源是 `srm27_nav_bringup/config/{real,simulation}/current/speed_limits.yaml`。

## 包分层

| 层 | 路径 | 说明 |
|---|---|---|
| 决策 | `src/srm27_behavior` | 行为树、BT 插件、任务参数 |
| 导航 | `src/srm27_navigation` | `srm27_nav_bringup`（launch/参数/地图/RViz）、`srm27_minco_core`、`srm27_minco_controller`、`srm27_nav_plugins`、`point_lio`、`fake_vel_transform`、`small_gicp_relocalization`、`sensor_scan_generation` 等 |
| 定位前端 | `src/frontend`、`src/localization_interfaces` | ZLIO 前端与重置/定位接口 |
| 底盘 | `src/srm27_chassis_control` | 速度合成、限幅、看门狗（`srm_cmd_mux`） |
| 下位机通信 | `src/srm27_nav_protocol` | 串口协议、速度反馈、底盘状态 |
| 模型 | `src/srm27_robot_description` | URDF/xacro、雷达/云台外参（实车与仿真共用一份模型） |
| 仿真 | `src/srm27_gazebo_simulator` | Gazebo 世界、SRM 模型、传感器桥、速度适配 |
| 接口 | `src/interfaces` | 决策/自定义消息接口 |
| 工具/依赖 | `src/tools`、`src/dependencies` | 第三方包与工具；`dependencies.repos` 记录来源 |

> 本轮**没有**物理拆分 `src/srm27_navigation` 的 ROS 包；包路径与构建缓存保持原样。
> 目录边界与后续拆分建议见入口表和工作区清理记录。

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

# 建图
./scripts/bringup/start_real_slam.sh
./scripts/bringup/start_real_slam.sh --save 227_1114
./scripts/bringup/start_sim_slam.sh
./scripts/bringup/start_sim_slam.sh --collect --name rmuc_2025
```

## 定位与控制模式

实车导航（四者互斥，区别是 `map→odom` 由谁发布）：

| 模式 | 命令 | 说明 |
|---|---|---|
| LIO + 静态 TF | `--lio`（未显式指定且前端为 `zlio` 时的默认） | ZLIO/Point-LIO 出里程计，`map→odom` 静态；不需要先验 PCD |
| GICP 重定位 | `--reloc`（默认前端 `point_lio` 时） | small_gicp 与 `maps/pcd/<site>_mapframe.pcd` 配准 |
| SLAM | `--slam` | `slam_toolbox` 发布 `map→odom`，边建图边导航 |
| 静态地图 | `--static` | 只加载栅格图 + 静态 TF；仅当外部提供 `odom→base_link` 时可用 |

仿真导航：

| 模式 | 命令 | 说明 |
|---|---|---|
| Gazebo 真值 | 默认 | 不接 LIO，验证控制/规划链路 |
| ZLIO | `--lio`（默认前端） | `frontend` + `zlio_bridge` |
| Point-LIO | `--lio --lio-frontend point_lio` | 回退前端 |
| GICP | `--gicp --prior-pcd <pcd>` | 先验点云重定位 |

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
