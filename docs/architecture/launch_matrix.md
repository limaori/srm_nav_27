# SRM27 启动入口矩阵（唯一入口表）

> 状态：当前有效
> 适用范围：本工作区 `src/srm27_navigation/srm27_nav_bringup`
> 最后核对：2026-11-14
>
> 本文件是 launch 入口的**唯一权威表**。新增/修改入口时先改这里，再改代码；
> 其他文档里的入口说明若与本表冲突，以本表为准。

## 1. 四个正式入口

| # | 用途 | 推荐命令 | 入口 launch | 默认控制器 | 默认定位/前端 | 默认参数 | 速度出口 |
|---|---|---|---|---|---|---|---|
| 1 | 实车导航 | `./scripts/bringup/start_real_nav.sh` | `nav_srm_real_launch.py` | MINCO (`srm27_minco_controller::MincoMpcController`) | 未显式指定时走 `--lio`：`zlio` + 静态 `map->odom`；`--reloc` 才用 small_gicp + 先验 PCD | `config/real/current/nav2_params_srm_minco_fakeyaw.yaml` | `cmd_vel_nav_fake → fake_vel_transform → cmd_vel_nav_body → srm_cmd_mux → cmd_vel_chassis → srm27_nav_protocol` |
| 2 | 仿真导航 | `./scripts/bringup/start_sim_nav.sh` | `nav_srm_simulation_launch.py` | MINCO（唯一支持；`-c omni` 已拒绝） | 默认 Gazebo 真值里程计；`--lio` 切 ZLIO（默认前端）或 `--lio-frontend point_lio`；`--gicp` 用先验 PCD | `config/simulation/current/nav2_params_srm_fakeyaw.yaml` | `cmd_vel_nav_fake → fake_vel_transform → cmd_vel_nav_body → srm_cmd_mux → cmd_vel_sim → 适配器 → Gazebo` |
| 3 | 实车建图 | `./scripts/bringup/start_real_slam.sh` | `real_mapping_launch.py` | 不涉及（建图链路） | `zlio`（`--lio-frontend point_lio` 回退） | `config/real/current/mapping_params.yaml` + `zlio_mid360.yaml` | 不涉及导航；底盘串口由 `srm27_nav_protocol` 单独启动 |
| 4 | 仿真建图 | `./scripts/bringup/start_sim_slam.sh` | `sim_slam_launch.py` | 不涉及（建图链路） | Point-LIO | `config/simulation/current/nav2_params_srm.yaml` + Point-LIO `mid360.yaml` | 不涉及导航；Gazebo 由脚本单独拉起 |

### 入口细节与默认值

**1）实车导航**

- `start_real_nav.sh` 的 `CONTROLLER` 默认 `minco`；`--controller omni` 仍保留为**正式回退链**，
  参数文件 `config/real/current/nav2_params_srm.yaml`，但它与当前 fakeyaw/mux 链不是同一条，
  现场切换前先看脚本 `--help` 里的提示。
- `LIO_FRONTEND` 默认 `zlio`。未给 `--slam/--reloc/--lio/--static` 时，脚本按前端选择默认：
  `zlio → --lio`（静态 `map->odom`），`point_lio → --reloc`（需要先验 PCD）。
- 地图未给 `-m` 时，从包内 `map/real|simulation/` 与工作区 `maps/` 里选**最近修改**的一份，
  并在终端打印实际选择；实车请优先显式 `-m <site>`。
- `--reloc` 需要 `maps/pcd/<site>_mapframe.pcd`；`point_lio` + `--reloc` 是实车 GICP 组合。
- 速度链由 `fake_vel_transform` 把 `gimbal_yaw_fake` 下的平移一次性转到 `base_link`；
  `srm_cmd_mux` 只透传平移并叠加决策层 `rotation_cmd` 的 `wz`。`yaw_mode` 默认 `xy_only`。

**2）仿真导航**

- 脚本默认世界 `rmuc_2025`；参数文件固定 `nav2_params_srm_fakeyaw.yaml`（MINCO + fakeyaw）。
- 默认不使用 LIO（Gazebo 真值里程计）；`--lio` 才接 ZLIO/Point-LIO。仿真里控制器只出平移，
  底盘 `wz` 由 `rotation_cmd` 在 `srm_cmd_mux` 合成。
- 默认命名空间 `red_standard_robot1`；Gazebo 与导航分两个标签页启动。

**3）实车建图**

- 一条 launch 起：雷达驱动（可关）、车体 TF、Point-LIO/ZLIO、`loam_interface`/`zlio_bridge`、
  `sensor_scan_generation`、`pointcloud_to_laserscan`、`slam_toolbox`、RViz。
- `--save <name>` 调 `slam_toolbox` 的 `save_map` + `serialize_map`，输出
  `maps/<name>/<name>.{yaml,pgm,posegraph,data}`。
- 累积点云落在外部数据目录：`~/srm_nav_27_data/generated_pcd/point_lio/scans.pcd`
  （`SRM27_PCD_DIR` 可覆盖），不再写源码包里的 `point_lio/PCD/`。
- `--record-bag` 录到 `~/srm_nav_27_data/bags/`，用于离线重建 PCD。

**4）仿真建图**

- `sim_slam_launch.py` 把 `nav_srm_simulation_launch.py` 固定为 `slam:=True`，
  另起 Gazebo、Point-LIO、slam_toolbox、`map_saver_server`。
- 退出时 Point-LIO 写 `~/srm_nav_27_data/generated_pcd/point_lio/scans.pcd`；
  `--collect` 收到 `maps/pcd/<name>.pcd`。
- 2D 栅格图用 `map_saver` 服务另存；脚本会打印现成命令。

## 2. launch 文件分类（16 个 canonical + 16 个前缀别名）

路径均相对 `src/srm27_navigation/srm27_nav_bringup/launch/`。

ROS 2 的 `ros2 launch <pkg> <file>` 会在 `share/launch` 下按 **basename 递归搜索**；
同一 basename 出现两次会直接报 `file ... was found more than once`。因此：

- **根目录 `<name>_launch.py` 是 canonical**，旧命令原样可用；
- 分组目录放**唯一前缀名**的别名（`real_*.py` / `sim_*.py` / `common_*.py` / `compat_*.py`），
  内部 `importlib` 加载根目录 canonical，逻辑只有一份。

| 根目录 canonical（旧命令） | 分组别名（新命令） | 状态 | 角色 | 说明 |
|---|---|---|---|---|
| `nav_srm_real_launch.py` | `real/real_nav_srm_real_launch.py` | **current** | 正式入口 1 | 实车导航唯一链路：fakeyaw + mux + MINCO |
| `nav_srm_simulation_launch.py` | `simulation/sim_nav_srm_simulation_launch.py` | **current** | 正式入口 2 | 仿真导航唯一链路 |
| `real_mapping_launch.py` | `real/real_mapping_entry_launch.py` | **current** | 正式入口 3 | 实车 MID360 建图 |
| `sim_slam_launch.py` | `simulation/sim_slam_entry_launch.py` | **current** | 正式入口 4 | 仿真 SLAM 建图（薄包装） |
| `nav2_stack_launch.py` | `common/common_nav2_stack_launch.py` | **current** | 内部 | Nav2 主栈，按 `slam` 二选一 |
| `navigation_launch.py` | `common/common_navigation_launch.py` | **current** | 内部 | 控制器/规划器/行为服务器，速度出口 |
| `localization_launch.py` | `common/common_localization_launch.py` | **current** | 内部 | Point-LIO / ZLIO + GICP + 静态 TF |
| `nav2_slam_launch.py` | `common/common_nav2_slam_launch.py` | **current** | 内部 | `slam:=True` 分支（SLAM + map_saver） |
| `rviz_launch.py` | `common/common_rviz_launch.py` | **current** | 内部 | RViz |
| `joy_teleop_launch.py` | `common/common_joy_teleop_launch.py` | **current** | 内部 | 手柄遥控 |
| `real_robot_state_publisher_launch.py` | `real/real_tf_entry_launch.py` | **current** | 工具 | 只发实车 TF，便于分步调试 |
| `srm_simulation_record_launch.py` | `simulation/sim_record_entry_launch.py` | **current** | 诊断 | 仿真回归录包 + metadata |
| `nav_multi_simulation_launch.py` | `simulation/sim_multi_robot_launch.py` | **experimental** | 多机 | 多机器人仿真，不在四个正式入口内 |
| `nav_real_launch.py` | `compat/compat_nav_real_launch.py` | **compat** | 旧实车入口 | 旧上游链路；参数默认 `config/real/legacy/nav2_params_upstream.yaml`，不要用于当前实车 |
| `nav_simulation_launch.py` | `compat/compat_nav_simulation_launch.py` | **deprecated** | 弃用壳 | 只转发到 `nav_srm_simulation_launch.py`，保留旧命令 |
| `standalone_robot_state_publisher_launch.py` | `compat/compat_standalone_tf_launch.py` | **compat** | 独立 TF 调试 | 与建图/导航共用同一份 URDF；不属于正式入口 |

状态含义：

- **current**：当前维护、可作为正式入口或正式内部依赖。
- **compat**：仍能跑，但只用于旧命令/旧链路兼容；新工作不要依赖。
- **deprecated**：明确弃用，只保留转发；下一轮清理可归档。
- **experimental**：实验入口，未纳入验收。

## 3. 入口依赖关系

```text
start_real_nav.sh
└── nav_srm_real_launch.py
    ├── real_robot_state_publisher_launch.py   (可选，脚本分步启动时)
    ├── nav2_stack_launch.py
    │   ├── localization_launch.py  (slam:=False)
    │   └── nav2_slam_launch.py     (slam:=True)
    │       └── navigation_launch.py
    └── rviz_launch.py

start_sim_nav.sh
├── srm27_gazebo_simulator/srm_sim.launch.py   (标签页 1)
└── nav_srm_simulation_launch.py               (标签页 2)
    ├── nav2_stack_launch.py
    └── rviz_launch.py

start_real_slam.sh
└── real_mapping_launch.py
    ├── srm27_robot_description + gimbal_joint_state_relay
    ├── livox_ros_driver2 / point_lio / zlio_bridge
    ├── loam_interface + sensor_scan_generation
    ├── pointcloud_to_laserscan
    └── slam_toolbox + rviz

start_sim_slam.sh
├── srm27_gazebo_simulator/srm_sim.launch.py
└── sim_slam_launch.py
    └── nav_srm_simulation_launch.py (slam:=True)
        └── nav2_slam_launch.py (Point-LIO + slam_toolbox)
```

## 4. fake_vel_transform 的当前职责（重要）

`fake_vel_transform` **不是可删除的旧链路残留**。当前 MINCO/fakeyaw 实车与仿真链路都依赖它：

- 导航参考系是 `gimbal_yaw_fake`（与 `odom` 平行、朝向稳定），
  而底盘执行系是 `base_link`/云台系；
- `fake_vel_transform` 订阅 `cmd_vel_nav_fake`，按云台/底盘夹角把平移旋到
  `cmd_vel_nav_body`，并保持 `wz` 语义；
- 两个正式导航入口都要求 `use_fakeyaw_transform=True`、`command_frame=gimbal_yaw_fake`，
  launch 启动时会核对参数文件里的 `FollowPath.command_frame`，不一致直接报错。

只有旧步兵 `gimbal_yaw_fake`/`MecanumDrive2` 那套仿真执行器已删除；
**组件本身仍在当前链路中**，归档/删除必须另开改造，不能按“旧链路”处理。

## 5. 参数文件分类

路径相对 `src/srm27_navigation/srm27_nav_bringup/config/`。分组目录是 canonical，
原路径是兼容符号链接。

| 文件 | 状态 | 用途 |
|---|---|---|
| `real/current/nav2_params_srm_minco_fakeyaw.yaml` | **current** | 正式入口 1 默认（MINCO + fakeyaw） |
| `real/current/nav2_params_srm_minco.yaml` | **current** | MINCO 无 fakeyaw 的 A/B 对照 |
| `real/current/nav2_params_srm.yaml` | **current** | Omni 回退链（`--controller omni`） |
| `real/current/mapping_params.yaml` | **current** | 实车建图/slam_toolbox |
| `real/current/speed_limits.yaml` | **current** | 实车速度限幅唯一来源 |
| `real/current/zlio_mid360.yaml` | **current** | 实车 ZLIO 前端 |
| `real/legacy/nav2_params_upstream.yaml` | **legacy** | 上游基线，仅旧 `nav_real_launch.py` |
| `simulation/current/nav2_params_srm_fakeyaw.yaml` | **current** | 正式入口 2 默认（MINCO + fakeyaw） |
| `simulation/current/nav2_params_srm.yaml` | **current** | 仿真基线 / 内部 launch 默认 |
| `simulation/current/nav2_params_srm_minco.yaml` | **current** | MINCO A/B 对照 |
| `simulation/current/speed_limits.yaml` | **current** | 仿真速度限幅唯一来源 |
| `simulation/current/zlio_mid360.yaml` | **current** | 仿真 ZLIO 前端 |
| `simulation/legacy/nav2_params.yaml` | **legacy** | 上游基线，仅内部 launch 兜底默认 |

参数文件里若出现“旧链路/历史实验”注释，先对照本表；不要仅凭文件名判断 current。

## 6. 兼容策略与后续

- launch：根目录 `<name>_launch.py` 是 canonical；分组目录是唯一前缀名的别名。
  旧命令 `ros2 launch ... <name>` 保持可用，新命令可用 `real_nav_srm_real_launch.py` 等前缀名。
- config：根目录文件是指向 `current/` / `legacy/` 的兼容符号链接；参数内容只有一份。
- 新增代码请引用 `config/real/current/...` 这类分组路径；launch 新命令用前缀别名
  （`real_*` / `sim_*` / `common_*` / `compat_*`），逻辑仍以根目录 canonical 为准。
- 下一轮若确认现场脚本与文档全部切换，可把 canonical 改名为前缀名并删除旧根文件，
  同时更新 `test_cli_parity.py`、`test_fakeyaw_*_wiring.py`、`test_lio_frontend_wiring.py`
  里的路径常量。
- 删除任何 launch 前先跑：

```bash
ros2 launch srm27_nav_bringup <name> --show-args
colcon test --packages-select srm27_nav_bringup
```
