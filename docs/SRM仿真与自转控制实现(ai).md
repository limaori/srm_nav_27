# SRM 仿真与自转控制实现说明

> **本文由 AI 生成**，依据 `docs/SRM仿真与导航自转控制实施方案.md` 实施完成后的源码逐项核对写成，
> 用于记录落地后的实际接口与用法。文中所有话题名、参数名、默认值均取自工作空间当前源码；
> 若与源码不一致，以源码为准。

本文对应的实现范围：SRM 模型与几何单一来源、仿真速度执行、导航平移与独立自转的速度合成。
雷达采集、点云处理、定位与 Nav2 规划链路沿用原有实现，本文只描述它们的接入点。
设计依据见 [`SRM仿真与导航自转控制实施方案.md`](./SRM仿真与导航自转控制实施方案.md)。

---

## 1. 三个新包的职责与目录结构

### 1.1 `srm27_robot_description`：SRM 几何与外参的唯一定义

```text
srm27_robot_description/
├── config/srm27_sentry_geometry.yaml   # 唯一定义：底盘、轮组、雷达外参、碰撞包络
├── srm27_robot_description/
│   ├── __init__.py                     # get_share_directory / get_geometry_file / load_geometry
│   └── model_builder.py                # build_urdf() / build_sdf()，由同一份 YAML 生成
├── scripts/generate_robot_model.py     # CLI：--format urdf|sdf
├── urdf/srm27_sentry.urdf.xacro        # 与几何 YAML 同步的公共 xacro 版本
├── meshes/mid360.stl                   # 本包内的雷达 mesh 副本
├── launch/robot_description_launch.py  # 发布 robot_description / 静态 TF
└── test/test_model_builder.py          # 模型生成用例（pytest 兼容）
```

要点：

- `config/srm27_sentry_geometry.yaml` 是 URDF、SDF 和启动脚本的共同数据源，改尺寸或安装位姿只改这一份。
- 当前实车 launch（`real_robot_state_publisher_launch.py` / `real_mapping_launch.py`）仍然加载
  `srm27_nav_bringup/urdf/sentry_robot_cylinder.xacro`；`urdf/srm27_sentry.urdf.xacro` 是描述包内
  与几何 YAML 数值同步的公共版本，供后续实车链路统一切换，目前没有 launch 引用它。
- 当前几何：底盘半径 `0.27 m`、高 `0.20 m`、质量 `8.2 kg`；四轮半径 `0.075 m`、宽 `0.04 m`、
  轮心 `(±0.18, ±0.18, 0.075)`；雷达帧 `front_mid360`，安装位姿
  `xyz=[0.15, -0.15, 0.22]`、`rpy=[-0.06981317007977318, 0, -1.5707963267948966]`；
  导航碰撞包络半径 `collision.envelope_radius: 0.33 m`（圆柱半径不是整车最大边界）。
- `build_sdf()` 生成的模型只含圆柱底盘、四个轮组、MID360 和底盘 IMU（`chassis_imu`，200 Hz）。
  **没有**装甲、灯条、射击、云台关节链，也**没有** `base_footprint`、`gimbal_yaw`、`gimbal_yaw_fake`
  这些中间坐标系；`base_link` 是与实际底盘刚性固定、随底盘自转的根坐标系。
- 传感器名保持不变（`front_mid360_lidar` / `front_mid360_imu`），因此现有 bridge、点云处理与定位无需改动。
- 命令行生成模型：

  ```bash
  ros2 run srm27_robot_description generate_robot_model.py --format urdf
  ros2 run srm27_robot_description generate_robot_model.py --format sdf --output /tmp/srm_sentry.sdf
  ```

### 1.2 `srm27_chassis_control`：速度合成与独立自转

```text
srm27_chassis_control/
├── config/srm_chassis_control.yaml     # 三个节点的权威默认参数
├── launch/srm_chassis_control.launch.py
├── include/srm27_chassis_control/
│   ├── velocity_mix.hpp                # 纯逻辑：限幅与合成
│   ├── rotation_waveform.hpp           # 纯逻辑：波形与参数校验
│   └── twist_watchdog.hpp              # 纯逻辑：单调时钟超时判定
├── src/
│   ├── cmd_mux_node.cpp                # 可执行文件 cmd_mux，节点名 srm_cmd_mux
│   ├── rotation_controller_node.cpp    # 可执行文件 rotation_controller
│   ├── rotation_test_sender_node.cpp   # 可执行文件 rotation_test_sender
│   ├── velocity_mix.cpp / rotation_waveform.cpp / twist_watchdog.cpp
└── test/                               # 两个 gtest：速度合成、波形与超时
```

职责边界：

- `srm_cmd_mux`：唯一合成点，也是 `cmd_vel_sim` 的唯一发布者。只取导航输入的 `linear.x/linear.y`，
  **丢弃导航输入的 `angular.z` 并统计次数**；`wz` 只来自独立自转输入。
- `rotation_controller`：只订阅 `rotation_cmd`、只发布 `rotation_velocity`，只使用 `angular.z`，
  不发布也不修改任何导航话题。
- `rotation_test_sender`：按 launch / YAML 参数产生 `stop`、`constant`、`periodic` 三种自转波形。
- 纯逻辑与 ROS 解耦（速度合成、超时判定、波形），由 gtest 覆盖；节点层只做收发与定时。
- 服务名使用 `~/` 前缀（ROS 2 中只有 `~/xxx` 才展开为 `/<ns>/<node>/xxx`，普通相对名只到 `/<ns>/xxx`）。

### 1.3 `srm27_gazebo_simulator`：SRM 速度执行与仿真入口

```text
srm27_gazebo_simulator/
├── config/srm_sim.yaml                 # 世界名、Gazebo GUI、SRM 初始位姿、速度执行参数
├── config/ros_gz_bridge.yaml           # /clock、真值里程计、joint_states、雷达点云与 IMU
├── worlds/srm_empty.sdf                # 空场调试世界（物理步长 1 ms）
├── launch/srm_sim.launch.py            # 仿真入口
├── plugins/srm_velocity_system/        # SRM 自有 Gazebo Fortress system 插件
│   ├── SrmVelocitySystem.hh
│   └── SrmVelocitySystem.cc            # srm27::gazebo::systems::SrmVelocitySystem
├── src/srm_velocity_adapter_node.cpp   # 可执行文件 srm_velocity_adapter
└── env-hooks/srm_velocity_system.dsv.in # 把插件库目录加入 Gazebo 插件搜索路径
```

要点：

- 插件库名 `libsrm_velocity_system.so`，插件名 `srm27::gazebo::systems::SrmVelocitySystem`。
- `PreUpdate` 读取最新速度输入缓存，用 `Link::SetLinearVelocity` / `SetAngularVelocity` 按
  **Link 坐标系（车体系）** 设置平面速度，保留 z 平移与 roll/pitch 角速度；仿真暂停、仿真重置
  和命令超时（用仿真时间）时清零。**不做**轮速 PID、力矩控制、底盘跟随云台或额外旋转策略。
- `PostUpdate` 按 50 Hz 在 Gazebo Transport 发布真值里程计 `<model>/odometry`，含实际 pose 与 twist。
- `srm_velocity_adapter` 订阅 ROS 侧 `cmd_vel_sim`（车体系），校验 NaN/Inf（含非有限值时按零速处理并计数），
  转成 `ignition.msgs.Twist` 发布到 `<robot_name>/cmd_vel`；守护线程用**单调时钟**做超时清零并持续发零速。
- **底盘速度命令不走 `ros_gz_bridge.yaml`**：由适配器直接写 Gazebo Transport，避免桥接直通绕过校验。
- 本包只提供 SRM 模型与仿真入口；场地素材（世界 SDF、GUI 配置、mid360 模型）由 `rmu_gazebo_simulator` 提供。

---

## 2. 速度 / 自转接口表

下表话题名均为**相对机器人命名空间**的名字（默认命名空间见 §4）。所有速度消息统一为
`geometry_msgs/msg/Twist`，坐标系约定为车体系：x 前、y 左、z 上。

| 话题（相对 `<ns>`） | 类型 | 发布者 | 订阅者 | 字段含义 |
| --- | --- | --- | --- | --- |
| `cmd_vel_nav` | `geometry_msgs/msg/Twist` | 导航链路（`velocity_smoother` 或 `controller_server`） | `srm_cmd_mux` | `linear.x/y` 导航平移；`angular.z` 一律被 mux 丢弃 |
| `rotation_cmd` | `geometry_msgs/msg/Twist` | `rotation_test_sender` | `rotation_controller` | 只用 `angular.z`，自转请求（rad/s） |
| `rotation_velocity` | `geometry_msgs/msg/Twist` | `rotation_controller` | `srm_cmd_mux` | 只用 `angular.z`，独立自转速度 |
| `cmd_vel_sim` | `geometry_msgs/msg/Twist` | `srm_cmd_mux`（唯一发布者） | `srm_velocity_adapter` | 合成后的最终执行命令，车体系 |
| `odometry` | `nav_msgs/msg/Odometry` | `simulation_ground_truth_odometry`（仿真真值模式） | Nav2 | 平面真值里程计，含实际 yaw 与 wz |
| `chassis_odometry_gt` | `nav_msgs/msg/Odometry` | `ros_gz_bridge`（桥接插件真值里程计） | `simulation_ground_truth_odometry` | Gazebo 插件按 50 Hz 发布的原始真值 |
| `diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 4 个节点（mux、rotation_controller、rotation_test_sender、速度适配器） | 诊断工具 | 超时、限幅、丢弃计数与波形状态 |
| `<robot_name>/cmd_vel`（Gazebo Transport） | `ignition.msgs.Twist` | `srm_velocity_adapter` | `SrmVelocitySystem` | Gazebo 侧速度输入 |

### 2.1 `srm_cmd_mux` 参数（`config/srm_chassis_control.yaml`）

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `nav_topic` | `cmd_vel_nav` | 导航平移输入 |
| `rotation_topic` | `rotation_velocity` | 独立自转输入 |
| `output_topic` | `cmd_vel_sim` | 合成输出 |
| `publish_rate` | `200.0` | 合成频率（Hz） |
| `nav_timeout` | `0.3` | 导航输入超时（s），超时清零 `vx/vy` |
| `rotation_timeout` | `0.1` | 自转输入超时（s），超时清零 `wz` |
| `vx_max` / `vy_max` | `0.5` / `0.5` | 单轴平移上限（m/s） |
| `v_max` | `0.5` | 平移合速度上限（m/s，按向量模长） |
| `wz_max` | `2.0` | 自转角速度上限（rad/s） |
| `diagnostics_period` | `0.5` | 诊断发布周期（s） |

### 2.2 `rotation_controller` 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `input_topic` / `output_topic` | `rotation_cmd` / `rotation_velocity` | 自转请求与自转输出 |
| `publish_rate` | `200.0` | 输出频率（Hz） |
| `request_timeout` | `0.5` | 自转请求超时（s），超时清零 |
| `max_angular_accel` | `0.0` | 角加速度上限（rad/s²），0 表示不限制 |
| `wz_max` | `2.0` | 输出角速度上限（rad/s） |
| `diagnostics_period` | `0.5` | 诊断发布周期（s） |

### 2.3 `rotation_test_sender` 参数与波形

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `output_topic` | `rotation_cmd` | 输出话题 |
| `publish_rate` | `200.0` | 发布频率（Hz） |
| `enabled` | `true` | 启动时是否启用 |
| `rotation_mode` | `stop` | `stop` / `constant` / `periodic` |
| `angular_speed` | `0.0` | 恒速模式的有符号角速度（rad/s） |
| `offset` / `amplitude` | `0.0` / `0.0` | 周期模式的平均角速度与非负变化幅度（rad/s） |
| `period` / `phase` | `4.0` / `0.0` | 周期（s）与初相位（rad） |
| `sine_wave` | `true` | true 为正弦，false 为方波 |
| `diagnostics_period` | `0.5` | 诊断发布周期（s） |

波形计算（`rotation_waveform.cpp`）：

```text
stop      : 0
constant  : angular_speed
periodic  : offset + amplitude * sin(2πt / period + phase)       # sine_wave = true
            offset + amplitude * sign(sin(2πt / period + phase)) # sine_wave = false（方波）
```

方波的 `sign` 由 `sin` 的符号决定，`sin` 恰为 0 时取正幅度，因此输出始终在 `offset ± amplitude`
之间跳变，而不会出现 0。

- 波形时间取**仿真时间**，从“启用”或“波形参数改变”时开始计算；周期性发布本身不重置相位。
- 拒绝非有限速度、负幅度，以及周期模式下的非正周期（`ros2 param set` 会返回中文原因并拒绝写入）。
- `offset=0` 可实现周期换向；`offset>amplitude` 可实现同方向周期变速。
- 停止命令与超时清零优先直接归零，不做减速斜坡；只有正常变速才受 `max_angular_accel` 限制。

### 2.4 服务

| 服务 | 类型 | 作用 |
| --- | --- | --- |
| `/<ns>/srm_cmd_mux/stop_all` | `std_srvs/srv/Trigger` | 两路输入清零并保持零输出，直到 `resume_all` |
| `/<ns>/srm_cmd_mux/resume_all` | `std_srvs/srv/Trigger` | 重新启用两路输入，等待重新下发速度 |
| `/<ns>/rotation_controller/clear_rotation` | `std_srvs/srv/Trigger` | 清空自转目标并清零输出 |
| `/<ns>/rotation_test_sender/enable` | `std_srvs/srv/Trigger` | 启用发送器，相位从零开始 |
| `/<ns>/rotation_test_sender/disable` | `std_srvs/srv/Trigger` | 暂停发送器（接收侧超时后自动清零） |
| `/<ns>/rotation_test_sender/reset_waveform` | `std_srvs/srv/Trigger` | 波形相位从头开始 |

---

## 3. 合成与安全优先级

### 3.1 合成公式（`velocity_mix.cpp`）

```text
vx = clamp(nav_vx, -vx_max, vx_max)          # 先单轴限幅
vy = clamp(nav_vy, -vy_max, vy_max)
scale = min(1, v_max / hypot(vx, vy))        # 再按向量模长限幅；零向量时 scale = 1
vx *= scale
vy *= scale
wz = clamp(rotation_wz, -wz_max, wz_max)     # 自转独立限幅，与平移无关
```

非有限输入按 0 处理；`v_max <= 0` 时按不允许运动处理（`scale = 0`），不会出现不受限的平移。

### 3.2 安全优先级（从高到低）

1. **`stop_all` / 急停**：两路输入立即清零并保持零输出，直到显式 `resume_all`。
2. **输入超时清零**：导航输入超过 `nav_timeout`（0.3 s）清零平移，自转输入超过 `rotation_timeout`（0.1 s）
   清零自转，两者互不影响（导航停了自转可以继续）。
3. **执行侧超时清零**：`srm_velocity_adapter` 超过 `command_timeout`（0.1 s）未收到 `cmd_vel_sim`
   就清零并持续发零速；插件侧再用仿真时间做一次同样的超时保护。
4. **暂停 / 重置 / 退出**：插件在仿真暂停与重置时清空速度缓存；mux 与适配器退出前各写一次零速。
5. **默认拒绝**：导航输入的 `angular.z` 一律丢弃；NaN/Inf 一律按零速处理并计数。

### 3.3 导航 `wz` 的所有权

- 控制器参数 `enable_rotation: false`、`use_rotate_to_heading: false`，导航链路本就不应产生自转角速度。
- 恢复行为与行为树同样输出到导航速度出口，其中若出现角速度分量也会被 mux 丢弃。
- 自转角速度的**唯一**来源是 `rotation_velocity`，因此“导航平移 + 底盘自转”不会互相覆盖。
- 计时全部用**单调时钟**（`twist_watchdog`），只有周期波形用仿真时间；暂停时波形相位自然冻结。

---

## 4. 启动方式

### 4.1 一键脚本（推荐现场使用）

```bash
./script/start_sim_nav.sh -h                    # 打印文件头注释里的权威用法
./script/start_sim_nav.sh                       # 默认 rmuc_2025 + 隧道地图，不自转
./script/start_sim_nav.sh -m rmuc_2025          # 换普通场地地图
./script/start_sim_nav.sh -w srm_empty --run    # 空场 + Gazebo 直接开始运行
./script/start_sim_nav.sh --rotation-mode constant --rotation-speed 1.0
./script/start_sim_nav.sh --rotation-mode periodic \
    --rotation-offset 1.0 --rotation-amplitude 0.5 --rotation-period 4.0
./script/start_sim_nav.sh --no-rviz --no-rotation   # 只跑仿真 + 导航
DRY_RUN=1 ./script/start_sim_nav.sh             # 只打印将执行的命令
```

脚本开三个标签页：

| 标签页 | 内容 |
| --- | --- |
| 1 | `ros2 launch srm27_gazebo_simulator srm_sim.launch.py world:=<世界> gui:=<bool> run_immediately:=<bool>` |
| 2 | `ros2 launch srm27_nav_bringup nav_srm_simulation_launch.py ...`（含速度合成与自转发送器） |
| 3 | 可选手柄自转（`--teleop`，默认关闭）：`srm27_teleop_twist_joy_node` 输出 remap 到 `rotation_cmd` |

常用参数：`-w/--world`（`rmuc_2025` / `rmuc_2024` / `rmul_2024` / `rmul_2025` / `srm_empty`）、
`-m/--map`（地图名或 YAML 绝对路径）、`-p/--params`（默认 `config/simulation/nav2_params_srm.yaml`）、
`--gui/--no-gui`、`--run/--no-run`、`--smoother/--no-smoother`、`--rviz/--no-rviz`、
`--rotation-mode/-speed/-offset/-amplitude/-period/-phase/-wave`、`--rotation/--no-rotation`、`--teleop/--no-teleop`。
世界名从 `srm27_gazebo_simulator/config/srm_sim.yaml` 读取，`-w` 覆盖它。

脚本保留的保护：`flock` 保证 Gazebo 单实例、清理上次 launch 遗留的孤儿节点（避免两个 `/clock`）、
地图与参数文件存在性校验、`--params` 必须是绝对路径、布尔值归一化。

调试辅助脚本：

| 脚本 | 用途 |
| --- | --- |
| `script/clean_sim_processes.sh` | 清理本工作空间的全部仿真残留进程（Gazebo / bridge / 适配器 / Nav2 / RViz） |
| `script/diag_nav_abort.sh` | 单次安全的导航诊断：启动前检查残留、独立进程组启动、内存看门狗、日志落到 `log_diag/nav_abort/` |
| `script/kill_gzb.sh` / `script/kill_rviz.sh` | 分别清理 Gazebo 与 RViz |

`diag_nav_abort.sh` 可用环境变量调整场景：`WORLD`、`GOAL_X`、`GOAL_Y`。

### 4.2 手动 bringup（不用脚本）

```bash
# 终端 1：Gazebo 世界 + SRM 模型 + 传感器桥接 + 速度适配器
ros2 launch srm27_gazebo_simulator srm_sim.launch.py

# 终端 2：点云转换 + 真值里程计 + Nav2 + 速度合成 + RViz
ros2 launch srm27_nav_bringup nav_srm_simulation_launch.py \
  namespace:=red_standard_robot1 \
  world:=rmuc_2025 \
  map:=/absolute/path/to/map/simulation/rmuc_2025_tunnel.yaml \
  params_file:=/absolute/path/to/config/simulation/nav2_params_srm.yaml \
  slam:=False use_pcd_localization:=False use_lio_odometry:=False \
  use_sim_time:=True use_rviz:=True \
  use_velocity_smoother:=True rotation_mode:=stop
```

也可以让导航入口一并拉起仿真（`start_simulation:=true`；该入口的 `use_sim_time` 默认已是 `True`）：

```bash
ros2 launch srm27_nav_bringup nav_srm_simulation_launch.py \
  start_simulation:=true map:=/absolute/path/to/<MAP>.yaml
```

`nav_srm_simulation_launch.py` 的默认值：`namespace:=red_standard_robot1`、`world:=rmuc_2025`、
`params_file:=config/simulation/nav2_params_srm.yaml`、`start_simulation:=False`、
`start_chassis_control:=True`、`start_rotation_sender:=True`、`rotation_mode:=stop`、
`use_velocity_smoother:=True`、`cmd_vel_nav_topic:=cmd_vel_nav`、`use_rviz:=True`。

> [!NOTE]
> 速度执行链路只在 `nav_srm_simulation_launch.py` 里对接：它把
> `use_fake_vel_transform` 强制为 `False`，并把导航速度出口设为 `cmd_vel_nav`。

### 4.3 空场调试

```bash
# 只跑仿真（headless，直接运行）
ros2 launch srm27_gazebo_simulator srm_sim.launch.py world:=srm_empty gui:=false run_immediately:=true

# 手动下发纯平移 / 纯自转命令（车体系）
ros2 topic pub -r 20 /red_standard_robot1/cmd_vel_sim geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.0}}"
ros2 topic pub -r 20 /red_standard_robot1/cmd_vel_sim geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.3}, angular: {z: 1.0}}"
```

世界解析顺序：`srm27_gazebo_simulator/worlds/<world>.sdf` →
`rmu_gazebo_simulator/resource/worlds/<world>_world.sdf`；`world_sdf:=<绝对路径>` 优先级最高。
Gazebo 默认**以暂停启动**，`run_immediately:=true` 才直接运行。

### 4.4 旧入口的当前状态

| 旧入口 / 旧组件 | 状态 |
| --- | --- |
| `srm27_nav_bringup nav_simulation_launch.py` | 已改为**弃用壳**，转发到 `nav_srm_simulation_launch.py`，默认参数文件改成 `nav2_params_srm.yaml` |
| `rmu_gazebo_simulator bringup_sim.launch.py` | 已**下线**，运行时直接报错并提示替代入口 |
| `rmu_gazebo_simulator` 的 SRM 模型、`spawn_robots.launch.py`、`gz_world.yaml`、`base_params.yaml`、`ros_gz_bridge.yaml` | 已删除 |
| `MecanumDrive2` 底盘插件、`rmoss_gz_base` 底盘/云台控制器 | SRM 链路不再使用，替换为 `srm_velocity_system` + `srm_velocity_adapter` |
| `gimbal_yaw_fake` / `fake_vel_transform` | 旧步兵链路；SRM 仿真已删除该链路，`use_fake_vel_transform` 默认 `True` 仅为兼容其他配置保留 |

### 4.5 实施过程中发现并修掉的问题

| 问题 | 现象 | 处理 |
| --- | --- | --- |
| `nav_srm_simulation_launch.py` 只给 `ign_sim_pointcloud_tool` 传 `use_sim_time` | 节点的 `pcd_topic` / `n_scan` / `horizon_scan` / `ang_res_y` 等参数回退到源码默认值，其中 `ang_res_y` 由 1.903…° 变成 1.0°，转换出的点云垂直角度错误，影响地形与障碍高度 | 改为传 `configured_params`（按命名空间 `root_key` 重写 `use_sim_time` 的参数文件），与旧入口一致；实测 `ros2 param get ... ang_res_y` 恢复为 1.903225806451613 |
| 真值里程计雷达外参硬编码 | 旧脚本里写死了另一套模型的外参，与 SRM 的 `front_mid360` 安装位姿不一致 | 改为从 `srm27_robot_description` 的几何 YAML 读取，读不到时回退并打印告警 |
| ROS 2 相对服务名只展开到命名空间 | `create_service("stop_all")` 得到 `/<ns>/stop_all`，与文档中的 `/<ns>/<node>/stop_all` 不符 | 改用 `~/stop_all` 等写法，服务名变成 `/<ns>/srm_cmd_mux/stop_all` |

---

## 5. 测试矩阵怎么跑

在空场（`world:=srm_empty`）先做前四组，确认符号与量值正确，再进场地叠导航。

| 场景 | 下发方式 | 期望结果 |
| --- | --- | --- |
| 纯 `vx` | `cmd_vel_sim` 发 `linear.x=0.3` | 沿 +x 前进，里程计 `twist.linear.x≈+0.300` |
| 纯 `vy` | `cmd_vel_sim` 发 `linear.y=0.3` | 沿 +y（左）平移，x 不变，`twist.linear.y≈+0.298` |
| `vx+vy` | `cmd_vel_sim` 发 `linear.x=0.3, linear.y=0.3` | 45° 斜向，合速度不超过 `v_max=0.5` |
| 纯 `wz` | `cmd_vel_sim` 发 `angular.z=1.0` / `-1.0` | 逆/顺时针自转，`twist.angular.z≈±0.99` |
| 平移 + 恒速自转 | `cmd_vel_nav` 发 `(0.4, 0.4)`，`rotation_velocity` 发 `wz=1.5` | `cmd_vel_sim=(0.3536, 0.3536, wz=1.5)`：平移按模长限幅、导航角速度被丢弃 |
| 平移 + 周期自转 | 导航平移不变，发送器 `periodic`（offset 1.0 / amplitude 0.5 / period 4 s） | 输出落在 0.5～1.5 rad/s，周期 4 s，平移不受影响 |

上表的 `twist` 指实际执行速度：空场只跑 `srm_sim.launch.py` 时读 `/<robot_name>/chassis_odometry_gt`
（插件的 Link 系真值速度，50 Hz），跑完整导航入口时也可读真值节点转发的 `/<ns>/odometry`。

每轮至少记录这些话题（`<ns>` 默认 `red_standard_robot1`）：

```bash
ros2 topic echo /<ns>/cmd_vel_nav --once        # 导航请求的速度（angular.z 应为 0）
ros2 topic echo /<ns>/rotation_cmd --once       # 自转请求
ros2 topic echo /<ns>/rotation_velocity --once  # 自转输出
ros2 topic echo /<ns>/cmd_vel_sim --once        # 合成后的执行命令
ros2 topic hz   /<ns>/cmd_vel_sim               # 应约 200 Hz
ros2 topic echo /<ns>/odometry --once           # 实际位姿与实际速度（真值）
ros2 topic echo /<ns>/diagnostics --once        # 超时 / 限幅 / 丢弃计数
ros2 run tf2_ros tf2_echo map base_link         # 完整 TF 链
```

记录时必须能区分三件事：**导航请求的速度**、**合成后的执行命令**、**实际运动速度**（`odometry` twist）。
诊断消息里的关键字段：`nav_wz_dropped_count`（导航角速度被丢弃的次数）、`non_finite_*_count`、
`translation_scale`、`clamped`、`nav_timeout`、`rotation_timeout`、`target_wz`、`output_wz`、`waveform_time`。

不重启导航切换自转模式：

```bash
ros2 service call /<ns>/rotation_test_sender/disable std_srvs/srv/Trigger
ros2 param set /<ns>/rotation_test_sender rotation_mode periodic
ros2 param set /<ns>/rotation_test_sender offset 1.0
ros2 param set /<ns>/rotation_test_sender amplitude 0.5
ros2 param set /<ns>/rotation_test_sender period 4.0
ros2 service call /<ns>/rotation_test_sender/enable std_srvs/srv/Trigger
```

急停与恢复：

```bash
ros2 service call /<ns>/srm_cmd_mux/stop_all std_srvs/srv/Trigger
ros2 service call /<ns>/srm_cmd_mux/resume_all std_srvs/srv/Trigger
```

### 5.1 单元测试

```bash
colcon test --packages-select srm27_robot_description srm27_chassis_control
colcon test-result --verbose
```

- `srm27_robot_description`：5 个模型生成用例（模型格式、几何单一来源、旧步兵组件已删除、
  `base_link` 与雷达帧名稳定、速度插件配置正确）。
- `srm27_chassis_control`：两个 gtest（速度合成 `VelocityMix`、波形与超时 `RotationWaveform` /
  `TwistWatchdog`），另有 uncrustify / cppcheck / xmllint 检查。
- 注意：本机 python3 环境的 pytest11 插件（`launch_testing` 与较新 pluggy）不兼容，
  因此 `srm27_robot_description` 用 `ament_add_test` 直接执行 Python 测试脚本，而不是
  `ament_add_pytest_test`；测试文件本身仍保持 pytest 兼容。
- 也可以用 `./script/kill_gzb.sh` 清理残留 Gazebo / 桥接 / 适配器进程后再起下一轮。

---

## 6. 实测验证结论

以下为本机实测结果（空场 `world:=srm_empty`，headless）：

**仿真与几何**

- headless 启动后 `/clock` 正常。
- 订阅 `cmd_vel_sim` 发纯 `vx=+0.3`：实测真值里程计 `twist.linear.x=+0.300`，车体沿 +x 前进。
- 纯 `vy=+0.3`：实测 `twist.linear.y=+0.298`，沿 +y（左）平移且 x 不变。
- 纯 `wz=+1.0`：实测 `twist.angular.z=+0.990`（逆时针）；`wz=-1.0` 时为 `-0.991`。
- 命令停止后位置与姿态保持不变，说明执行侧超时清零生效。

**速度合成链路**（离线验证，`use_sim_time:=False`）

- `cmd_vel_nav=(0.4, 0.4, angular.z=9.0)` + `rotation_velocity wz=1.5`
  → `cmd_vel_sim=(0.3536, 0.3536, wz=1.5)`：平移按模长限幅，导航的 `angular.z` 被丢弃，
  自转只来自独立自转输入。
- `cmd_vel_sim` 实测约 195 Hz（配置 200 Hz）。
- 导航停止 0.3 s 后平移清零，而 `wz` 保持 1.5（两路超时互不影响）。
- `srm_cmd_mux` 的 `stop_all` 服务调用后输出全零，`resume_all` 后 `wz` 恢复。
- periodic 正弦模式（offset 1.0 / amplitude 0.5 / period 4 s）实测输出落在 0.5～1.5 rad/s。
- `ros2 param set` 设置负幅度与非正周期被拒绝，并给出中文原因。

**测试与静态检查**

- `srm27_robot_description` 5 个模型生成用例全部通过。
- `srm27_chassis_control` 两个 gtest（速度合成 / 波形与超时判定）全部通过；
  uncrustify、cppcheck、xmllint 也通过。
- 全工作空间 `colcon build`（43 个包）成功；`colcon test-result` 汇总 58 个用例 0 失败。

**导航端到端**（`world:=rmuc_2025` + `map:=rmuc_2025_tunnel.yaml`，headless，单套仿真栈）

- 生命周期节点 `controller_server` / `bt_navigator` / `velocity_smoother` 全部 `active`。
- TF 链完整：`map → odom → base_link → front_mid360` 均可查。
- `compute_path_to_pose` 到 `map(1.5, 1.5)` 返回 `SUCCEEDED`。
- `navigate_to_pose` 到 `map(1.5, 1.5)` 返回 **`SUCCEEDED`**；真值里程计最终停在
  `(1.414, 1.400)`，与目标相差约 0.13 m（`xy_goal_tolerance` 为 0.15 m），
  到达后 `cmd_vel_sim` 全零、位置不再变化。
- 到点后把自转发送器切到 `constant, angular_speed=1.0`：`rotation_velocity=1.0`，
  `cmd_vel_sim=(0, 0, wz=1.0)`，里程计姿态随之变化 —— 说明"导航平移 + 独立自转"的
  合成在整条链路上成立。

指标：每次运行至少记录 `/clock`、`cmd_vel_nav`、`rotation_cmd`、`rotation_velocity`、
`cmd_vel_sim`、`odometry`、TF、原始点云、IMU、`terrain_map`、`behavior_tree_log` 与
`diagnostics`；诊断与超时状态在 `/<ns>/diagnostics`。

---

## 7. 已知边界与注意事项

1. **只支持单机器人**：命名空间、Gazebo 模型名、TF 与话题都按单机约定；多机需要重新设计命名与 TF 划分。
2. **行为树与手柄接入未纳入本次改动**：导航行为树沿用现有 XML，恢复行为中的角速度分量由 mux 丢弃；
   手柄自转只是 `--teleop` 标签页的可选输出（remap 到 `rotation_cmd`），默认关闭。
3. **速度级执行不做轮毂动力学**：插件直接设置 Link 平面速度，没有轮速 PID、力矩控制或打滑建模，
   因此**不能用它评估真实的底盘动力学性能**，只能观察“导航平移叠加自转”的运动学效果。
4. **轮组接触与摩擦仍需实测**：轮子摩擦系数、碰撞几何与物理步长会影响卡墙、擦墙、过薄墙与隧道场景；
   空场验证通过不等于场地内一定通过，进入场地后需单独观察。
5. **仿真与地图必须对齐**：世界名、SRM 初始位姿、栅格地图和 odom 原点都来自
   `srm27_gazebo_simulator/config/srm_sim.yaml` 与地图 YAML，launch 里不再重复写偏移；
   换地图时要注意机器人出生点与地图原点是否一致。
6. **`map → odom` 同一时刻只能有一个发布者**：`slam`、`use_pcd_localization`、`use_lio_odometry`
   与默认静态 TF 互斥；仿真真值里程计只在三者都不启用时发布 `odom → base_link`。
7. **真值里程计与定位里程计不要同时用**：启用 Point-LIO / GICP / SLAM 后，仿真真值节点
   （`simulation_ground_truth_odometry`，含它发布的 `lidar_odometry`）不会启动，
   `odom → base_link` 由对应定位链路提供；需要对照实际运动时应改用定位链路的 `odometry` 与
   `registered_scan`，不要再去找真值话题。
8. **旧入口仍可运行但会打印弃用提示**：`nav_simulation_launch.py` 是转发壳；
   `rmu_gazebo_simulator bringup_sim.launch.py` 已下线，调用会直接抛错。
   新旧底盘执行器互斥，不要同时启动两套仿真。
9. **参数以 YAML 为准**：`config/srm_chassis_control.yaml`、`config/srm_sim.yaml`、
   `config/srm27_sentry_geometry.yaml` 分别是底盘控制、仿真入口与几何的权威来源；
   代码内默认值与 YAML 一致，仅用于未加载参数文件时的兜底。
10. **目标点必须落在栅格地图覆盖且可通行的区域内**：地图 `origin` 为 `[-3.58, -9.44]`，
    map 原点对应机器人出生点，地图覆盖 x∈[-3.58, 25.57]、y∈[-9.44, 6.76]。下发到地图
    覆盖范围之外（例如 `map(5.4, 9.5)`）的目标时，规划与跟踪会退化，`navigate_to_pose`
    最终以 `ABORTED` 结束。**在 `map(1.5, 1.5)` 这类地图内的目标上实测为 `SUCCEEDED`**
    （见 §6），因此 `ABORTED` 不是速度链路问题，先检查目标点与地图是否匹配。
11. **不要在有多套残留栈时判断导航效果**：实测中出现过导航反复 `ABORTED`、机器人停在
    同一位置的假象，原因是有 5～6 套残留的仿真/Nav2 进程同时在跑（重复的
    `static_transform_publisher`、真值里程计与代价地图互相干扰）。清理干净后同一条命令
    立即 `SUCCEEDED`。调试导航前务必先做残留检查。

> [!WARNING]
> **仿真调试时不要同时运行多套仿真栈。** Gazebo Fortress 会加载 ogre2 渲染与
> 1875 线 GPU 雷达，单个 `gzserver` 加上完整 Nav2 代价地图已经占用数 GB 内存；
> 本机（15 GB 内存 / 2 GB swap）在同时存在多套残留栈时发生过整机卡死。
> 每次启动前先确认没有残留进程：
>
> ```bash
> ./script/clean_sim_processes.sh          # 一键清理本工作空间的仿真残留进程
> ./script/kill_gzb.sh && ./script/kill_rviz.sh
> ```
>
> `script/start_sim_nav.sh` 自带 `flock` 单实例锁，正常使用不会叠起两套；用测试脚本
> 手工拉起时，必须确保脚本退出时把**整个进程组**（含 `gzserver`、bridge、适配器、
> Nav2 节点）一并结束，只 `kill` 外层 `ros2 launch` 不足以回收子进程。
> 可直接复用 `script/diag_nav_abort.sh`：它在启动前检查残留、用独立进程组启动、
> 带内存看门狗，并把日志落到 `log_diag/`。

---

相关文档：[仿真数据流](仿真数据流%28ai%29.md)、[SRM 仿真与导航自转控制实施方案](SRM仿真与导航自转控制实施方案.md)、
[哨兵导航 TF 与下位机数据接口](哨兵导航TF与下位机数据接口%28ai%29.md)、[MID-360 使用指南](mid360使用指南.md)。
