# 仿真链迁移实车实施记录

> 本文由 AI 生成，记录 2026-11-06 把仿真已验证的导航链（`scripts/bringup/start_sim_nav.sh
> --chassis-spin`：MINCO + fakeyaw + `srm_cmd_mux` + 小陀螺 + ZLIO）迁移到**实车**的
> 改动、设计取舍、上电验收步骤与已知缺口。方案层面的背景见
> [`wz控制迁移导航与fakeyaw仿真实施方案.md`](../design/wz控制迁移导航与fakeyaw仿真实施方案.md)，
> 仿真侧的落地细节见
> [`wz控制迁移与fakeyaw实施记录(ai).md`](wz控制迁移与fakeyaw实施记录%28ai%29.md) 与
> [`仿真LIO前端替换Z-LIO实施记录(ai).md`](仿真LIO前端替换Z-LIO实施记录%28ai%29.md)。

---

## 1. 结论速览

| 项 | 迁移前（实车） | 迁移后（实车默认） |
| --- | --- | --- |
| 控制器 | MINCO，`yaw_policy.mode=xy_only` | MINCO，`yaw_policy.mode=se2`（`--yaw-mode spin` 可切小陀螺） |
| 导航参考系 | `base_link`（随底盘自转） | **`base_link_fake`**（与 odom 平行、朝向稳定） |
| 速度命令系 | `base_link`（控制器做一次 `R(-ψ)`） | `base_link_fake`（`fake_vel_transform` 做唯一一次 `R(-ψ)`） |
| 速度出口 | `velocity_smoother → cmd_vel_chassis` | `velocity_smoother → cmd_vel_nav_fake → fake_vel_transform → cmd_vel_nav_body → srm_cmd_mux → cmd_vel_chassis` |
| 到点判定 | `OmniStoppedGoalChecker`（不看 wz） | `StoppedGoalChecker` + `rot_stopped_velocity`（看 wz） |
| 小陀螺 | 无 | `rotation_cmd → MINCO.spin_request → 导航输出 wz`（`--chassis-spin`） |
| 里程计前端 | Point-LIO | Point-LIO（默认）/ **ZLIO**（`--lio-frontend zlio`，实车参数） |
| 速度限幅来源 | `config/real/speed_limits.yaml` | 同一份，新增 `@fakeyaw` 段与 `srm_cmd_mux` 段 |
| 云台自转 | 无 | **仍未支持**（下位机驱动，协议不回传关节角，见 §5） |

一句话：**上位机侧已经与仿真完全同构，唯一的执行端差异是最后一段话题名
（`cmd_vel_sim` → `cmd_vel_chassis`）**；剩下两个"物理侧"缺口（wz 标定、云台关节角回传）
不是上位机代码能补的，本文给出接口与验收方法。

---

## 2. 迁移前后的链路对照

```text
仿真（start_sim_nav.sh --chassis-spin）
  MINCO/Nav2(base_link_fake) ─cmd_vel_nav_fake─► fake_vel_transform(nav_se2)
      ─cmd_vel_nav_body─► srm_cmd_mux ─cmd_vel_sim─► srm_velocity_adapter ─► Gazebo

实车（start_real_nav.sh，迁移后默认）
  MINCO/Nav2(base_link_fake) ─cmd_vel_nav_fake─► fake_vel_transform(nav_se2)
      ─cmd_vel_nav_body─► srm_cmd_mux ─cmd_vel_chassis─► srm27_nav_protocol ─► 串口 ─► C 板
```

**只有最后一段不同**，这也正是 `srm_chassis_control.launch.py` 新增 `output_topic`
参数、并把"实车出口必须是 `cmd_vel_chassis`"写成启动校验的原因：写错时链路全绿、车不动。

---

## 3. 改动清单

### 3.1 新增文件

| 文件 | 作用 |
| --- | --- |
| `config/real/nav2_params_srm_minco_fakeyaw.yaml` | 实车迁移链参数：`base_link_fake` 参考系、`command_frame`、`yaw_policy.mode=se2`、`spin_request`、`StoppedGoalChecker`、`fake_vel_transform(nav_se2)`、`zlio_bridge`、`imu_saturation_guard`。其余全部沿用实车 `nav2_params_srm_minco.yaml` 的实测值（**不从仿真抄**） |
| `launch/nav_srm_real_launch.py` | 实车入口（与 `nav_srm_simulation_launch.py` 同构）：注入 `yaw_mode`/判停器、起 `srm_cmd_mux`、起云台关节角中继（可选）、启动时核对并打印整条速度链 |
| `config/real/zlio_mid360.yaml` | ZLIO 实车算法配置：与仿真那份**同一套键**，只改 3 处（`body_from_lidar.translation`、`window_frames=10`、`window.maximum_interval=0.25`） |
| `scripts/gimbal_joint_state_relay.py` | 云台关节角 → `/joint_states` 中继（协议补齐回传字段后即可接进 TF） |
| `test/test_fakeyaw_real_wiring.py` | 实车入口接线回归（7 个用例） |

### 3.2 修改文件

| 文件 | 改动 |
| --- | --- |
| `scripts/srm_speed_limits.py` | 新增 `@fakeyaw` **附加 profile**（可与 `@minco` 叠加，后写优先）；按参数内容判定，不看文件名；`chain_summary` 只对迁移链把 mux 算作一层 |
| `config/real/speed_limits.yaml` | 新增 `velocity_smoother@fakeyaw`（放开 yaw：1.5 rad/s / 4.5 rad/s²）与 `srm_cmd_mux` 段（v 4.5、wz 1.5） |
| `src/srm27_chassis_control/launch/srm_chassis_control.launch.py` | 新增 `output_topic`（仿真默认 `cmd_vel_sim`，实车传 `cmd_vel_chassis`） |
| `src/srm27_chassis_control/config/srm_chassis_control.yaml` | 注释更新：mux 不再只存在于仿真链 |
| `launch/nav2_stack_launch.py` | 透传 `zlio_config` 与 ZLIO 输入接线（`zlio_input_type/lidar_topic/imu_topic/imu_acceleration_scale/body_frame`） |
| `launch/localization_launch.py` | ZLIO 的输入接线由写死改为 launch 参数（仿真/实车不同） |
| `scripts/srm_minco_real_preflight.py` | 识别 fakeyaw profile；判停器按链期望相反判定；新增 `check_fakeyaw_chain`（参考系统一、`nav_se2`、话题自环、`odom_timeout`、`spin_request`、yaw 外推）；限速层新增 mux 校验 |
| `scripts/manual/spin_control.sh` | 支持根命名空间（`ROBOT_NS=""`，实车默认）；status 的执行端话题自动识别 `cmd_vel_sim`/`cmd_vel_chassis` |
| `scripts/bringup/start_real_nav.sh` | MINCO 默认改走迁移链入口；新增 `--yaw-mode` / `--chassis-spin` / `--lio-frontend` / `--zlio-config` / `--mux` / `--gimbal-joint-topic` / `--speed-limits`；**参数面与 `start_sim_nav.sh` 对齐**（见 §4.6）；Nav2 就绪后自动下发小陀螺；摘要与自检按链打印 |
| `scripts/bringup/kill_nav.sh` | 入口匹配加入 `nav_srm_real_launch`；孤儿兜底加入 `cmd_mux` / `fake_vel_transform_node` / `zlio*` / `imu_saturation_guard` / `gimbal_joint_state_relay` |
| `test/test_speed_limits.py` / `test/test_lio_frontend_wiring.py` | 覆盖 `@fakeyaw` 叠加与 profile 判定；ZLIO 参数改为求值替换后断言 |
| `test/test_cli_parity.py` | 守住"仿真脚本的每个参数实车都能解析"（含 `--gicp`/`--init-pose` 的语义映射与实车专有参数不被误删） |

### 3.3 下位机反馈（0x0010 底盘速度包，2026-11-06）

> **后续变更（2026-11-08）**：`0x0010` 第 4 个字段改为 `chassis_to_gimbal_yaw`
> （底盘相对云台，度），话题 `chassis_yaw_deg` 改名为 `chassis_to_gimbal_yaw_deg`，
> 并新增 `/gimbal_joint_angle`（云台相对底盘，弧度）接进 `gimbal_yaw_joint`。
> 本节表格是当时的记录，以 [导航通信协议 §4.2/§4.4](../architecture/导航通信协议-下位机接口说明.md)
> 与 [哨兵导航TF与下位机数据接口 §5.4](../architecture/哨兵导航TF与下位机数据接口%28ai%29.md) 为准。

| 文件 | 改动 |
| --- | --- |
| `srm27_nav_protocol/include/packet_typedef.hpp` | `ID_CHASSIS_SPEED = 0x0010`、`ReceiveChassisSpeedData`（25 B，`static_assert` 钉死帧长与数据段偏移） |
| `srm27_nav_protocol/src/srm27_nav_protocol.cpp` | 按 `cmd_id` 分派新增 `0x0010`；长度自检、`NaN/Inf` 丢弃、失联/恢复告警；发布 `/chassis_speed`（`TwistStamped`）与 `/chassis_yaw_deg`（`Float32`，度） |
| `srm27_nav_protocol/config/srm27_nav_protocol.yaml` | 新增 `publish_chassis_speed` / `chassis_speed_topic` / `chassis_yaw_topic` / `chassis_speed_frame` / `chassis_speed_timeout_sec` |
| `scripts/test/test_chassis_speed_feedback.py` | 反馈帧工具：`--self-test`（CRC/帧长自检）、`--device`（只读监听统计）、`--pty`（无硬件时造伪串口喂帧，端到端验证 ROS 节点） |
| `srm27_nav_protocol/test/test_feedback_protocol.py` | 协议级回归：CRC 参考向量、CRC16 覆盖 `cmd_id`、轮转 + 槽位跳过、坏帧恢复、断帧、长度自检、`seq` 回绕 |
| `docs/导航通信协议-下位机接口说明.md` | §4.2 补 `0x0010`；新增 §4.3（10 ms 槽位轮转 / 必须按 `cmd_id` 分派）与 §4.4（上位机实现状态与话题）；示例帧与速查表同步 |
| `CMakeLists.txt` | 安装 `gimbal_joint_state_relay.py`；注册 `test_fakeyaw_real_wiring` |

---

## 4. 关键设计决定

### 4.1 为什么 mux 也要搬到实车

仿真侧引入 `srm_cmd_mux` 是为了"`cmd_vel_sim` 的唯一发布者 + 单点钳位 + 超时清零"。
实车上它的价值反而更大：

* `cmd_vel_chassis` 只有一个发布者，"谁在给速度"不再需要靠 `ros2 topic info` 现场推断；
* `wz_max` 这一层钳位**独立于**控制器与串口，是首跑验证 wz 时的第一道闸；
* `stop_all` / `resume_all` 是上位机侧唯一能"清零并保持"的软件手段
  （串口节点没有急停服务，见 `scripts/bringup/start_real_nav.sh` 顶部）。

代价是多一个 200 Hz 节点与 0.3 s 导航超时；两者都在仿真实测过，风险可接受。

### 4.2 速度限幅为什么要"叠加"而不是新变体

`@minco` / `@omni` 是按 `FollowPath.plugin` 判定的**控制器**变体，而迁移链与实车旧链
用的是同一个 MINCO 控制器 —— 变体相同，但 `velocity_smoother` 的 yaw 界要求**相反**
（旧链必须钳 0；迁移链必须放开）。把它做成新的控制器变体是错的（会连带改控制器段），
所以做成**叠加 profile**：

```text
生效顺序（后者覆盖前者，与文件书写顺序无关）：
  共用段（无 @） → @minco/@omni → @fakeyaw
```

profile 判定**看内容不看文件名**（`FollowPath.fake_base_frame` 或
`fake_vel_transform.transform_mode == nav_se2`），因此 `-p` 指过来的临时参数副本
（launch 会先重写成 `/tmp/*.yaml`）也能正确命中。

### 4.3 `odom_timeout` 与实车里程计频率

实车 `/odometry` 由 `sensor_scan_generation` 发布，频率**跟随雷达帧率**（10 Hz，周期 0.1 s），
不是仿真的 50 Hz 真值。因此 `fake_vel_transform.odom_timeout` 取 **0.60 s**（6 帧余量，
与 `sensor_scan_generation.max_sample_gap` 同档）；`FollowPath.state_timeout` 保持实车值
**0.25 s**（2.5 帧），不照抄仿真的 0.5 s（那是给仿真 50 Hz odom 的）。

### 4.4 ZLIO 的实车参数与仿真差异

| 项 | 仿真 | 实车 | 写错的表现 |
| --- | --- | --- | --- |
| IMU 加速度单位 | m/s²（`imu_acceleration_scale: 1.0`） | **g**（`9.81`） | 放大 9.81 倍 → 滤波器当场发散；或压小 → 加速度几乎不起作用 |
| 点云格式 | `pointcloud2` + `velodyne_points` | **`livox`（CustomMsg）+ `livox/lidar`** | 收不到数据/解码错 |
| 里程计 child frame | `front_mid360` | **`livox_imu`** | `zlio_bridge` 查 TF 失败 → 不发布 `lidar_odometry` |
| 雷达→IMU 平移 | `[0, 0, 0.03]` | **`[-0.011, -0.02329, 0.04412]`**（= `point_lio.extrinsic_T`） | 点云整体偏 3~4 cm |
| 初始化窗口/观测间隔 | 20 帧 / 0.12 s（20 Hz） | **10 帧 / 0.25 s**（10 Hz） | 每帧都被判"观测间隔过长"，反复初始化 |

⚠ `zlio_mid360.yaml` 受 z-loc 的**严格校验**约束：未知键会直接抛异常，所以实车那份
**键集与仿真完全一致**（只有值不同），已用脚本核对过。

### 4.5 yaw 外推默认关闭

`fake_vel_transform` 的 yaw 外推（`chassis_cmd_vel_topic` + `actuation_delay`）在仿真里
用 mux 输出的 wz 序列补"指令生效时刻"的航向锯齿。实车串口链路的下发延迟**还没标定**，
所以实车参数里 `chassis_cmd_vel_topic: ""` = 不订阅、不外推（行为与迁移前一致）。
标定方法见 §6 S6。

---

### 4.6 命令行参数与仿真对齐

同一条命令在仿真与实车都能跑（`-w/--world`、`--gui`、`--run` 这类 Gazebo 专用参数实车**接受并忽略**）：

| 参数 | 仿真语义 | 实车语义 |
| --- | --- | --- |
| `-m/--map`、`--prior-pcd`、`-p/--params`、`--rviz`、`--smoother`、`--rviz-config` | 同名同义 | 同名同义 |
| `-c/--controller` | `auto|minco` | `auto|minco|omni`（`omni` 是实车旧链） |
| `--gicp` / `--no-gicp` | 先验 PCD 重定位 / 关掉（静态图 + 真值） | `= --reloc` / `= --no-reloc`（LIO + 静态图） |
| `--lio` / `--no-lio` | LIO 出里程计 / 用 Gazebo 真值 | LIO + 静态 `map->odom` / **无里程计来源**（按 `--static` 处理并告警） |
| `--lio-frontend`、`--zlio-config` | point_lio / zlio | 同名同义（实车 ZLIO 参数在 `config/real`） |
| `--init-pose "<x,y,z,roll,pitch,yaw>"` | `map->odom` 初值 | 只取 `x/y/yaw`（实车是二维静态 TF），`z/roll/pitch` 告警忽略 |
| `--yaw-mode` | `auto|se2|spin` | 同左，另加 `xy_only`（A/B 对照） |
| `--chassis-spin [wz]` / `--no-chassis-spin` | 底盘小陀螺 | 同名同义（首跑建议 0.3，mux 钳到 1.5） |
| `--gimbal-spin [wz]` / `--no-gimbal-spin` | Gazebo 插件驱动云台 | **接受但不执行**（下位机驱动、无指令通道），明确告警 |
| `--verify-spin` | 无头自主验证 | **接受但拒绝执行**（会自主驱动车辆），打印人工 S1~S4 步骤 |
| `-w/--world`、`--gui`、`--run`、`--prior-pcd-manifest` | 仿真专用 | 接受并给一行"实车忽略"提示 |
| `--mux`、`--gimbal-joint-topic`、`--speed-limits`、`--map-to-odom`、`--slam/--reloc/--static`、`--lidar-*`、`--joy`、`--stop`、`--list-maps` | — | 实车专有 |

环境变量同样有别名：`USE_GICP`、`USE_LIO_ODOMETRY`、`ROBOT_NS`、`INIT_POSE`、`YAW_MODE`、
`CHASSIS_SPIN_WZ`、`GIMBAL_SPIN_WZ`、`CHASSIS_SPIN_READY_TIMEOUT`、`LIO_FRONTEND` 等在两份脚本里
含义一致。`test/test_cli_parity.py` 会在 CI 里守住这条契约（仿真脚本的每个 case 分支实车都必须有）。

## 5. 两个执行端缺口

### 5.1 wz 由 C 板执行（已支持，但必须标定）

《SRM 校内赛导航通信协议》§3.3 早期明确写着 **`wz` 被忽略**。经确认，C 板固件**已改**，
现在执行 wz。但"执行"不等于"执行得对"：符号、增益、死区、加减速能力都只有转起来才知道。

因此迁移后：

* `config/real/speed_limits.yaml` 里 `srm_cmd_mux.wz_max = 1.5`、`velocity_smoother@fakeyaw`
  的 yaw 界也是 1.5 rad/s —— **首跑刻意压低**，验过再抬；
* `--chassis-spin` 默认请求 2.0 rad/s，会被上面两层钳到 1.5，属刻意行为；
* 首跑请**架起车辆**（轮子离地）或用 `--chassis-spin 0.3` 这种小转速先确认方向。

**反馈通道（2026-11-06 新增）**：下位机 `0x0010` 速度包现在被解析成
`/chassis_speed`（`TwistStamped`：`linear.x=vx`、`linear.y=vy`、`angular.z=wz`）与
`/chassis_to_gimbal_yaw_deg`（`Float32`，度；2026-11-08 前叫 `/chassis_yaw_deg`，
且当时是"底盘 yaw"口径，**新口径是底盘相对云台，不能再拿 wz×t 去对**）。
标定时**不需要再靠人眼看车**：

```bash
ros2 topic echo /cmd_vel_chassis   # 指令 wz（mux 钳位后）
ros2 topic echo /chassis_speed     # 实际 vx/vy/wz（底盘速度观测器的融合结果）
ros2 topic echo /chassis_to_gimbal_yaw_deg   # 底盘相对云台（度）；云台不动时底盘转过多少它变多少
ros2 topic echo /gimbal_joint_angle          # 换算后的 gimbal_yaw_joint（rad，取反并 wrap）
```

口径提醒：`0x0010` 填的是**融合结果** `speed->fused`（底盘系），不是轮速正解，
也不是"指令已执行"的回显；指令→观测有延迟与尺度差，对账时要一起算。

### 5.2 云台关节角（2026-11-08 已补齐回传）

> **更新**：本节下面记的是 2026-11-06 的"缺口"状态。2026-11-08 起下位机 `0x0010`
> 的第 4 个 float32 就是 `chassis_to_gimbal_yaw`（底盘相对云台，度），上位机换算成
> `/gimbal_joint_angle`（云台相对底盘，弧度）接进 `gimbal_yaw_joint`，
> `start_real_nav.sh` 默认就带 `--gimbal-joint-topic gimbal_joint_angle`。
> 当前口径、符号与风险以 [导航通信协议 §4.4](../architecture/导航通信协议-下位机接口说明.md) 和
> [哨兵导航TF与下位机数据接口 §5.4](../architecture/哨兵导航TF与下位机数据接口%28ai%29.md) 为准。

实车云台由下位机驱动自转，雷达装在云台上，于是

```text
base_link yaw = 云台世界 yaw − 云台关节角
```

`robot_state_publisher` 用 `gimbal_joint_type:=revolute` 时，`base_link -> gimbal_yaw`
这条 TF 角**只来自 `/joint_states`**。当时协议上→下只有 12 字节 `vx/vy/wz`、
下→上只有比赛状态类数据，**没有关节角字段**，所以当时的实车没有任何节点发布 `/joint_states`：

* 云台不转 → 关节角恒 0，`revolute` 与 `fixed` 等价，导航正常；
* 云台在转 → `sensor_scan_generation` 会把云台转动当成底盘转动，`base_link` yaw 持续漂，
  导航"看到车自己在转"，**而且不会报任何错**。

**上位机侧已经就绪的部分**：`scripts/gimbal_joint_state_relay.py` 把
"下位机回传的关节角"转成 `/joint_states`（关节名 `gimbal_yaw_joint`，与 URDF 一致），
输入支持 `std_msgs/Float32`（`float_msg_type` 可选 `example_interfaces`）与 `JointState`，
入口参数 `gimbal_joint_topic`（`start_real_nav.sh --gimbal-joint-topic <话题>`）。

**需要下位机/协议侧补的部分**（建议字段，沿用
[`哨兵导航TF与下位机数据接口(ai).md`](../architecture/哨兵导航TF与下位机数据接口%28ai%29.md) §5）：

| 字段 | 类型 | 含义 | 备注 |
| --- | --- | --- | --- |
| `yaw_joint_angle` | float32 / int16×k | 云台相对底盘的**实际关节角**（rad） | 必须带采样时间戳；是相对角，不是云台世界 yaw |
| `yaw_joint_velocity` | float32 | 关节角速度（rad/s） | 选填，用于插值与短时预测 |
| `timestamp` | uint32/uint64 | 实际采样时刻 | 需能映射到 ROS 时间 |
| `base_wz` | float32 | 底盘陀螺实测 wz | 强烈建议；用于闭环核对"指令 vs 实际" |

实测验收（补完之后）：

1. `ros2 topic hz /joint_states` → 应 ≥ 100 Hz；
2. 底盘静止、只转云台：`ros2 run tf2_ros tf2_echo odom base_link` 的 yaw **应基本不动**
   （这是"关节角接对了"的判据）；
3. 云台固定、只让底盘自转：`base_link` yaw 应真实变化；
4. 两者同时转：`scripts/diagnostics/check_yaw_sources.py` 的"两源转角差"仍在容差内。

**协议侧已经补齐**（2026-11-08）：`0x0010` 第 4 个 float32 是 `chassis_to_gimbal_yaw`
（底盘相对云台、度、(-180,180]），上位机取反换成弧度后发 `/gimbal_joint_angle`，
`gimbal_joint_sign` 可现场翻转符号。**仍然要防超时**：数据断了 relay 会在
`timeout_sec`（默认 0.3 s）后停止发布 `/joint_states`，TF 停在最后一帧——这是刻意的
（不发 0 冒充回零），但是导航不会因此自己停，第一次上车要盯这条告警。

---

## 6. 上电验收步骤

> 每一步都给了"通过判据"。任一步不通过就停在那一步排查，不要往下走。

### S0 构建与静态检查

```bash
cd ~/srm_nav_27
colcon build --symlink-install --packages-select \
  srm27_nav_bringup srm27_chassis_control srm27_nav_plugins
source install/setup.bash

# 链路口径只解析、不启动
./scripts/bringup/start_real_nav.sh -n --no-reloc

# 限速分层自检（哪一层卡住速度）
python3 src/srm27_navigation/srm27_nav_bringup/scripts/srm_speed_limits_check.py

# 实车参数预检（只读，不需要 ROS）
python3 src/srm27_navigation/srm27_nav_bringup/scripts/srm_minco_real_preflight.py \
  --params src/srm27_navigation/srm27_nav_bringup/config/real/nav2_params_srm_minco_fakeyaw.yaml \
  --expect-minco
```

判据：`-n` 输出的速度链路是
`… → cmd_vel_nav_fake → fake_vel_transform → cmd_vel_nav_body → srm_cmd_mux → cmd_vel_chassis → srm27_nav_protocol → 串口`；
预检 **0 个阻塞问题**（警告里 `braking_deceleration`、`command_lookahead` 等待辨识项按清单记录）。

### S1 不含 wz 的链路（`xy_only` 对照）

```bash
./scripts/bringup/start_real_nav.sh --yaw-mode xy_only --no-joy
```

判据：

* `ros2 node list` 有 `srm_cmd_mux` 与 `fake_vel_transform`；
* `ros2 topic info /cmd_vel_chassis -v` **发布者只有 `srm_cmd_mux`**；
* `ros2 run tf2_ros tf2_echo base_link base_link_fake` 的 yaw ≈ `-底盘 yaw`（静止时 ≈ 0）；
* RViz 里点一个近处目标，车按预期方向移动，`wz` 恒 0。

这一步验证的是**平移链路**（fake 只转平移、mux 出口正确），与 wz 无关，风险最低。

### S2 wz 方向与增益（架车！）

```bash
# 轮子离地，或至少留人守物理急停
./scripts/bringup/start_real_nav.sh --yaw-mode spin --chassis-spin 0.3
```

判据：

```bash
ros2 topic echo /cmd_vel_nav_fake --once     # wz ≈ +0.3
ros2 topic echo /cmd_vel_nav_body --once     # wz 与上面**逐条相同**（fake 不动 wz）
ros2 topic echo /cmd_vel_chassis --once      # wz 相同，且不超过 mux.wz_max=1.5
```

* **方向**：`+wz` 应为**逆时针**（俯视，右手法则）。反了就在
  `srm27_nav_protocol` 或下位机侧改符号 —— **不要**在 `speed_limits` 里绕。
* **增益**：请求 0.3 rad/s 时，30 s 内底盘应转过约 8.6 rad（≈1.37 圈）。明显偏快/偏慢
  说明下位机有换算，记为 `k = 实际/指令`，在参数清单里记录。

### S3 se2 首次闭环（场地空旷）

```bash
./scripts/bringup/start_real_nav.sh --no-joy
```

判据：`/FollowPath/diagnostics` 的 `planning_result=success`、`minimum_clearance>0`；
下发目标后车头逐渐对齐路径切线（`base_link` yaw 随路径变化，但**代价地图不转**）；
到点后 `/cmd_vel_chassis` 的 wz 归零（`rot_stopped_velocity=0.05` 生效）。

### S4 小陀螺导航（边转边走）

```bash
./scripts/bringup/start_real_nav.sh --chassis-spin 1.0     # 建议先 1.0，不要直接 2.0
```

判据：

* 有导航目标时 `/cmd_vel_chassis` 的 wz 恒为请求值（被钳到 ≤1.5），无目标时 **wz=0 且车不转**
  （迁移契约：控制器只在跟随路径时输出速度）；
* `ros2 run tf2_ros tf2_echo base_link base_link_fake` 的 yaw 持续变化而
  `map→base_link_fake` 稳定 —— 这是"自转被正确隔离在真实底盘系"的直接证据；
* 到点判定仍能通过（`spin` 模式已自动换成只看平移的 `OmniStoppedGoalChecker`）。

### S5 ZLIO A/B（可选）

```bash
./scripts/bringup/start_real_nav.sh --lio-frontend zlio --no-joy
```

判据：

* `ros2 node list` 有 `zlio`、`zlio_bridge`、`imu_saturation_guard`，**没有** `loam_interface`；
* 静止时 `ros2 topic echo /livox/imu --once` 的 `linear_acceleration` 模长 ≈ **1.0**（单位 g），
  若 ≈ 9.81 则说明驱动改成输出 m/s² 了，必须把 `zlio_imu_acceleration_scale` 改回 1.0；
* `ros2 topic hz /odometry` ≈ 雷达帧率，`zlio_bridge/events` 无持续 `frame jump` 告警；
* 与 Point-LIO 用同一段路线 A/B（`--lio-frontend point_lio`），比较
  `debug_tools/plot_lio_gt_yaw.py` 的轨迹一致性与 `minimum_clearance`。

### S6 yaw 外推标定（可选，默认关闭）

1. 打开订阅：把 `nav2_params_srm_minco_fakeyaw.yaml` 的
   `fake_vel_transform.chassis_cmd_vel_topic` 改成 `cmd_vel_chassis`；
2. 录包（自转 + 直行各 60 s）：`ros2 bag record /cmd_vel_chassis /odometry /tf`;
3. 互相关求"指令 wz 阶跃 → odom yaw 角速度开始变化"的时延中位数，作为 `actuation_delay`；
4. 复跑，看 `/fake_vel_transform/yaw_debug`（x=预测 yaw，y=odom yaw，z=积分量）与
   `diagnostics` 里的 `odom_age_sim`；残差明显变小再保留，否则把话题改回空串。

---

## 7. 回退

| 想回到… | 做法 |
| --- | --- |
| 实车旧链（导航不控自转，不经 mux） | `./scripts/bringup/start_real_nav.sh -p <abs>/config/real/nav2_params_srm_minco.yaml`（或 `-c omni` 回到 Omni） |
| 不串 mux（排查用，车可能不动） | `./scripts/bringup/start_real_nav.sh --no-mux` |
| 导航不控自转但保留新参考系 | `./scripts/bringup/start_real_nav.sh --yaw-mode xy_only` |
| 定位回 Point-LIO | 去掉 `--lio-frontend zlio`（默认就是 point_lio） |
| 解除 wz 首跑限速 | 改 `config/real/speed_limits.yaml` 的 `srm_cmd_mux.wz_max` 与 `velocity_smoother@fakeyaw` 的 yaw 值，**重启**导航栈（两者都没有动态回调） |

---

## 8. 本地验证证据（无硬件）

| 项 | 结果 |
| --- | --- |
| `nav_srm_real_launch.py --show-args` | 参数齐全；默认 `params_file` 指向迁移链、`mux_output_topic=cmd_vel_chassis` |
| 起整条入口（`autostart=false`、无硬件） | 启动日志打印迁移链；`srm_cmd_mux` 拿到 `wz_max=1.5 / v_max=4.5 / nav_topic=cmd_vel_nav_body / output_topic=cmd_vel_chassis`；`fake_vel_transform` 进入 `nav_se2`、`odom_timeout=0.6`、外推关闭 |
| 向 `/cmd_vel_nav_body` 发 `wz=3.0` | mux 日志：`导航 wz 被限幅到 1.500 rad/s`；停发 0.3 s 后：`超过 0.300 s 未收到导航速度，平移与 wz 均已清零` |
| 云台中继 | 发 `/gimbal_joint_angle=0.7`（Float32）→ `/joint_states` 输出 `name=[gimbal_yaw_joint], position=[0.7]` |
| 0x0010 反馈（pty 端到端） | 喂 `vx=0.4 / vy=-0.1 / wz=0.75 / chassis_to_gimbal_yaw=45°` → `/chassis_speed` 解出同值且 `frame_id=base_link`，`/chassis_to_gimbal_yaw_deg=45.0`，`/gimbal_joint_angle=-0.7854`（2026-11-08 复核）；`data_length=12` 的伪帧被丢弃并告警 |
| 0x0010 异常路径 | `NaN` 帧三条话题都不发（只计数）；停发 0.5 s 出现"中断超过"告警，恢复后再发出现"恢复"日志 |
| 协议回归 | `test_feedback_protocol` 8/8 通过（含"改 `cmd_id` 必须让 CRC16 失败"的反证、轮转中丢槽后仍逐帧正确） |
| `test_fakeyaw_real_wiring` | 7/7 通过（默认链路、话题对接、参考系统一、ZLIO 接线、yaw_mode 注入、mux 出口校验、旧链拒绝、中继常驻与输入类型切换） |
| `test_speed_limits` | 15/15 通过（含 `@fakeyaw` 叠加、profile 按内容判定、mux 只在迁移链算一层） |
| `test_fakeyaw_launch_wiring` / `test_lio_frontend_wiring` / `test_zlio_bridge_math` | 全部通过 |
| `colcon test`（bringup + chassis_control） | 0 failures |
| 预检（迁移链参数） | 0 阻塞 / 8 警告（均为待辨识项：制动、`command_lookahead`、外形包络等） |

**未验证**（需要实车）：wz 真实方向与增益、串口下发延迟、ZLIO 在实车 MID-360 上的收敛性、
云台关节角回传。

---

## 9. 遗留与后续

| 项 | 状态 |
| --- | --- |
| 云台关节角回传 | 协议侧补齐字段后，`--gimbal-joint-topic` 即可接入，上位机无需再改 |
| `--gicp` + ZLIO 组合 | 仍未接入（`nav2_stack_launch` 会拒绝 `zlio + 非 lio`） |
| `--slam` + 迁移链 | 建图链路不经过 `localization_launch`，不启动 zlio；建图请用 point_lio |
| yaw 外推 `actuation_delay` | 未标定，实车默认关闭 |
| `srm_cmd_mux` 的 `stop_all` | 只是上位机侧软件闸，**不是急停**；物理急停永远是第一手段 |
| 制动/延迟辨识 | `safety.braking_deceleration`、`mpc.command_lookahead` 仍是占位值，见 `docs/minco实车迁移实施记录(ai).md` 的待辨识清单 |

---

## 10. 2026-11-13：雷达安装角 roll 符号写反（zlio 点云整片歪 60° 的真因）

### 10.1 现象

同一车位先后起 `--lio-frontend zlio` 与 `point_lio`（其余参数相同、`--no-reloc`、
`map->odom` 静态 0）。RViz 同一份配置、同一 Fixed Frame=`map`：

| | zlio（错） | point_lio（对） |
| --- | --- | --- |
| `terrain_map_ext` | 整片"立起来"、散布 135 px、点数 ≈2× | 贴地薄带、散布 64 px |
| `sensor_scan` | 远端一大片"格子"铺到画面上缘 | 只在带子上方一小片 |
| 结论 | 看图像是点云被整体绕水平轴转了 | 正常 |

### 10.2 定位（新增 `scripts/diagnostics/check_lio_mount_consistency.py`）

拿**静止时 IMU 的比力**当唯一物理基准（比力方向 = "世界上方向在雷达系里的表达"），
三边对账（旧模型、point_lio 在跑时的实测输出）：

```
[IMU  ] |a|=0.9927, 比力=(+0.0278, -0.5005, +0.8653)  => 实测 roll = -30.04°
[TF   ] gimbal_yaw ← livox_imu  rpy=(30.00, 0.00, 63.00)°
[LIO  ] /lidar_odometry         rpy=(-29.78, -1.67, 63.96)°
[CLOUD] 地面法向与 +z 夹角 1.64°
TF ↔ IMU : 60.06°  **不一致**
LIO ↔ IMU: 0.28°   OK
```

即：**URDF（和 nav2 里的 gravity）写的是 +30°，实车真实安装是 -30°**，差 60°；
而 Point-LIO 发出来的雷达姿态反而是对的（-29.78°）。两种前端对同一个模型误差的
反应完全不同：

* `point_lio`：`rot_init` 用 `mapping.gravity` 把世界系扶正，`R_0 = Rx(-60°)` 把模型
  误差吸收进自己的世界系，再经 `loam_interface` 的 `T_base←lidar` 左乘，点云落地正好
  等于**真实**安装外参 → 云是对的。代价是 `/odometry` 被整体左乘一个常数
  `B = M·R_0·M⁻¹`：实测 `map->gimbal_yaw` 在出生点就是 **yaw 17.82° / (0.067,-0.028) m**
  （云台关节角仅 -2.2°，车没动），与 60° 模型误差的解析值 18.5° 吻合。
* `zlio`：`zlio_bridge` 的 `T_off = TF(gimbal_yaw←livox_imu)·R_0⁻¹` 把首帧位姿显式
  抵消掉，模型误差**没有任何环节吸收** → 整片 `registered_scan`/`terrain_map_ext`
  就是 `M_wrong`（roll +30°）而不是真实的 roll -30°，歪 60°。

所以"第一张（zlio）不对、第二张（point_lio）对"不是前端算法好坏，而是**模型错了**：
point_lio 把它藏起来了，zlio 把它露出来了。

### 10.3 修复

| 文件 | 改动 |
| --- | --- |
| `src/srm27_robot_description/urdf/srm27_sentry.urdf.xacro` | `lidar_roll` 默认 `0.523598776` → **`-0.523598776`**；注释写清实测依据 |
| `config/real/nav2_params_srm_minco_fakeyaw.yaml`（默认链）<br>`config/real/nav2_params_srm_minco.yaml`（minco 旧链）<br>`config/real/nav2_params_srm.yaml`（omni 旧链） | `point_lio.mapping.gravity` / `gravity_init` 的 y 分量 `-4.905…` → **`+4.905…`** |
| `config/real/zlio_mid360.yaml`、`scripts/mapping/pcd_to_mapframe.py` | 注释/平面提示同步（ZLIO 这份只写模长，符号错了它自己发现不了） |
| `config/srm27_sentry_geometry.yaml` | **不改**（Gazebo 世界按 +30° 自洽建的），只加注释说明"实车 -30° / 仿真 +30° 是两个数，别混" |
| 新增 `test/test_point_lio_gravity_matches_model.py` | xacro 的 `lidar_roll/pitch` 与三份实车 params 的 gravity 必须由同一公式算出（<1e-6），任一边改单边就红 |
| 新增 `scripts/diagnostics/check_lio_mount_consistency.py` | 实车只读对账：TF / `/lidar_odometry` / `/sensor_scan` 三边 vs IMU 实测，超 2° 退出码 1 |

改完 `install/` 不用重编（`urdf`、`config/real` 都是软链），但**必须重启**才生效
（`robot_description` 是启动时展开的，point_lio 的 gravity 也是启动时读的）。

### 10.4 重启后应看到

1. `python3 scripts/diagnostics/check_lio_mount_consistency.py` 三行全 OK（TF ↔ IMU 应为 ~0°，
   `/lidar_odometry` 的 roll 仍是 -30° 左右 —— 那是真实安装姿态，不是错误）。
2. `tf2_echo map livox_imu` 的 roll 从 `+30°` 变成 `-30°`。
3. **点云不变**：`M·R_0` 前后恒等于 `Rz(63°)Rx(-30°)`，所以 point_lio 的
   `registered_scan`/`terrain_map_ext` 与地图的对齐关系不变。
4. **`/odometry` 变了**：出生点从 `yaw≈17.8° / (0.067,-0.028) m` 回到 `0`。
   这是修正，不是回归；但**起步点位姿的约定要复核**：如果现场是靠"停车位置 = map 原点"
   来对图的，物理上要把车按新航向摆正（或在 `--map-to-odom X Y YAW` 里给回那 17.8°），
   否则地图里的车头朝向会比以前偏 ~18°。

### 10.5 实车验收（2026-11-13 15:0x，重启后 zlio 前端）

```
[IMU  ] |a|=0.9927, 比力=(+0.0278, -0.5005, +0.8653)  => 实测 roll=-30.05° pitch=-1.59°
[TF   ] gimbal_yaw ← livox_imu  xyz=(+0.127,-0.048,+0.028) rpy=(-30.00, -0.00, 63.00)°
[LIO  ] /lidar_odometry         rpy=(-30.01, -0.00, 63.00)°
[CLOUD] /sensor_scan 3006 点, 地面法向与 +z 夹角 0.87°
TF ↔ IMU : 1.59° OK    LIO ↔ IMU : 1.59° OK    地面 ↔ +z : 0.87° OK   → 退出码 0
```

对照修复前：`TF ↔ IMU 60.06°`（TF +30° / IMU -30°）、`LIO rpy=(-29.78,…)`。
现在 **TF 与 LIO 互相只差 0.01°**，两者与 IMU 的 1.59° 残差全部在 **pitch** 上
（模型 `lidar_pitch=0`，实测 -1.59°，即车/雷达在出生态有一个约 1.6° 的俯仰）；
roll 已经对齐到 0.05°。这 1.59° 如果在**不同车位重复出现**，才说明是安装俯仰，
可以再补 `lidar_pitch=-0.0277`；如果在不同车位变化，那是地面坡度，不要往模型里塞。

`livox_imu` 的 TF 平移从 `(0.088,-0.028,0.052)` 变成 `(0.127,-0.048,0.028)` 是**预期**的：
`livox_imu` 是"雷达原点在 IMU 系里 = −extrinsic_T"，这个 4.4 cm 的偏移会跟着安装角旋转，
按 `Rz(63°)Rx(-30°)·(0.011,0.02329,-0.04412)` + 雷达原点算出来正是
`(0.1275,-0.0478,0.0285)`，与实测一致。
