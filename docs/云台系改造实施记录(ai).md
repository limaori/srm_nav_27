# 云台系改造与决策层小陀螺 —— 实施记录

日期：2026-11-08
对应方案：`docs/导航改云台系与决策层小陀螺实施方案.md`（本文件记录**实际做了什么**、与方案的差异、验证结果与上车待办）
状态：**代码/配置已实施并全部单测通过；未上车**（方案 §5 的第 3/9 步与 §6.2 的实车步骤必须在车上做）

---

## 1. 一句话结论

导航的"车"从**底盘**（`base_link`）改成**云台**（`gimbal_yaw` / `gimbal_yaw_fake`）：
`/odometry` 的位姿只依赖 `livox_imu -> gimbal_yaw` 这段**固定外参**，不再经过关节角；
**TF 树也反过来**（`gimbal_is_root:=true`：`odom -> gimbal_yaw -> 雷达`，底盘挂在云台末端），
所以代价地图/BT 查 `odom -> gimbal_yaw_fake` 也不经过关节角；
导航只出平移（`xy_only`），底盘 `wz` 改由**决策层** `rotation_cmd` 在 `srm_cmd_mux` 里合成；
协议 `0x0010` 扩到 **20 字节**，回传**陀螺仪**转速（编码器转速由关节角求导得到）；
`fake_vel_transform` 的 yaw 外推改用**云台自己的转速**（默认 Mid-360 IMU，验证后切陀螺仪）。

---

## 2. 逐项改动（对应方案 §4）

### 2.1 §4.6 回传速度坐标系（配置）

| 文件 | 改动 |
| --- | --- |
| `src/srm27_nav_protocol/config/srm27_nav_protocol.yaml` | `chassis_speed_frame: base_link -> gimbal_yaw`；注释写明 **frame_id 只描述 vx/vy，`angular.z` 始终是底盘转速** |

### 2.2 §4.1 / §4.1.1 / §4.7 坐标系改云台（配置 + 代码 + 单测）

配置（`config/real/nav2_params_srm_minco_fakeyaw.yaml`）：

| 节点 | 参数 | 改动 |
| --- | --- | --- |
| `sensor_scan_generation` | `robot_base_frame` | `base_link -> gimbal_yaw` |
| `sensor_scan_generation` | `cache_robot_base_transform` | 新增，实车 `True`（默认 `false`，仿真不变） |
| `sensor_scan_generation` | `publish_robot_base_tf` | 新增，实车 `True`：直接发 `odom->gimbal_yaw`，不再发 `odom->base_link`（见 §2.9 的 TF 反转） |
| `sensor_scan_generation` | `base_frame` | 保持 `base_link`（只用于旧路径/对照，本链路不再发它的 TF） |
| `srm27_robot_description`（URDF） | `gimbal_is_root` | 新增，实车 `true`：`gimbal_yaw` 当根、`base_link` 挂末端（见 §2.9） |
| `srm27_nav_protocol` | `gimbal_joint_sign` | `-1.0 -> +1.0`（树反转后关节角 = 底盘相对云台，与回传同向） |
| `loam_interface` | `base_frame` | `base_link -> gimbal_yaw` |
| `loam_interface` | `ground_frame` | **新增**，`base_link`（见 §3.1 的 z 补偿） |
| `zlio_bridge` | `base_frame` / `ground_frame` | 同上（ZLIO 是实车默认前端，必须一起改） |
| `fake_vel_transform` | `robot_base_frame` / `fake_robot_base_frame` | `base_link / base_link_fake -> gimbal_yaw / gimbal_yaw_fake` |
| `bt_navigator` / `local_costmap` / `global_costmap` / `behavior_server` | `robot_base_frame` | `base_link_fake -> gimbal_yaw_fake` |
| `controller_server.FollowPath` | `base_frame` | `base_link -> gimbal_yaw` |
| `controller_server.FollowPath` | `command_frame` / `fake_base_frame` | `base_link_fake -> gimbal_yaw_fake` |

代码：

| 文件 | 改动 |
| --- | --- |
| `sensor_scan_generation/src/sensor_scan_generation.cpp`（+hpp） | ① 新增 `cache_robot_base_transform`：`lidar -> robot_base_frame` 开机查一次并缓存（固定外参）；② `lidar -> base_frame` 查询失败时**只跳过 `odom->base_link` 的 TF**并告警，`/odometry` 照常发；③ 缓存模式下这条可选查询用 0.02 s 超时（tf2 对不存在的帧也会等满 timeout，实测 0.5 s/帧会把 `/odometry` 拖到 2 Hz）；④ `getTransform()` 增加 `timeout_seconds` 参数（默认 0.5，行为不变） |
| `sensor_scan_generation/test/test_odometry_without_joint_tf.cpp` | **新增单测**：只发 `gimbal_yaw -> livox_imu` 静态 TF、**不发** `base_link` 时，缓存模式仍按点云频率输出 `/odometry` 且不发 `odom->base_link`；缓存关闭时行为与改造前一致（整帧跳过） |

### 2.3 §4.2 导航只出平移（配置 + launch + 脚本）

| 文件 | 改动 |
| --- | --- |
| `nav2_params_srm_minco_fakeyaw.yaml` | `FollowPath.yaw_policy.mode: se2 -> xy_only`；`general_goal_checker.plugin -> srm27_nav_plugins::OmniStoppedGoalChecker`（只看平移） |
| `config/real/speed_limits.yaml` | `velocity_smoother@fakeyaw` 的 `max/min_velocity` yaw 分量 `±1.0 -> 0.0`，`max/min_accel(decel)` yaw `±4.5 -> 0.0` |
| `launch/nav_srm_real_launch.py` | 默认 `yaw_mode=xy_only`（`auto`/`se2` 都改写成 `xy_only`）；判停器对 `spin/xy_only/auto/se2` 一律换 `OmniStoppedGoalChecker`；`command_frame` 默认 `gimbal_yaw_fake` 并据此校验参数文件；新增 `mux_spin_topic`（默认 `rotation_cmd`）传给 mux；启动日志打印两路输入 |
| `script/start_real_nav.sh` | 默认 `YAW_MODE=xy_only`；`--yaw-mode se2` **直接拒绝**并提示改用 `rotation_cmd`；`--yaw-mode spin` 保留参数面但改写成 `xy_only` 并提示；`--chassis-spin` **不再**切 MINCO 的 `spin` 模式（请求走 mux）；迁移链识别同时接受 `gimbal_yaw_fake` 与旧 `base_link_fake`；启动摘要与验收提示按新链路重写 |

### 2.4 §4.3 决策层小陀螺进 mux（代码 + 单测）

| 文件 | 改动 |
| --- | --- |
| `srm27_chassis_control/src/cmd_mux_node.cpp` | **两路输入**：`nav_topic` 只取 `linear.x/y`（`angular.z` 一律忽略 + 计数 + 告警），`spin_topic` 只取 `angular.z`；两路**各自**超时（`nav_timeout` / `spin_timeout`）；`wz` 加变化率限制 `wz_ramp`（`stop_all` 例外，立即归零）；`stop_all`/`resume_all` 同时清两路；诊断新增 `dual_input` / `spin_wz_input` / `spin_freq_hz` / `spin_valid` / `spin_timeout_count` / `spin_input_count` / `non_finite_spin_count` / `nav_wz_ignored_count` / `output_wz` / `wz_ramp(_active)` |
| `srm27_chassis_control/include+src/velocity_mix.*` | `MixInput` 增加 `spin_enabled/spin_wz/spin_valid`，`MixOutput` 增加 `wz_from_spin/nav_wz_ignored`，`MixLimits` 增加 `wz_ramp`；新增纯函数 `slewLimit()`、`isNonZeroRate()`。`spin_enabled=false` 时**逐字保持**旧行为（仿真/回退） |
| `srm27_chassis_control/config/srm_chassis_control.yaml` | 新增 `spin_topic`（**默认空 = 单输入**）、`spin_timeout: 0.25`、`wz_ramp: 4.5` |
| `srm27_chassis_control/launch/srm_chassis_control.launch.py` | 新增 `spin_topic` launch 参数并透传（默认空 → 仿真链行为不变；实车入口传 `rotation_cmd`） |
| `srm27_chassis_control/test/test_velocity_mix.cpp` | **新增单测**：双输入分工、导航 wz 被忽略、决策层超时=停转、`wz_max` 钳位、单输入回退、`slewLimit` 的 6 条边界 |

**决策层接口**（与方案 §4.3 / §8.2 一致）：`/<ns>/rotation_cmd`，`geometry_msgs/Twist`，
**只读 `angular.z`**（rad/s，俯视逆时针为正）。持续转就以 ≥10 Hz 持续发；停发 0.25 s 即停转；
也可以直接发 0。`script/spin_control.sh` 不用改（它本来就是发这个话题）。

### 2.5 §4.4.1 协议加转速字段（代码 + 文档 + 测试）

| 文件 | 改动 |
| --- | --- |
| `srm27_nav_protocol/include/packet_typedef.hpp` | `DATA_LENGTH_CHASSIS_SPEED: 16 -> 20`；新结构体加 `gimbal_yaw_rate_imu`（数据段偏移 16，`static_assert` 钉死 29 字节与字段偏移）；**新增旧帧结构体** `ReceiveChassisSpeedDataV1`（25 字节）。用旧结构体读新帧会补零 + 刷 stderr 警告，所以必须分开。**不加编码器转速字段**：它就是 `chassis_to_gimbal_yaw` 的变化率（2026-11-08 决定） |
| `srm27_nav_protocol/src/srm27_nav_protocol.cpp` | `0x0010` 按 `data_length` 分支：20=新帧（发 4 条话题）、16=旧帧（转速话题不发、节流告警一次、**不丢帧**）、其它=丢弃并计数；`publishChassisSpeed()` 改为按字段传值；新增 `gimbal_yaw_rate`（`TwistStamped`，与其它话题共用同一个 stamp）；NaN/Inf 整帧丢弃也会覆盖新字段 |
| `src/srm27_nav_protocol/config/srm27_nav_protocol.yaml` | 新增 `gimbal_yaw_rate_topic`，并更新 `0x0010` 的字段/坐标系说明；`gimbal_joint_sign` 改 `+1.0` |
| `script/test_chassis_speed_feedback.py` | 参考实现：`LEN_CHASSIS_SPEED=20` + `LEN_CHASSIS_SPEED_V1=16`；`build_chassis_speed()` 4 参=旧帧、5 参=新帧；`decode()` 两种都认（旧帧转速为 `None`）；`--pty` 默认发新帧（`--rate-imu`），`--legacy` 发旧帧；`--self-test` 同时校验新旧两种帧长 |
| `src/srm27_nav_protocol/test/test_feedback_protocol.py` | 新帧形状（29 B / `data_length=20` / 偏移 16）、**旧帧兼容**、非法长度仍被拒 |
| `script/check_chassis_speed_chain.sh` | 伪串口改发新帧（带两路转速），长度告警文案改成 24/16 |
| `docs/导航通信协议-下位机接口说明.md` | §0/§1/§4.2/§4.4/§4.5/附录 A 全面更新（字段表、结构体、四条话题、示例帧、新旧帧兼容规则、关节角符号随树方向） |

### 2.6 §4.4.2 / §4.4.3 假车头补延迟（代码 + 单测）

| 文件 | 改动 |
| --- | --- |
| `fake_vel_transform/include+src` | 新增 `yaw_rate_source`（`cmd_wz` / `gimbal_rate` / `imu`，非法值直接起不来）、`gimbal_rate_topic`、`imu_topic`、`imu_target_frame`、`gimbal_rate_timeout`；`gimbal_rate` 订阅 `TwistStamped.angular.z`；`imu` 订阅 `sensor_msgs/Imu`，用 `livox_imu -> imu_target_frame` 的**固定旋转**把角速度投影到竖直轴（缓存一次，不逐帧查 TF）；积分区间仍是 `[odom_stamp - d, now]`；断流超过 `gimbal_rate_timeout` 时推算项按 0 并计数 |
| `fake_vel_transform/include+src/rate_integration.*` | **新增纯逻辑**：实测转速的分段常值积分（零阶保持、断流按 0、左端未覆盖只少算）；节点只负责把 `rclcpp::Time` 换成秒 |
| `fake_vel_transform/test/test_rate_integration.cpp` | **新增单测** 9 条（恒速、阶跃、断流、尾段、左端、空样本、退化区间、门限非法、负转速） |
| `config/real/nav2_params_srm_minco_fakeyaw.yaml` | `yaw_rate_source: "imu"`（`gimbal_rate` 待 §8.1 验符号）、`actuation_delay: 0.02 -> 0.05`（方案 §4.4.2 的起始值，按 §6.2 第 4 步实测再定） |

### 2.7 §4.5 点云自车过滤（核对结论：**无需改代码**）

逐项核对（下游链路 + `intensity_voxel_layer` + 两个 terrain 节点）：

- **全仓库没有任何方框（box）自车过滤器**；所有自车/近场过滤都是**半径**或 **z 带**：
  - `point_lio preprocess.blind = 0.3`（以**雷达**为心的球，2026-11-06 已刻意降到小于车身外廓）；
  - `zlio frontend minimum_range = 0.3`（同上）；
  - `terrain_analysis minDyObsDis = 0.3`（以**雷达**为心的 XY 圆盘；圆心是雷达，偏轴 0.133 m）；
  - `intensity_voxel_layer obstacle_min_range = 0.2`（以 `sensor_frame=livox_imu` 为心的"太近不可信"门）；
  - **代价地图 `footprint_clearing`（圆，`robot_radius: 0.40`，圆心 = 导航机器人位姿 = 云台轴）** —— 这是真正意义上的自车过滤，已经是圆、半径 ≥ 0.33、与底盘自转无关。
- z 带（`minRelZ/maxRelZ`、`lowerBoundZ/upperBoundZ`、`min/max_obstacle_height`）对 yaw 旋转不变，换系不影响。
- 结论：**没有方框需要改成圆**；`gimbal_yaw_fake` 与原 `base_link_fake` 同在 yaw 轴上，代价地图的自车清除圆半径与圆心都没变。
- ⚠ 唯一遗留：terrain 的圆盘以**雷达**为心（偏轴 0.133 m），有效自车半径约 0.167 m。把它抬到 0.47 m 会同时屏蔽 0.3~0.47 m 的真实近场回波，属于"用避障换干净"，**本次刻意不动**；若以后出现"车身自反射进 ESDF"，再按 0.33+0.133 提到 0.47。

### 2.8 其它顺带修掉的问题

| 文件 | 改动 |
| --- | --- |
| `loam_interface`（代码）、`zlio_bridge`（脚本） | **新增 `ground_frame`**：云台系让 `base_frame` 离地 0.268 m，`T(base<-lidar)` 快照会把 **odom 的 z=0 抬到云台平面**（地面变成 z=-0.268），而代价地图的 `min/max_obstacle_height`、`origin_z` 与 `terrain_map` 的绝对 z 都是绝对值。现在在拍快照时把这段高度反向补偿，odom z=0 仍在地面，**所有下游 z 阈值一个都不用改**。留空 = 不补偿（仿真/旧链行为不变） |
| `scripts/srm_minco_real_preflight.py` | ① 到点判停器检查按 `yaw_policy.mode` 区分（`xy_only` 期望 `OmniStoppedGoalChecker`，`se2/spin` 才期望 `StoppedGoalChecker`）；② 新增云台系检查（`cache_robot_base_transform`、`base_frame != robot_base_frame`、`ground_frame` 补偿、`yaw_rate_source` 合法性/合理性）；③ 更新 `xy_only` 的说明文案 |
| `test/test_fakeyaw_real_wiring.py` | 全部按新坐标系重写断言，并补齐"mux 订阅 `rotation_cmd`""`/odometry` 的 child = `gimbal_yaw`""固定外参缓存""`ground_frame` 补偿""`yaw_rate_source=imu`""判停器只看平移"等回归 |
| `test/test_cli_parity.py` | 断言改为"`--chassis-spin` 仍在，但**不再**切 `YAW_MODE=spin`" |

---

### 2.9 TF 树反转（2026-11-08 补充，方案原稿漏项）

原方案只把 `/odometry` 改成云台位姿，但**代价地图 / BT / 假车头查的是 TF**：只要
`odom -> gimbal_yaw` 这一段还经过 `base_link`，关节角那几十度的误差就照样进代价地图
（云台快转时 §1.2 的现象会复现一半）。所以树必须反过来：

```text
旧：odom -> base_link（底盘）--[关节角]--> gimbal_yaw -> 雷达
新：odom -> gimbal_yaw（固定外参）-> 雷达
      gimbal_yaw --[关节角]--> base_link（底盘，末端）
```

| 文件 | 改动 |
| --- | --- |
| `srm27_robot_description/urdf/srm27_sentry.urdf.xacro` | 新增 `gimbal_is_root`（默认 false）：为 true 时 `gimbal_yaw_joint` 变成 `gimbal_yaw -> base_link`、origin z 取 `-chassis_top`，根只剩 `gimbal_yaw`；`root_frame=base_footprint` 在该模式下被忽略（否则会出现两个根）。仿真/旧配置不传，行为逐字不变 |
| `scripts/srm_robot_description.py` | `build_description(..., gimbal_is_root=)` 透传；`lidar_mounting()` 从"只往上走"改成**无向关节图 BFS**（树反转后 `base_link` 在雷达下面，旧实现会 ValueError），两种方向解析出的安装位姿逐位相同；`livox_scan` 挂到雷达所在那一支（反转时挂 `gimbal_yaw`，偏移 = `T(gimbal_yaw <- 雷达)`） |
| `sensor_scan_generation`（代码） | 新增 `publish_robot_base_tf`：为 true 时直接发 `odom->robot_base_frame`，**完全不查** `base_frame` 也不再发 `odom->base_link`（否则 `base_link` 有两个父节点） |
| `launch/real_robot_state_publisher_launch.py`、`launch/real_mapping_launch.py` | 新增 `gimbal_is_root` 参数（默认 false） |
| `script/start_real_nav.sh` | `RSP_CMD` 加 `gimbal_is_root:=true`（实车） |
| `srm27_nav_protocol.yaml` | `gimbal_joint_sign: -1.0 -> +1.0`（关节角语义变成"底盘相对云台"） |
| `test/test_gimbal_tree_direction.py` | **新增 6 条回归**：默认方向 / 反转后的根与关节 z / 两种方向安装位姿逐位相同 / 实车参数开 `publish_robot_base_tf` / 脚本传 `gimbal_is_root:=true` / `gimbal_joint_sign == +1.0` 且没有编码器转速话题 |
| `sensor_scan_generation` 单测 | 新增第 3 条用例：树反转模式下发 `odom->gimbal_yaw` 且**绝不**发 `odom->base_link`（即使关节 TF 存在） |
| `scripts/srm_minco_real_preflight.py` | 新增检查：`robot_base_frame != base_link` 但 `publish_robot_base_tf` 未开 → **WARN**（"TF 还绕 base_link，关节角会进代价地图"） |

效果：关节角只出现在树的末端（`gimbal_yaw -> base_link`）。它断流时只有 RViz 里的底盘
消失，`/odometry`、代价地图、规划全都不受影响。

⚠ **建图链（`real_mapping_launch.py` + `config/real/mapping_params.yaml`）保持原方向**：
它的 `sensor_scan_generation.robot_base_frame` 仍是 `base_link`、没有 `publish_robot_base_tf`，
URDF 也不传 `gimbal_is_root` —— 两边一致，行为不变（建图/录包不在本方案范围内）。
将来若要让建图也走云台系，**必须同时**改这三处，否则 `gimbal_yaw` 会出现两个父节点。

## 3. 与方案的差异（都写在代码/配置注释里）

0. **方案 §4.1 漏了 TF 树的方向**：原稿只改 `/odometry`，但代价地图/BT 查 TF，绕不过
   关节角。已补：URDF `gimbal_is_root:=true` + `sensor_scan_generation.publish_robot_base_tf`
   + `gimbal_joint_sign` 改 `+1.0`（详见 §2.9）。方案文档已同步更新。
1. **§4.1.1 的 z 补偿**：方案只提到"`gimbal_yaw` 的 z 不在地面，要逐项检查代价地图/地形"。实测发现真正的机制是 **`loam_interface`/`zlio_bridge` 的开机快照把 odom 的 z 原点抬到了云台平面**（地面 z=-0.268），影响面比 `robot_base_frame` 的 z 大得多（`sensor_scan_generation` 的 `/odometry` z 恒被压平为 0）。因此改成**在源头补偿**（新增 `ground_frame`），而不是逐项改下游阈值 —— 下游参数与仿真保持完全一致。
2. **mux 的 `spin_topic` 默认空 = 单输入**：方案写"默认 `rotation_cmd`"，但 mux 的配置文件是**仿真链与实车链共用的**，仿真里 `wz` 仍由 MINCO 输出。做法：节点代码默认 `rotation_cmd`，共享配置写空（= 单输入，仿真/回退行为不变），**实车入口 launch 显式传 `rotation_cmd`**。方案 §7 的"`spin_topic` 留空即单输入"正好就是这个开关。
3. **协议只加**一路**转速（20 字节）**：编码器转速 = `chassis_to_gimbal_yaw` 的变化率，
   由上位机求导即可（2026-11-08 决定，方案原稿写的是加两路 24 字节）。
4. **`yaw_rate_source` 实车默认 `imu`（不是 `gimbal_rate`）**：方案 §4.4.2 的表把 `gimbal_rate` 标成"实车默认"，但 §4.4.3 说明协议字段落地前先用 Mid-360 IMU，且 §8.1 要求两个转速的**符号上车实测**。现在协议字段已实现，但符号未测，所以默认 `imu`（Mid-360 装在云台上，正方向由安装姿态唯一确定）；**§8.1 测完再切 `gimbal_rate`**，切之前先两路对照。
5. **`rotation_cmd` 的消息类型是 `geometry_msgs/Twist`**（只读 `angular.z`）：方案 §8.2 写成 `std_msgs/Float32`，但现有 `script/spin_control.sh`、MINCO 的 `spin_request` 都用 `Twist`；为不破坏现有工具，统一用 `Twist`。
6. **`--yaw-mode spin` 保留但按 `xy_only` 跑**：方案要求"`se2` 直接拒绝"，`spin` 没说。实车脚本把 `spin` 改写成 `xy_only` 并提示（请求走 mux）；直接 `ros2 launch ... yaw_mode:=spin` 仍会照原样注入 `spin`，用于 A/B 对照。
7. **`zlio_bridge` 一起改**：方案 §4.7 只点名 `loam_interface`，但实车默认前端是 ZLIO，同样的开机快照问题必须一起处理，否则换前端就复现。

---

## 4. 验证结果（本次已跑）

编译（`colcon build --symlink-install`，全部通过）：`srm27_chassis_control`、`sensor_scan_generation`、
`fake_vel_transform`、`loam_interface`、`srm27_nav_protocol`。

单测/回归：

| 用例 | 结果 |
| --- | --- |
| `srm27_chassis_control`：`test_velocity_mix`（17+6 条，含双输入/斜率）与 `test_twist_watchdog` | 通过 |
| `sensor_scan_generation`：`test_odometry_without_joint_tf`（关节角 TF 缺失时 `/odometry` 照常输出；关闭缓存时保持旧行为；**树反转时只发 `odom->gimbal_yaw`、绝不发 `odom->base_link`**） | 通过 |
| `fake_vel_transform`：`test_velocity_transform` + 新增 `test_rate_integration`（9 条） | 通过 |
| `srm27_nav_protocol`：`test_feedback_protocol.py`（含新旧帧兼容） + `script/test_chassis_speed_feedback.py --self-test` | 通过 |
| `srm27_nav_bringup`：`test_fakeyaw_real_wiring`、`test_fakeyaw_launch_wiring`、`test_cli_parity`、`test_lio_frontend_wiring`、`test_zlio_bridge_math`、`test_speed_limits`、**`test_gimbal_tree_direction`（新增 6 条）** | 通过 |
| `srm_minco_real_preflight.py --expect-minco`（实车 fakeyaw 参数） | **0 个阻塞问题**，6 警告（都是改造前就有的：制动未辨识、`reaction_latency=0`、`command_lookahead` 未标定等） |
| `srm_speed_limits_check.py` | 分层自洽，无阻塞 |

`cmd_mux` 运行时冒烟（`ROS_DOMAIN_ID=77` 隔离，未接硬件）：双输入启动、导航 `angular.z=0.7`
被忽略并告警、`100 Hz` 输出、`wz` 斜坡 4.5 rad/s²、决策层停发 0.25 s 后 `wz -> 0`
（实测归零 ≈ 1.63 s，理论 1.67 s）。

> 环境注意：本机的 `~/.ros` 是只读的，`launch` 相关测试要 `export ROS_LOG_DIR=/tmp/...` 才能跑；
> 两个包里的 `clang_format` 测试因为**仓库里没有 `.clang-format`** 一直是失败的（改造前就如此），与本方案无关。

---

## 5. 上车待办（按顺序，前一步不过不进下一步）

> ⚠ 方案 §1.3 的警告仍然有效：下位机已按云台系执行 vx/vy，**在完成下面第 1、2 步之前不要跑实车导航**。

0. **先看树**：`ros2 run tf2_ros tf2_echo gimbal_yaw base_link` 应得 `z = -0.26805`，
   `tf2_echo odom gimbal_yaw` 的父系是 `odom` 而不是 `base_link`（`ros2 run tf2_tools view_frames`）。
   树没反过来的话下面所有测试都会"看起来正常但代价地图仍在抖"。
1. **静止 + 云台快转**（底盘不动，云台 150°/s，30 s）：
   `tf2_echo gimbal_yaw_fake odom` 的 yaw 匀速变化、xy 不画圈（<2 cm）；`/odometry` 线速度 <0.03 m/s；
   `debug_tools/capture_gimbal_spin.py` 的 raw LIO 斜率应为 1.00。
   不过 —— 说明 LIO 在高速转动下本身估不准，先查 LIO 外参/重力，本方案不解决。
2. **架车确认方向**：云台与底盘差 90°，发前进 0.2 m/s，车朝**云台**指的方向走；
   云台转动时给前方 1 m 的小目标，平移方向不随云台转动而偏。
3. **小陀螺**（架车）：`script/spin_control.sh on 0.5` → 底盘转；停发 0.25 s 停转；
   底盘转时 `/odometry` 位置不变、速度 <0.03 m/s。
4. **测延迟**：云台匀速转 + 直线走 2 m，看 `yaw_debug` 与实际轨迹调 `actuation_delay`（当前 0.05 s）。
5. **落地跑点**：先云台不转、小陀螺关，再逐项打开；日志里 `validation_failed` 的"当前速度"不再 >0.42；
   不出现 `进入脱困`（或只在真贴障碍时出现）。
6. **协议转速符号（§8.1）**：下位机发新固件后，底盘不动云台自转，比较 `gimbal_yaw_rate` 与
   `/livox/imu` 换算后的 z 转速（同号、大小接近）；再让底盘小陀螺、云台稳向，比较编码器转速方向。
   测完把 `yaw_rate_source` 切到 `gimbal_rate`，两路对照一次再定默认。
7. **云台零位（§8.3）**：按方案 §8.3 测 `gimbal_yaw` 的 x 轴与"云台朝正前方"是否一致（±3°）；
   超差就在 URDF 的 `gimbal_yaw_joint` 上加固定 yaw 偏移。

---

## 6. 回退开关一览（每一步都可单独回退）

| 开关 | 位置 | 回到改造前 |
| --- | --- | --- |
| `chassis_speed_frame` | `srm27_nav_protocol.yaml` | `base_link` |
| `sensor_scan_generation.robot_base_frame` / `base_frame` / `cache_robot_base_transform` / `publish_robot_base_tf` | 实车参数文件 | `base_link` / `base_link` / `false` / `false` |
| URDF `gimbal_is_root` + `srm27_nav_protocol.gimbal_joint_sign` | `start_real_nav.sh` 的 RSP_CMD / 协议 yaml | `gimbal_is_root:=false` / `-1.0`（两者必须一起改） |
| `loam_interface.base_frame` / `ground_frame` | 实车参数文件 | `base_link` / 空 |
| `zlio_bridge.base_frame` / `ground_frame` | 实车参数文件 | 同上 |
| `fake_vel_transform.robot_base_frame` / `fake_robot_base_frame` | 实车参数文件 | `base_link` / `base_link_fake` |
| 四处 `robot_base_frame` + `FollowPath.command_frame/fake_base_frame/base_frame` | 实车参数文件 | `base_link_fake` / `base_link` |
| `yaw_policy.mode` + 判停器 | 实车参数文件 + launch | `se2` + `StoppedGoalChecker` |
| `velocity_smoother@fakeyaw` 的 yaw 界 | `config/real/speed_limits.yaml` | `±1.0` |
| mux `spin_topic` | `srm_chassis_control.yaml` / 实车 launch | 留空 = 单输入 |
| `fake_vel_transform.yaw_rate_source` | 实车参数文件 | `cmd_wz`（+ `chassis_cmd_vel_topic`） |

整体回退：把上表全部还原，并让下位机切回底盘系下发/回传。
