# 仿真云台系改造实施记录（云台系速度 + wz 与导航解耦）

日期：2026-11-12
对应实车改造：[`docs/云台系改造实施记录(ai).md`](云台系改造实施记录%28ai%29.md)（2026-11-08 实车链）
状态：**已实施并验证**（单元/回归 46 项通过；无头 Gazebo 冒烟；`--verify-spin` 端到端 PASS）。
仿真入口：`script/start_sim_nav.sh`。

---

## 1. 一句话结论

仿真导航从"底盘系（base_link）+ 导航拥有 wz"改成**与实车一致的云台系**：

* 导航的"车"是云台：`odom -> gimbal_yaw` 只依赖固定外参（`livox -> gimbal_yaw`），
  TF 树反过来（URDF `gimbal_is_root:=true`）；
* 导航**只出平移**（`yaw_policy.mode=xy_only`），底盘 wz 由决策层 `rotation_cmd`
  经 `srm_cmd_mux` 合成，**完全和导航解耦**；
* 仿真的"下位机"（`SrmVelocitySystem`）按**云台系**解释 vx/vy：执行前按云台关节角
  旋到底盘系（等价实车 MCU 按云台的"前"执行）；真值里程计改为输出
  `odom -> gimbal_yaw`。

---

## 2. 数据流（改造后）

```text
Gazebo（云台 + 雷达，gimbal_spin_rate 可转）
  └─ SrmVelocitySystem
       ├─ 真值里程计：gimbal_yaw 的位姿/速度（child_frame = gimbal_yaw）
       ├─ 速度执行：vx/vy 按云台系解释 -> Rz(theta) -> 底盘 link 系执行；wz 不旋转
       └─ 云台关节恒速（与底盘速度独立）

simulation_ground_truth_odometry（GT 模式）
  └─ odom -> gimbal_yaw（z 按 chassis_top 补偿回地面），/odometry child = gimbal_yaw

Nav2 / MINCO（gimbal_yaw_fake，xy_only，只出 vx/vy）
  └─ cmd_vel_nav_fake -> fake_vel_transform（gimbal_yaw -> gimbal_yaw_fake，
       yaw 外推用云台 IMU 实测角速度）
       -> cmd_vel_nav_body（云台系 vx/vy，wz = 0）
       -> srm_cmd_mux ◄─ rotation_cmd（决策层底盘 wz，只取 angular.z）
       -> cmd_vel_sim -> srm_velocity_adapter -> Gazebo
```

与实车链的差别只剩执行端最后一段（实车是 `cmd_vel_chassis -> srm27_nav_protocol -> 串口`）。

---

## 3. 改动清单

### 3.1 仿真器（模型 / 执行 / 里程计）

| 文件 | 改动 |
| --- | --- |
| `srm27_gazebo_simulator/plugins/srm_velocity_system/SrmVelocitySystem.cc` | 新增 SDF 参数 `command_in_gimbal_frame`：为 true 时读云台关节角 θ，把输入平移按 `v_body = Rz(θ)·v_gimbal` 旋到底盘再执行（wz 不旋转，始终是底盘角速度）；新增 `odom_link`：真值里程计输出该 link 的位姿/速度（云台系填 `gimbal_yaw`），`child_frame_id` 随之改为该 link；关节角不可用时告警一次并按底盘系执行 |
| `srm27_robot_description/model_builder.py` | `build_sdf(..., command_in_gimbal_frame=False, odom_link="")` 写入插件参数；`build_urdf(..., gimbal_is_root=False, gimbal_axis_sign=1.0)` 透传反转树与轴符号 |
| `srm27_robot_description/urdf/srm27_sentry.urdf.xacro` | 新增 `gimbal_axis_sign`（默认 `1.0`，实车行为不变）：仿真反转树时传 `-1.0`，因为 Gazebo `/joint_states` 的云台角是"云台相对底盘"，与反转后 URDF 关节（底盘相对云台）方向相反 |
| `srm27_gazebo_simulator/launch/srm_sim.launch.py` | 新增 `velocity_command_frame`（chassis/gimbal，默认 chassis）与 `odom_link`；`gimbal` 时自动传 `command_in_gimbal_frame=true`、`odom_link=gimbal_yaw`、`build_urdf(gimbal_is_root=true, gimbal_axis_sign=-1)` |

> 默认仍是 `chassis`：`start_sim_slam.sh` 等旧链路不受影响（建图不在本次范围）。

### 3.2 真值里程计（`simulation_ground_truth_odometry.py`）

* `base_frame` 改为 `gimbal_yaw`（launch 固定传入）。
* 新增 `base_height`（默认自动取几何 YAML 的 `chassis_top=0.26805`）：里程计原点 z 取
  "云台世界 z − 云台高度"，保证 odom 原点仍是**地面**，与旧链路的 z 约定一致，下游 z 阈值不用改。
* `gimbal_yaw -> 雷达` 的 TF 兜底外参扣掉云台高度（固定外参）。

### 3.3 仿真 Nav2 参数（`config/simulation/nav2_params_srm_fakeyaw.yaml`）

| 节点/键 | 改为 |
| --- | --- |
| `sensor_scan_generation.robot_base_frame` | `gimbal_yaw`；新增 `cache_robot_base_transform: true`、`publish_robot_base_tf: true`（GT 模式 `publish_tf: False` 不变） |
| `loam_interface` / `zlio_bridge` | `base_frame: gimbal_yaw`、`ground_frame: base_link`（把 odom 的 z 原点拉回地面） |
| `fake_vel_transform` | `robot_base_frame: gimbal_yaw`、`fake_robot_base_frame: gimbal_yaw_fake`；`yaw_rate_source: imu`（`livox/imu` -> `gimbal_yaw` 的 z 分量；云台系下不能再用底盘 cmd_wz）、`actuation_delay: 0.05` |
| `bt_navigator` / `global_costmap` / `local_costmap` / `behavior_server` | `robot_base_frame: gimbal_yaw_fake` |
| `controller_server.FollowPath` | `base_frame: gimbal_yaw`、`command_frame/fake_base_frame: gimbal_yaw_fake`、`yaw_policy.mode: xy_only` |
| `general_goal_checker` | `srm27_nav_plugins::OmniStoppedGoalChecker`（只看平移，去掉 `rot_stopped_velocity`） |
| `spin_request.topic` | 保留 `rotation_cmd`（xy_only 下只缓存并拒绝，保留给 A/B） |

### 3.4 仿真限幅（`config/simulation/speed_limits.yaml`）

新增 `velocity_smoother@fakeyaw` 叠加段：平移档不动，`yaw` 的 max/min/accel/decel 全部钳 0 ——
"导航漏出 wz"的最后一道上游闸（与 `config/real/speed_limits.yaml` 同义）。
`detect_profile` 通过参数文件里的 `gimbal_yaw_fake` 自动命中该段。

### 3.5 仿真入口（`launch/nav_srm_simulation_launch.py`）

* `command_frame` 默认 `gimbal_yaw_fake`，启动核对只接受它（不再接受 `base_link_fake`）。
* 新增 `mux_spin_topic`（默认 `rotation_cmd`），传给 `srm_chassis_control.launch.py` 的 `spin_topic`。
* `yaw_mode`：`auto/se2 -> xy_only`；`spin` 保留给直接 launch 的 A/B；判停器一律
  `OmniStoppedGoalChecker`。
* GT 里程计参数 `base_frame:=gimbal_yaw`；`start_simulation:=true` 时给 `srm_sim.launch.py`
  传 `velocity_command_frame:=gimbal odom_link:=gimbal_yaw`。

### 3.6 启动脚本（`script/start_sim_nav.sh`）

* 默认航向策略 `xy_only`；`auto -> xy_only`；`spin -> 按 xy_only 跑并提示`；`se2` 直接拒绝
  （云台系下导航的 yaw 是云台、wz 转的是底盘）。
* `--chassis-spin` 不再切 MINCO 的 spin 模式，**不再要求 `--yaw-mode spin`**：请求直接发
  `rotation_cmd`，由 mux 消费；没有导航目标时也照转；停发 0.25 s 停转。
* 命令系 `gimbal_yaw_fake`；参数文件校验改为 `gimbal_yaw_fake`；仿真命令加
  `velocity_command_frame:=gimbal odom_link:=gimbal_yaw`。
* 摘要 / 运行提示 / `--verify-spin` 断言全部按新契约重写：
  `导航 fake/body 的 wz 恒 0`、`mux 输出 wz 跟随 rotation_cmd`、`边转边走 / 到点成功 / 撤回即停`。

### 3.7 工具与测试

| 文件 | 改动 |
| --- | --- |
| `script/spin_control.sh` | 文档与 `status` 文案改为"云台系由 mux 消费；旧 spin 模式才由 MINCO 消费"；三层速度段说明 nav 两层 wz=0、执行层 = 请求值 |
| `test/test_fakeyaw_launch_wiring.py` | 断言改为云台系：`gimbal_yaw_fake`、`gimbal_yaw`/`gimbal_yaw_fake`、`xy_only`、`OmniStoppedGoalChecker`、`spin_topic=rotation_cmd`、`sensor_scan_generation`/`loam_interface`/`zlio_bridge` 的云台系与 `ground_frame` |

---

## 4. 验证结果

### 4.1 单元/回归

```bash
export ROS_LOG_DIR=/tmp/ros_log
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  src/srm27_navigation/srm27_nav_bringup/test/
# 46 passed
```

`srm27_gazebo_simulator` 重新编译通过；`srm_speed_limits_check.py` 对仿真 fakeyaw 配置判定
"分层自洽"（已命中 `@fakeyaw` profile）。

### 4.2 无头 Gazebo 冒烟（单测覆盖不到的执行端）

| 用例 | 结果 |
| --- | --- |
| 云台系速度执行 | `gimbal_spin_rate=-1.0`、发 `cmd_vel_sim.vx=0.3`：真值里程计的**云台系** twist = `vx=+0.298, vy=+0.000, wz=-1.000`；云台 yaw 7 s 转 -6.98 rad。底盘按云台前向走圆弧（与实车 MCU 按云台执行的语义一致） |
| 真值里程计 | `child_frame_id=gimbal_yaw`；`odom pose z=0.268`（云台离地），`TF odom->gimbal_yaw z=0.2681` |
| mux 双输入 | nav `(0.2, 0.05, wz=0.3)` + `rotation_cmd wz=1.0` -> `cmd_vel_sim (0.200, 0.050, 1.000)`；日志明确"导航输入带了非零 angular.z，已忽略" |
| 完整栈 | `controller_server active`；`yaw_policy.mode=xy_only`、`command_frame=gimbal_yaw_fake`、`OmniStoppedGoalChecker`、`srm_cmd_mux.spin_topic=rotation_cmd`；TF `gimbal_yaw -> gimbal_yaw_fake` 存在 |

### 4.3 端到端 `--verify-spin`

```bash
SRM27_GAZEBO_LOCK_FILE=/tmp/srm27_gazebo.lock \
OPEN_MODE=direct ./script/start_sim_nav.sh --verify-spin \
  -w rmuc_2025 -m rmuc_2025 --no-gui --no-rviz
```

结果：**PASS**（节选）

```text
wz 统计: fake +0.000, body +0.000, sim +2.000
[PASS] 导航只出平移（fake 的 wz 恒 0，wz 已与导航解耦）
[PASS] fake -> body 不引入 wz（body 的 wz 也恒 0）
[PASS] mux 输出的 wz 跟随决策层自转请求（rotation_cmd -> mux）
[PASS] 平移也在动
[PASS] 里程计实际转向与请求同号（odom_wz +1.999）
[PASS] 导航到点成功（status=4），终点误差 0.39~0.47 m（多次运行）
[PASS] 撤回请求后车停转（尾段 odom_wz=+0.238）
结论: PASS（导航只出平移、wz 走 mux、边转边走、到点成功、撤回即停）
```

---

## 5. 与实车链的差异 / 已知注意点

1. **GICP / SLAM 未迁移**：`small_gicp_relocalization`、`slam_toolbox` 仍用 `base_link`；
   本次只覆盖 GT / LIO / ZLIO 三种定位。`--gicp` 仍可用（`map->odom` 是变换，与参考系无关），
   但 `--reloc` 的适配不在本次范围。
2. **建图链不变**：`start_sim_slam.sh` 仍用 `nav2_params_srm.yaml` + `srm_sim.launch.py` 默认
   `velocity_command_frame=chassis`，与云台系导航不混用。
3. **yaw 外推用 IMU**：仿真的 Mid-360 IMU 由 Gazebo 给出，`front_mid360 -> gimbal_yaw`
   固定旋转可查；云台系下底盘 `cmd_wz` 不再代表云台朝向，不能用。
4. **关节轴符号**：仿真反转树时 `gimbal_axis_sign=-1.0`（Gazebo 关节角是"云台相对底盘"）；
   实车 `build_description` 不传，保持 `+1.0`（下位机回传"底盘相对云台"）。
5. **`--gimbal-spin` 的语义变了**：以前只让雷达转，导航仍是底盘系；现在导航是云台系，
   云台相对底盘转动时"云台前向"在变，底盘会按云台前向走圆弧 —— 这正是实车 MCU 的行为。
   只想验证"底盘不转、云台转"时，导航需要用云台系看（代价地图随云台转，点云也在转，
   属预期）。
6. **默认仍不转**：`--chassis-spin` / `--gimbal-spin` 都不给 = 底盘与云台都不转。

## 6. 回退

| 目标 | 做法 |
| --- | --- |
| 只回退"云台系执行" | `srm_sim.launch.py` 传 `velocity_command_frame:=chassis odom_link:=`；GT 节点 `base_frame:=base_link`；参数文件换回改造前版本（或把四处 `gimbal_yaw*` 改回 `base_link*`、`yaw_policy.mode: se2`、判停器 `StoppedGoalChecker`） |
| 只回退"wz 解耦" | `mux_spin_topic:=`（空 = 单输入）+ `yaw_mode:=spin`（直接 launch）；脚本入口需要 `--yaw-mode spin`，但脚本已把它改写成 xy_only，**只能直接 launch 做 A/B** |
| 完全回退 | 上述两项一起，并把 `speed_limits.yaml` 的 `velocity_smoother@fakeyaw` 段删掉（恢复 `@minco` 的 yaw=8.5） |
