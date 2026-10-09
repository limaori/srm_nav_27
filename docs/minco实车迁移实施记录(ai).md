# MINCO + MPC 实车迁移实施记录与上电验收清单

> 依据：[迁移minco实施方案(ai).md](迁移minco实施方案%28ai%29.md)（下称“方案”）与
> [minco迁移实施记录(ai).md](minco迁移实施记录%28ai%29.md)（下称“仿真记录”，§6.1–§6.8 是仿真侧已经踩过的坑）。
> 本次范围：**把仿真里跑通的 MINCO + MPC 接到实车链路，并给出分阶段上电验收清单**（方案 §10 的 P5）。
> 状态标记：【已实现】代码/配置完成；【未验证】没有实车跑过；【未实施】本次没做；
> 【待辨识】必须靠实车测量才能定值。
>
> **一句话结论**：链路本身（换参数文件 + 关闭 fake_vel_transform）已经打通，启动前会自动预检；
> 但**参数里仍有两项是仿真标定值**（`mpc.command_lookahead`、`safety.braking_deceleration`），
> 在按 §6 做完辨识之前，本文不承诺任何实车性能，也不建议超过 1.5 m/s。

## 1. 交付物总览

| 位置 | 作用 | 状态 |
|---|---|---|
| `script/start_real_nav.sh` | 实车唯一入口：新增 `--controller auto\|omni\|minco`、MINCO 预检、工作区一致性检查、MINCO 专属自检输出 | 【已实现】 |
| `src/.../srm27_nav_bringup/scripts/srm_minco_real_preflight.py` | 上电前的**只读**参数预检（不需要 ROS）：插件名 / use_sim_time / 坐标系 / 里程计频率与 `state_timeout` / 制动走廊 / 三层限速 / 到点几何 | 【已实现】，可用 `--check` 之外的独立命令复跑 |
| `src/.../srm27_nav_bringup/test/test_minco_real_preflight.py` | 预检自身的回归测试（14 个用例）：合成配置逐条制造“应该被拦”的情形，并断言**随仓库发布的实车 MINCO 配置必须零阻塞** | 【已实现】，`colcon test --packages-select srm27_nav_bringup` 通过 |
| `src/.../srm27_nav_bringup/config/real/nav2_params_srm_minco.yaml` | 实车 MINCO 参数：修正 `state_timeout`，补齐文件头说明与待辨识标注 | 【已实现】，数值【待辨识】 |
| `src/.../srm27_nav_bringup/CMakeLists.txt` | 安装预检脚本到 `lib/srm27_nav_bringup` | 【已实现】 |
| `src/.../srm27_minco_controller/{include,src}/terminal_stop.*` | 终点急停状态机：一进目标检查器容差就立即输出零速，不再追踪末点 | 【已实现】+ 10 个单元测试 |
| `src/.../srm27_minco_controller/test/test_terminal_stop.cpp` | 急停边界 / 滞回 / 坏样本 / 停车几何算术 | 【已实现】 |
| `src/.../srm27_nav_plugins/*goal_checkers/omni_stopped_goal_checker.*` + `goal_checker_plugins.xml` | 到点判定：位置 + 航向 + **平动**停稳，角速度不参与 | 【已实现】+ 10 个单元/插件测试 |
| 本文 §4.6–§4.10 | 2026-10-09 14:00 实车终点往返的成因与修复（终点急停、会话重置、`reaction_latency` 解耦、过期 twist、到点判定去角速度） | 【已实现】，实车复测【未验证】 |
| 本文 §6 | 分阶段上电验收清单 + 中止条件 + 回退 | 清单本身【已实现】，执行【未验证】 |

**没有新增 `nav_srm_real_launch.py`。** 方案 §P5 原本拟新增这个入口，但实车链路实际走的是
`start_real_nav.sh` → `nav2_stack_launch.py`，而后者**与控制器无关**（只把 `params_file`
透传给各节点）。再写一份实车 launch 会造出第二个入口，与仓库自己的审阅门槛
（“参数唯一”“避免多套互相覆盖的入口”）冲突。所以本次改为**换控制器 = 换参数文件**，
入口保持唯一，并把这层选择显式化成 `--controller`。这条偏离在 §8 里明确记录。

## 2. 仿真链路 vs 实车链路

迁移出问题的地方几乎都在“两侧不一样但名字一样”的参数上，所以先把差异摆开。

| 环节 | 仿真（已跑通） | 实车 | 差异的影响 |
|---|---|---|---|
| 局部控制器 | `srm27_minco_controller::MincoMpcController` | 同一个插件，靠 `--controller minco` 选中参数文件 | 默认参数文件是 Omni；不显式选就**根本没换控制器** |
| 机器人参考系 | `base_link`（真实随底盘运动） | 同上 | — |
| `fake_vel_transform` | **关闭**（`nav_srm_simulation_launch.py` 传 `use_fake_vel_transform: False`） | 旧 Omni 链路默认**开启**；MINCO 模式现已默认关闭 | 见 §4.2；不关闭会给平移速度叠一次多余的 `R(-yaw)` |
| 速度出口 | `cmd_vel_nav` → `srm_cmd_mux` → `cmd_vel_sim` → Gazebo 速度插件 | `velocity_smoother` → `cmd_vel_chassis` → `srm27_nav_protocol` → 串口 | 实车没有 mux，导航角速度由 smoother 钳 0 |
| 限幅点 | mux（`v_max`） | smooth（`max_velocity [1.5,1.5,0.0]`）+ 串口（`max_vx/vy 2.5`、`max_wz 1.0`） | 任何一层留低都会静默钳住上游 |
| 平移上限 | **1.5 m/s**（2026-10-09 的"提速档 3.0"已撤销，两端现已一致） | **1.5 m/s**（与实车 Omni/smooth/串口对齐） | 撤销原因：3.0 m/s 下切内弯的横向偏移量超过行为树 `RemovePassedGoals radius=0.35`，途经点删不掉 → 3 Hz 重规划把路径绕回去 → 车在途径点之间来回跑（见 `config/simulation/nav2_params_srm_minco.yaml` 文件头 / `srm_chassis_control.yaml` 的限幅注释） |
| `/odometry` 来源 | `simulation_ground_truth_odometry`（真值适配器） | `sensor_scan_generation` | — |
| `/odometry` 频率 | **50 Hz**（`publish_period_ns = 20 ms`） | **≈10 Hz**（跟随雷达帧率） | **这是本次发现的关键阻塞项**，见 §4.1 |
| MPC 自转权限 | `yaw_policy.mode = xy_only`，角速度上下界为 0 | 同上 | 导航不产生自转；实车目前也没有 `cmd_spin` / `rotation_*` 发布者 |
| 自转来源 | 独立链路 `rotation_test_sender → rotation_controller → rotation_velocity → srm_cmd_mux` | **没有对应节点**（`srm27_chassis_control` 只在仿真被 launch） | 实车自转只能来自下位机或行为树（`srm27_behavior` 的 `PublishSpinSpeed`） |

仿真与实车 MINCO 参数的逐项差异（由预检脚本自动列出）：

| 参数 | 实车 | 仿真 | 说明 |
|---|---:|---:|---|
| `planning_horizon` | 2.0 | 3.0 | 与各自局部代价地图尺寸（5×5 / 8×8）成比例 |
| `limits.max_linear_speed` | 1.5 | 3.0 | 实车按既有链路对齐 |
| `minco.terminal_speed` | 0.0 | 0.8 | 实车更保守：每个 2 m 局部视野都以“停住”收尾 |
| `state_timeout` | **0.25** | 0.10 | 本次改动，见 §4.1 |
| `mpc.command_lookahead` | 0.20 | 0.20 | 两边都是**仿真标定值**，【待辨识】 |
| `safety.braking_deceleration` | 0.0 | 0.0 | 都未填，退化为 `limits.max_linear_accel`，【待辨识】 |

> 结论：**“仿真跑通”只说明算法链和停止语义成立，不说明实车参数可用。**
> §6 的阶梯就是用来把这个差距一步步量出来的。

## 3. 实车链路（本次改动后）

```text
                                                          ┌─ (Omni 模式) cmd_vel_nav2_result → fake_vel_transform ─┐
controller_server ─cmd_vel_controller─► velocity_smoother ─┤                                                        ├─► cmd_vel_chassis ─► srm27_nav_protocol ─► /dev/ttyACM0 115200 ─► C 板
                                                          └─ (MINCO 模式) 直接作为出口 ──────────────────────────┘
```

* **MINCO 模式默认不经 `fake_vel_transform`**，`cmd_vel_nav_topic = cmd_vel_chassis`。
* `behavior_server` / `bt_navigator` 的恢复行为各自把 `cmd_vel` remap 到同一出口，
  所以 Spin / BackUp 也直达底盘（本来就不经 smoother）。
* 现在链路里**唯一的软件限幅点**是 `velocity_smoother`（1.5 m/s、3.0 m/s²、yaw 钳 0），
  之外是串口层的 `max_vx/vy 2.5`、`max_wz 1.0`。

启动命令：

```bash
# 默认（Omni，与改动前完全一致）
./script/start_real_nav.sh

# MINCO（自动选 MINCO 参数文件、自动关闭 fake_vel_transform、自动跑预检）
./script/start_real_nav.sh --controller minco

# 只看将要执行的命令，不启动任何进程
./script/start_real_nav.sh --controller minco -n
```

## 4. 本次定位并处理的问题

### 4.1 【已修复】实车 `/odometry` 是 10 Hz，而 `state_timeout` 是仿真 50 Hz 下的标定值

`state_timeout` 在代码里被**两处**使用（`minco_mpc_controller.cpp:113` 与 `:441`）：

1. `StateAdapter` 的状态新鲜度门限：既看单调时钟的**接收**年龄，也看 ROS 时间戳的**采样**年龄
   （`state_adapter.cpp:171-179`），超过就直接判 `state unavailable` → 输出零速；
2. **制动模型的 `reaction_latency`**（`config.reaction_latency = state_timeout_`）。

仿真里它是 0.10 s，因为真值适配器固定 50 Hz 发布（周期 0.02 s），有 5 倍余量。
实车的 `/odometry` 是 `sensor_scan_generation` 发布的，而它是拿 `lidar_odometry` +
`registered_scan` 做 `ApproximateTime` 同步之后才输出，**频率等于雷达帧率**：

* `livox_ros_driver2.publish_freq: 10.0`
* `point_lio.mapping.lidar_time_inte: 0.1`

两处都指向 **10 Hz，周期 0.10 s**。于是 `state_timeout = 0.10` 时门限**不大于到达周期**：
每个 50 Hz 控制周期都有相当概率被判“状态过期”→ 输出零速；持续超过
`build_grace_period`(1.0 s) 就抛异常，现象是**走走停停 + BT 反复 recovery**。
它与仿真记录 §6.6.1 是同一类失败（“约 1 ms 就再次失败”），只是触发源不同。

处理：`state_timeout: 0.10 → 0.25`（2.5 倍到达周期，吸收 LIO 解算延迟与传输抖动）。

**这个改动方向是保守的，不是放宽**：因为同一个值就是 `reaction_latency`，调大它会让
验证器要求的制动前缀同步变长。代价是 `local_path` 在截断端点处要求的制动走廊
`v·t_react + v²/(2a) + res` 从 0.575 m 增到 0.80 m，5×5 m 局部地图下的有效局部视野
由约 1.55 m 缩到约 1.32 m。若实测发现视野不够，正确做法是加大 `planning_horizon`
或局部代价地图，**不是**把 `state_timeout` 调回 0.10。

预检脚本会持续盯着这个关系（门限 ≤ 到达周期 = 阻塞项），并顺带核对
`state.max_sample_gap` 是否也有足够余量。

### 4.2 【已关闭】实车 MINCO 链路不再串 `fake_vel_transform`

仿真的 SRM 入口明确写着“关闭 `fake_vel_transform`，删除 gimbal_yaw_fake 链路”，
但实车脚本的默认值是**开启**。对 MINCO 来说这一层有两个独立的坏处
（`fake_vel_transform.cpp:95-157`）：

1. **多余的二次旋转**：`transformVelocity()` 对 `(vx, vy)` 施加 `R(-yaw)`
   （`aft_tf_vel.linear.x = vx·cos(yaw) + vy·sin(yaw)` 等），它假设输入是
   “与 odom 对齐的虚拟底盘系”。而 Nav2 控制器（含 MINCO）输出的**本来就是 `base_link` 系**。
   只有底盘 `yaw ≡ 0` 时该旋转才是恒等变换；一旦车头方向非零，平移方向会被**静默转错**。
   旧链路之所以“没出事”，是因为 `enable_rotation: false` + `xy_only` 下机器人几乎不自转、
   `yaw` 长期停在 0 —— 这是巧合，不是设计。
2. **同步分支失效**：它的 `syncCallback` 依赖 `local_plan` 话题来判断“控制器是否活跃”，
   而 `local_plan` **只有 Omni 控制器发布**（`omni_pid_pursuit_controller.cpp:164`）。
   MINCO 插件不发布该话题 ⇒ 节点永远停在“控制器不活跃”分支，每帧按当前 odom yaw 直发，
   行为与设计意图不一致（且 `current_robot_base_angle_` 的更新时机随 odom 抖动）。

处理：`--controller minco` 时默认 `USE_FAKE_VEL_TRANSFORM=0`，出口对准 `cmd_vel_chassis`。
显式加 `--fake-vel-transform` 仍可恢复旧链路，但那会打印一条列出上述两点的警告。
`--controller omni`（含默认行为）**完全不变**，Omni 链路与回退不受影响。

### 4.3 【已修正】安全文档与代码相反

`start_real_nav.sh` 原文写“**源码中没有超时清零逻辑**，所以发命令的节点退出后车不会自己停”。
代码事实相反：`srm27_nav_protocol.cpp` 的 `fillSpeedVector()`（`:707-738`）里有 0.5 s 看门狗
（`cmd_timeout_sec`），超过就把控制量归零，并且启动即发零速帧。
`start_real_slam.sh:276` 的说法才是对的。

这条被改成准确表述，并补上真正危险的那一半：

* 看门狗**只在节点活着时**有效。进程被 `kill -9`、主机崩溃、串口掉线时它没机会归零；
* C 板固件自己**没有超时保护**（源码注释 `:782-783` 明写），会一直用最后一帧的速度；
* `srm27_nav_protocol` 里**没有任何急停/使能 service**，软件层面唯一的停车手段是在
  `/cmd_vel_chassis` 上发零速度 `Twist`。

这三条现在直接印在脚本的启动前后提示里。

### 4.4 【已修复】到点几何：实车与仿真统一为 0.50 m

预检脚本一开始在仿真配置里发现一处不自洽：

| 配置 | `terminal_reached_radius` | `xy_goal_tolerance` | 当时结论 |
|---|---:|---:|---|
| 实车 | 0.20 | 0.40 | 自洽 ✅ |
| 仿真 | 0.20 | **0.15** | **不自洽** ⚠ |

车在离目标 0.15~0.20 m 处会被插件判成 `already_at_goal` 并输出零速（**不算失败**），
但目标检查器的容差是 0.15 不接受，随后的进度检查器（0.5 m / 10 s）会把目标判失败
→ BT 进入 recovery。这与仿真记录 §6.7.2 描述的现象互为表里。

**修复**：把实车与仿真的 `xy_goal_tolerance` 统一为 **0.50 m**（本次改动），
上面的不自洽随之消失（`0.20 ≤ 0.50`），预检里那条阻塞项不再出现；
实车与仿真的到点行为也变成同一套数。

这个数值同时是**终点急停的进入半径**（`terminal.tolerance: 0.0` 会跟随它），
以及必须满足的两条约束：

| 约束 | 当前 |
|---|---|
| `xy_goal_tolerance` > 行为树航点跳过半径 | `0.50 > 0.35` ✅ |
| `xy_goal_tolerance` ≥ `terminal_reached_radius` | `0.50 ≥ 0.20` ✅ |

### 4.5 顺带核对、结论是“无需改动”的项

| 检查 | 结论 |
|---|---|
| 规划系与代价地图系 | `planning_frame: odom` == `local_costmap.global_frame: odom` ✅（插件不做该校验，靠预检兜住） |
| `use_sim_time` | 实车 MINCO 文件里全为 `False` ✅ |
| 到点几何 | `terminal_reached_radius 0.20 ≤ xy_goal_tolerance 0.50` ✅ |
| MPC 预测窗口 | `N·dt = 30 × 0.02 = 0.60 s`，同时也是验证器的 `required_prefix_duration` ✅ |
| 制动走廊 | 要求前缀 `0.10 + 1.5/3.0 = 0.60 s`；2 m 起停剖面的最短时长 `2.0/1.5 + 1.5/3.0 = 1.83 s`，余量 67% ✅ |
| 三层限速 | `FollowPath.limits 1.5` ≤ `smoother 1.5` < 串口 `2.5` ✅ |
| 插件导出 | `srm27_minco_controller.xml` 正确导出 `srm27_minco_controller::MincoMpcController`，`libsrm27_minco_controller.so` 已安装 ✅ |

### 4.6 【已修复】终点振荡：一进目标容差就立即急停给零速

现场日志与源码快照：`log_diag/real_goal_20261009_140024/`（`REPORT.md`、`events.txt`、`runtime_params/`）。

**现象**：车以较快速度进入目标容差时判不了到点，控制器继续追精确末点，冲过末点再反向修正，
来回振荡；振荡中车被减速，速度终于低到满足停稳条件，才判成到达。

**直接原因（源码可查）**：`computeVelocityCommands()` 原来把传进来的 `_goal_checker` 直接丢掉
（`(void)_goal_checker;`），而 MPC 的代价里一直带着"到精确路径末点的位置误差"（`mpc.q_position = 20`）。
于是即使车已经进入 `StoppedGoalChecker` 的 0.40 m 容差（当时的值），控制器仍会把它往精确末点上推。
配置里的 `terminal_reached_radius = 0.20` **不是**独立的到点阈值：
`planning_worker` 只在"局部路径不足两点"时才用它；`at_goal_stop_` 也只在"没有可用轨迹"时起作用。

**为什么这对矛盾解不开**：容差是**位置**条件，而轨迹是**停在末点上**的停车剖面。
车进入容差圈时的速度是 `sqrt(2·a·tol)`：`tol = 0.40` 时约 1.5 m/s（几乎满速），`tol = 0.50` 时约 1.73 m/s ——
减速过程本来就发生在容差之内。所以任何"等它慢下来再接管"的门槛都等于永不接管
（早期一版就是这么写的，实测无效）。

**修复**：`TerminalStop`（`include|src/terminal_stop.{hpp,cpp}`，10 个单元测试）。
逻辑刻意做到最简单：**进容差 → 立即输出零速，没有任何后段控制**。

| 环节 | 规则 |
|---|---|
| 进入 | 位置进入目标检查器 `getTolerances()` 给出的 xy 容差（实车与仿真统一为 **0.50 m**）；**只看位置，不看速度** |
| 接管后 | 输出零速（`stopCommand(..., "terminal_stop")`），诊断里 `command_owner = terminal_stop` |
| 不追踪 | 不再输出任何朝末点或背向末点的速度 ⇒ 控制器**不可能再主动把车带出容差**，振荡在逻辑上不会出现 |
| 解除 | 距末点超过 `tolerance + exit_margin`（0.40 + 0.15 = 0.55 m）；另外换目标 / 定位重置 / 激活失活时清空 |
| 保留的校验 | 已提交轨迹的复检、规划请求提交都照常执行 |

**代价（必须如实记录）**：急刹的滑行距离**完全由底盘自己决定**，控制器不再限速。
车在距末点 `tol` 处被给零，滑行 `v²/(2·a_chassis)` 后停下。容差对末点是**对称的**，所以
"停车点仍落在末点 ±tol 内"的条件是 `v²/(2·a_chassis) <= 2·tol`，等价于

```text
a_chassis >= v_entry² / (4·tol)
```

实车代入（`v_max = 1.5`、`tol = 0.50`）：**底盘至少需要 1.12 m/s²**（容差从 0.40 放宽到 0.50 后，这条要求也随之下调）。
低于它就会滑出容差、再回来。预检脚本会把这条数字直接报出来（见下）。

**想减小急刹冲击的正确做法**：把 `terminal.tolerance` 调小 —— 进入越晚、速度越低：
`tol = 0.50` 时进入速度约 1.73 m/s（实车被 1.5 m/s 限速钳住），`tol = 0.15` 时约 0.95 m/s，`tol = 0.05` 时约 0.55 m/s。
**不要**重新引入减速曲线。

**预检脚本里关于这条的诚实说明**：本来想把 `v²/(2·a) <= 2·tol` 做成阻塞级检查，但发现它
**结构上不可能失败** —— 轨迹停车剖面用的是 `a_plan = min(max_linear_accel, braking_deceleration)`，
而急刹能力的估计也取 `braking_deceleration`，两者同源，于是滑行量恒 ≤ `tol`。
真正会出事的是**声明的那个值本身不准**（它现在还是占位的），那只能靠实测解决。
所以检查器只报出"底盘至少需要多少减速"这个可执行数字，并在 `braking_deceleration` 未填时警告，
而不是放一个永远不会触发的阻塞项充数。

观测：`diagnostics` 新增 `terminal_distance`、`terminal_active`、`terminal_tolerance`，
以及 `command_owner = terminal_stop` / `stop_reason = terminal_stop`。

### 4.7 【已修复】会话重置阈值 1 mm → 0.10 m，并补上原因日志

`setPlan()` 原来用 **1 mm** 判定“末点变了”，比全局规划器每次刷新的末点抖动（Theta* 的栅格量化本身就有厘米级）还小。
于是同一个任务在接近终点时被判成“新目标”，清空轨迹、热启动与重规划状态 ——
日志里两次 “Reached the goal” 之前各多出一次 `新导航会话 #N` 就是这么来的。

修复：新增 `goal_change_tolerance`（默认 0.10 m），并让会话重置日志带上**原因、末点坐标与坐标系**：

```text
MincoMpcController: 新导航会话 #2 (路径版本 39, 原因: path endpoint moved 0.031 m; 末点 1.560, -0.100, frame=map)
```

正常的路径刷新只打 DEBUG（"路径刷新 (版本 N)，保留当前轨迹 (末点移动 0.0031 m, 阈值 0.100 m)"），
所以现场区分“换目标”与“规划器抖动”不再需要靠猜。

同时修正一处顺序缺陷：`at_goal_stop_ = false` 原先放在会话/路径/限速版本校验**之前**，
于是任何成功结果（包括已经过期的旧会话结果）都会解除到点停车。现在改为通过版本校验之后才清。

### 4.8 【已修复】`reaction_latency` 与 `state_timeout` 拆开（终点短轨迹被拒的根因）

第二次任务的终点日志：

```text
terminal=global_goal | 有效前缀=0.2611s 需要=0.6s | 最小净空=1.207m 需要=0.38m
```

净空充足、速度很低，被拒的唯一原因是**时间覆盖**。验证器允许用“保持已校验的末点”补齐 MPC 窗口，
但要求轨迹的**真实总时长** ≥ `reaction_latency + v / a_brake`。而代码里
`config.reaction_latency = state_timeout`（两者共用同一个参数）。

§4.1 为了跟上实车 10 Hz 的 `/odometry` 把 `state_timeout` 从 0.10 提到了 0.25，
于是这条判据变成 `0.25 + v/3`：0.2611 s 的终点轨迹只允许 `v ≤ 0.033 m/s` 才可能通过。
**这是 §4.1 那次改动的副作用，两者本来是同一个参数，无法同时满足。**

修复：新增 `safety.reaction_latency`（`0` 表示退回 `state_timeout`，因此**仿真行为不变**）。
实车配置为 `state_timeout: 0.25`（状态新鲜度）+ `reaction_latency: 0.10`（控制链延迟）。

同时把失败日志补全，让这类失败不再只能靠猜（新增字段加粗）：

```text
… | terminal=global_goal terminal_speed=0 | 最小净空=… | 有效前缀=0.26s 需要=0.60s
| 当前速度=0.031m/s 制动需覆盖=0.26s 轨迹总时长=0.26s 末端速度=0.000 末端加速度=0.000 末端静止=true
```

最后三项直接区分“制动覆盖不足”与“末端没静止不能延拓”。

### 4.9 【已修复】过期 twist：不再把上一条有效速度当当前速度

`REPORT.md` §4 指出 `sensor_scan_generation.cpp:314` 在差分速度不可用时沿用上一条有效速度。
这条行为本身是为了避免“速度瞬间归零”的假象，但配合实车 10 Hz 的 `/odometry`
（日志里有 0.3/0.4/0.5 s 的间隔）就变成：**车其实已经停了，控制器与目标检查器仍读到 0.3 m/s 左右**。
后果是双重的：`StoppedGoalChecker` 迟迟判不了停稳；MPC 会为了消掉这个不存在的速度反向修正 ——
正好放大终点来回走。

修复的两处：

| 位置 | 改动 | 理由 |
|---|---|---|
| `sensor_scan_generation.max_sample_gap`（实车） | `0.20 → 0.60 s` | 让这些间隔给出**真实差分速度**而不是沿用旧值。位移差商的噪声正比于 `1/dt`，间隔越长噪声越小，原来的“长间隔放大噪声”顾虑对差商并不成立；真正要拦的是位姿跳变，由 `max_position_jump`/`max_yaw_jump` 负责。超过 0.60 s（暂停、定位重启）仍沿用旧值 |
| `FollowPath.state.max_sample_gap`（实车） | `0.20 → 0.60 s` | 同上，作用于插件的位姿差分回退路径。不会放大外推：外推窗口由 `state_timeout`(0.25 s) 单独封顶 |

**没有**把不可用的速度清零 —— 那会让 `StoppedGoalChecker` 在车还在动时判成功，方向更危险。

### 4.10 【已修改】到点判定不再依赖角速度（自建 GoalChecker）

原来的 `general_goal_checker` 用 `nav2_controller::StoppedGoalChecker`。它把 `wz` 和 `trans_stopped_velocity` 写在**同一组 AND 条件**里：

```text
平动速度 <= trans_stopped_velocity(0.03)  AND  角速度 <= rot_stopped_velocity(0.05)
```

配置层没有"只关掉角速度那一条"的开关。而 SRM 的情况是：

* 导航**不拥有**自转 —— 阶段一 `yaw_policy.mode = xy_only`（插件输出 `wz ≡ 0`），`velocity_smoother` 也把 yaw 上界钳成 0；哨兵的自转由下位机/独立链路负责；
* `yaw_goal_tolerance` 本来就是自由的（6.28 rad ≥ π），也就是"朝向不算到达条件"已经写在配置里了；
* 于是"自转中的哨兵永远判不了到点"是一个真实风险：导航既不产生也不控制 `wz`，却拿它否掉自己的到达结论。

**处理**：新增自建插件 `srm27_nav_plugins::OmniStoppedGoalChecker`，与 `StoppedGoalChecker` 的**唯一差别**就是去掉 `|wz| <= rot_stopped_velocity`：

| 判据 | 是否参与 |
|---|---|
| 平面距离 ≤ `xy_goal_tolerance` | 是 |
| 航向差 ≤ `yaw_goal_tolerance`（实车 6.28 = 不约束朝向） | 是（只作<u>位姿</u>比较） |
| 平动速度 `hypot(vx, vy)` ≤ `trans_stopped_velocity` | 是 —— 保证"到点"时车真的停住了，而不是高速掠过目标点 |
| **角速度 `wz`** | **否，完全不出现** |

`getTolerances()` 里角速度容差报成"未测量"（`lowest()`），与判定一致，而不是给一个假数值。

交付物：

| 位置 | 说明 |
|---|---|
| `src/.../srm27_nav_plugins/include|src/goal_checkers/omni_stopped_goal_checker.*` | 插件实现（`PLUGINLIB_EXPORT_CLASS`） |
| `src/.../srm27_nav_plugins/goal_checker_plugins.xml` | pluginlib 描述（`base_class_type="nav2_core::GoalChecker"`） |
| `src/.../srm27_nav_plugins/test/test_omni_stopped_goal_checker.cpp` | 10 个用例：`wz` 任意值不影响判定、平动仍必须停稳、位置/航向边界、非法参数被拒，外加 **pluginlib 按名字加载**（这条才是 `controller_server` 的真实加载路径，直接实例化通过不代表能加载） |
| `config/{real,simulation}/nav2_params_srm_minco.yaml` | `plugin:` 指向新插件；删除 `rot_stopped_velocity` 与 `stateful`（新插件无这两个参数） |

预检脚本新增两条**阻塞级**检查，防止改回去：`plugin` 必须是新插件、参数文件里不得残留 `rot_stopped_velocity`；
另有一条警告：`trans_stopped_velocity = 0`（等于不检查平动停稳）。

**未改动**：`config/real/nav2_params_srm.yaml`（Omni，A/B 对照与回退基线）仍用 `StoppedGoalChecker`。
它同样是自转不归导航管的底盘，是否一并切换由你决定 —— 改动方向是放宽条件，但会动到已验证的回退路径。

## 5. 待辨识清单（P5 的未完部分）

这些数值**现在都不是实测值**。预检脚本会把它们作为警告逐条列出。

| 参数 | 当前值 | 来源 | 为什么必须辨识 |
|---|---|---|---|
| `safety.braking_deceleration` | `0.0`（退化为 `limits.max_linear_accel = 3.0`） | 占位 | 它决定“有效前缀要多长才能停车”。填 0 等于**假设车能按加速上限刹车**；填小了会让每条候选轨迹被拒（实测 0.3 m/s² 时要求前缀 5.1 s，必然失败，见仿真记录 §6.5）。它同时是**安全参数**：乐观取值意味着轨迹按 0.375 m 刹停距离规划，实际刹不住就会冲出轨迹 |
| `mpc.command_lookahead` | `0.20 s` | 仿真标定 | 应等于“从下发命令到车真正动起来”的有效执行延迟（串口 100 Hz 重发 + 下位机解算 + 电机响应）。太小会复现速度层自锁（起步时 QP 最优解把加速后置，第 0 拍命令≈0，车不动，见 §6.4）；太大则过冲 |
| `safety.reaction_latency` | `0.10 s` | 按原配置反推的占位值 | 制动模型里的有效反应延迟。**必须与 `state_timeout` 拆开**：后者要跟上 10 Hz 里程计（≥0.25 s），前者只有 0.1 s 量级；混用会让终点短停车轨迹被一律拒绝（见 §4.6）。辨识方法：给 `/cmd_vel_chassis` 一个阶跃，量到 `/odometry` 速度起变化的时延，取中位数 |
| `limits.max_linear_accel` | `3.0 m/s²` | 抄自实车 smoother 既有值 | 它同时是**运动剖面的加速度**与输出变化率钳位。它决定了整车在 1.5 m/s 下的行为，但没有做过制动/加速辨识 |
| `safety.robot_radius` | `0.33 m` | 模型包络（车体 0.27 + 轮子外廓） | 不是实测外接圆。插件**只看 `LETHAL_OBSTACLE`(254)**，`local_costmap.inflation_radius` 对它没有影响，所以这个值就是唯一的碰撞几何 |
| `limits.max_angular_speed` / `max_angular_accel` | `1.0 / 0.5` | 占位（后者标注“实车无对应值”） | `xy_only` 阶段是死参数；一旦切 `follow_tangent`/SE(2) 立即生效 |
| MPC 权重 `q_*` / `r_*`，MINCO 权重 `w_*` | 同仿真 | 仿真调参 | `w_velocity/w_acceleration` 是在 `v_max 0.5 / a_max 0.3` 档位下测的，与现在生效的 `1.5 / 3.0` 不是同一工作点 |

辨识方法在 §6 的 S7。**在 S7 通过之前，不要提高限速，也不要把本文的参数值当作性能承诺。**

## 6. 分阶段上电验收清单

> 规则：**每一级通过才允许进下一级**。任一级出现“中止条件”就立即
> `./script/start_real_nav.sh --stop` + 物理急停，不要“再试一次看看”。
> 全程至少两人：一人操作、一人手放在急停上。
> **不需要 rosbag 就能判定大部分结论**：插件已经把终点状态直接放进 `/FollowPath/diagnostics`（`terminal_distance` / `terminal_active` / `terminal_tolerance` / `stop_reason` / `command_owner`），会话重置也带了原因与末点坐标。录 bag 只是可选的事后复盘手段。

### S0 离线预检（不接车、不上电）

```bash
cd /home/srm/srm_nav_27
# 1) 预检必须 0 阻塞
python3 src/srm27_navigation/srm27_nav_bringup/scripts/srm_minco_real_preflight.py \
    --params src/srm27_navigation/srm27_nav_bringup/config/real/nav2_params_srm_minco.yaml \
    --expect-minco
# 2) 启动解析必须正确选到 MINCO 文件且速度链路不经 fake_vel_transform
./script/start_real_nav.sh --controller minco -n | grep -E "局部控制器|速度链路|参数文件"
```

* 通过判据：预检 `0 个阻塞问题`；`局部控制器 = MINCO + MPC`；
  `速度链路 = ... velocity_smoother → cmd_vel_chassis (不经 fake_vel_transform)`。
* 中止条件：预检报阻塞、或解析出来的参数文件不是 MINCO 那份 → 先修配置再上电。

### S1 只起链路，不接底盘（车不上电/串口不插）

```bash
./script/start_real_nav.sh --controller minco --no-chassis --no-joy
```

在**新终端**里：

```bash
cd /home/srm/srm_nav_27 && source install/setup.bash
ros2 param get /controller_server FollowPath.plugin     # 必须是 MincoMpcController
ros2 node list | grep -i fake_vel_transform             # 必须无输出
ros2 topic hz /livox/lidar                              # ~10 Hz
ros2 topic hz /odometry                                 # ~10 Hz —— 记下实际值
ros2 run tf2_ros tf2_echo odom base_link                # LIO 里程计存在且连续
ros2 run tf2_ros tf2_echo map base_link                 # 完整 TF 链贯通
ros2 topic echo /local_costmap/costmap_raw --once | head -3
ros2 topic echo /terrain_map --once | head -3
ros2 lifecycle get /map_server                          # active
```

* 通过判据：
  1. `FollowPath.plugin` 是 `srm27_minco_controller::MincoMpcController`；
  2. 没有 `fake_vel_transform` 节点；
  3. `/odometry` 实测频率**填回预检的 `state_timeout` 判断**：
     `state_timeout(0.25) > 实测周期`。若实测只有 5 Hz（周期 0.2 s），把 `state_timeout` 提到 0.5；
  4. `odom → base_link` 与 `map → base_link` 都能查到，且**只有一个** `odom→base_link` 发布者
     （`ros2 topic info /odometry -v` 的发布者数应为 1）。
* 中止条件：`/odometry` 频率明显低于 5 Hz；或 TF 不贯通；或找不到 MINCO 插件。
* 此级**全程车不会动**（没有底盘节点），可以放心做。

### S2 底盘通电，但只验证“停得住”

```bash
./script/start_real_nav.sh --controller minco --no-joy      # 这次带上底盘串口
```

1. 确认串口存在（脚本会提示 `/dev/ttyACM0` 的权限问题）。
2. **看门狗验证**：链路上没有任何节点发速度时，`/cmd_vel_chassis` 应没有消息，
   而底盘应保持静止。手动发一次非零速度后立刻停掉发布：

   ```bash
   timeout 0.3 ros2 topic pub -r 20 /cmd_vel_chassis geometry_msgs/msg/Twist \
       "{linear: {x: 0.1}}"      # 只发 0.3 s
   # 等 1 s，车应当已经停住（0.5 s 看门狗兜底）
   ```

3. **急停验证**：按物理急停，确认底盘立刻断电/停止；再复位。
4. **零速出口验证**：`./script/start_real_nav.sh --stop` 之后确认底盘停住。

* 通过判据：第 2 步车在约 0.5 s 内停住（不是一直以 0.1 m/s 跑）；急停确实断力。
* 中止条件：看门狗无效（车持续跑）→ **不要继续**，先查串口固件与 `cmd_timeout_sec`；
  或急停不起作用 → 停止全部测试。
* 注意：这一步车会真的动 0.1 m/s 一小段，**必须在空旷处、有人扶车或垫空轮子的条件下做**。
  若无法垫空，请把 0.1 改成能在手里握住的更小值，或直接跳过第 2 步（但那样就少了一道保险）。

### S3 首动：低速档闭环（强烈建议，即使最终目标是 1.5 m/s）

首动不要用 1.5 m/s。用一份**临时副本**降速，不动仓库里的配置：

```bash
TMP=/tmp/nav2_params_srm_minco_firstrun.yaml
cp src/srm27_navigation/srm27_nav_bringup/config/real/nav2_params_srm_minco.yaml "$TMP"
sed -i 's/max_linear_speed: 1\.50/max_linear_speed: 0.40/;
        s/max_linear_accel: 3\.00/max_linear_accel: 0.60/;
        s/max_velocity: \[1\.5, 1\.5, 0\.0\]/max_velocity: [0.4, 0.4, 0.0]/;
        s/max_accel: \[3\.0, 3\.0, 0\.0\]/max_accel: [0.6, 0.6, 0.0]/;
        s/max_decel: \[-3\.0, -3\.0, 0\.0\]/max_decel: [-0.6, -0.6, 0.0]/;
        s/braking_deceleration: 0\.0/braking_deceleration: 0.6/' "$TMP"
./script/start_real_nav.sh --controller minco -p "$TMP" --no-joy
```

> 这里的 0.4 / 0.6 只是“能在一步内刹住”的量级，不是辨识结果；
> 它同时把 `braking_deceleration` 填成 0.6，让制动模型与降速档自洽
> （否则验证器仍按 3.0 m/s² 假设本体能力，属于乐观取值）。

操作：在 RViz 点一个**正前方 1.5~2 m** 的目标。

* 通过判据：
  1. 车**沿红线走**，不画龙、不来回；
  2. `ros2 topic echo /FollowPath/diagnostics --once` 里 `planning_result=success`、
     `minimum_clearance > 0.38`、`state_age` 与 `map_age` 都很小、
     `missed_deadline_count` 不持续增长；
  3. `maximum_speed` 不超过 `max_linear_speed`；
  4. `/cmd_vel_controller` 与 `/cmd_vel_chassis` 的 `wz` **恒为 0**；
  5. 到点后 `StoppedGoalChecker` 判定成功，BT 不进入 recovery；
  6. **终点不来回走**：`diagnostics` 的 `terminal_active` 在到点前变为 `true`，
     随后 `command_owner` 变为 `terminal_stop`、`requested_vx/vy` 归零，
     并且**不再出现任何非零命令**（不出现“冲过末点再反向修正”）；
  7. **没有多余的会话重置**：controller_server 日志里每个导航任务只应出现**一次**
     `新导航会话 #N`（任务开始时）。任务结束前再出现一次即为回归，看它的 `原因:` 字段。
* 中止条件（任一出现立即急停）：
  * 车朝**预期之外的方向**平移 → 先查坐标系/符号，不要调权重；
  * `planning_result` 持续非 success 且车不动超过 2 s；
  * `minimum_clearance` 掉到 0.38 以下；
  * 车出现明显往复或原地画圈。

### S4 符号与方向逐项确认（静止起步，用小位移）

在同一低速档下逐项确认。每次只动一项，动完停下：

| 目标位置（车体系） | 期望运动 | 要确认的约定 |
|---|---|---|
| 正前方 +x 0.8 m | 向前 | `vx` 正负号、里程计 x 方向 |
| 正左方 +y 0.8 m | 向左**平移**（机头不转） | `vy` 正负号；全向底盘横移 |
| 右后方 (−x, −y) | 向后右平移 | 组合符号 |
| 取消目标（RViz 里 Cancel） | **立即停住** | 取消语义；smoother/cancel 是否干净 |

* 通过判据：四项方向全对；取消后 0.5 s 内停住。
* 中止条件：任一方向反了 → **立即停止**，此时问题在坐标系或串口方向约定，
  不要去改 MINCO 权重。
  （已知的现场坑：`script/test_chassis_serial.py` 里记录了下位机有 90° 轴向交叉约定；
  先用那个脚本单独验证底盘方向，再回到导航。）

### S5 停止 / 取消 / 异常注入

逐项做，每项之间让车回到静止：

1. **取消目标**：期望 `cmd_vel_chassis` 归零、车停；
2. **杀掉定位**（`pkill -f point_lio`）：期望控制器在 `state_timeout`(0.25 s) 后判状态失效、
   输出零速，并在 `build_grace_period`(1.0 s) 后抛异常让 BT 走恢复；
   **车必须停住**；
3. **拔串口**：期望底盘按看门狗停（若串口先断，看门狗发不出去 —— 这时只能靠物理急停，
   所以这一项**必须有人手放在急停上**）；
4. **贴一张地图上没有的障碍物**在局部视野内：期望 `minimum_clearance` 保持 > 0.38，
   `minco_trajectory` 绕开或停车，**不撞**；
5. **人为把车搬动 1 m**（模拟定位跳变）：期望 `state.max_position_jump`(0.75 m) 触发里程计重置，
   轨迹作废、重新规划，而不是继续按旧轨迹走；同时 `terminal_active` 应被清空
   （定位跳变后“离终点多远”本身不再可信）；
6. **终点急停解除**：车停在终点急停状态时，人为把它推离终点超过
   `tolerance + exit_margin`（实车 0.40 + 0.15 = 0.55 m），
   期望 `terminal_active` 变回 `false` 并恢复跟踪，而不是一直输出零速。

* 通过判据：5 项的“车都停住 / 不撞”成立；日志里能看到对应的警告文本。
* 中止条件：第 2 项定位挂了车还在走；第 4 项撞上障碍 → 立即停测并回到 S0 复核参数。

### S6 连续导航与重规划（仍是低速档）

* 一条 5~10 m 的多点路径（用 `script/start_waypoints.sh` 或 RViz 连续下点），
  中途允许改目标；
* 通过判据：全程 `planning_result` 以 `success` 为主；`switched_trajectory_count` 合理增长；
  没有持续 recovery；末端停稳后目标判定成功；
* 中止条件：出现连续 recovery 循环（说明每一条候选轨迹都被拒），
  此时按 `diagnostics` 的 `reason` 与 `planning_result` 归因，**先看 reason 再改参数**。

### S7 辨识与提速（只有在 S3–S6 全通过后才做）

**S7.1 执行延迟辨识 → `mpc.command_lookahead`**

1. 让车静止，发一条**阶跃**速度命令（绕开导航，直接对 `/cmd_vel_chassis` 发 `vx=0.3`）；
2. 同时录 `/cmd_vel_chassis` 与 `/odometry`（`ros2 bag record`）；
3. 量“命令发出的时间戳”到“`/odometry` 速度首次达到 0.3 的 10%”之间的时间差；
4. 重复 5~10 次取中位数，得到有效执行延迟 `L`；
5. `command_lookahead` 设成与 `L` 同量级（不要超过 `mpc.prediction_steps × prediction_dt = 0.60 s`）。

判据：把 `command_lookahead` 从 0 逐步调到 `L`，同一个目标点的起步时间应明显缩短
（仿真里 0 → 0.2 s 使 1.0 s 内的位移从 −0.002 m 变成 0.325 m，见仿真记录 §6.4）。

**S7.2 制动辨识 → `safety.braking_deceleration`**

1. 在空旷直道上加速到目标档位速度 `v`，随后命令**满减速**（对 `/cmd_vel_chassis` 发反向/零速度），
   用 `/odometry` 记录从 `v` 到 0 的位移 `d`；
2. `a_brake = v² / (2d)`；
3. **取多次测量的最小值再打 7 折**作为 `braking_deceleration`——
   它必须是“可保证”的值，不是“最好的一次”；
4. 填进实车配置，然后重跑 S0 预检，确认制动走廊仍是“余量正常”。

判据：`0.25 + max_linear_speed / a_brake` 明显小于
`planning_horizon / v_max + v_max / max_linear_accel`（预检会直接给出这个比较）。

**S7.3 外形辨识 → `safety.robot_radius`**

实测车身外接圆半径（含轮子外廓、含所有凸出物），填进 `safety.robot_radius`。
注意它不是代价地图的 `robot_radius`——插件只用 `safety.*` 做碰撞判定。

**S7.4 提速**

只有在 S7.1–S7.3 都填了实测值、且 S3–S6 全通过之后，才把 `limits.max_linear_speed`
逐档提到 1.0 再考虑 1.5；每次提速都重跑 S0 预检并重做 S5 的第 2、4 项。

### 附：现场取证（任何一级都适用）

| 现象 | 第一手证据 | 判读 |
|---|---|---|
| 车不动，BT 不报错 | `ros2 topic echo /FollowPath/diagnostics --once` | `planning_result` + `reason` 直接给原因 |
| 车不动，插件也没输出 | `ros2 param get /controller_server use_sim_time`；`ros2 topic hz /odometry` | `use_sim_time` 必须 False；odom 频率必须 ≥5 Hz 且小于 `state_timeout` 的倒数 |
| 走走停停 | 同上，看 `state_age` | 定位/里程计问题，不是规划问题 |
| 规划一直失败 | `diagnostics.reason` | `validation_failed` / `no_path` / `optimize_*` 各有不同归因 |
| 车不沿红线 | RViz 同时显示全局路径(map)、`/FollowPath/planning_input_path`(odom)、`/FollowPath/minco_trajectory`(odom) | 三者形状一致 → 走跟踪/执行层；不一致 → 规划层 |
| 到点判定反复失败 | `diagnostics` 里是否 `already_at_goal`；`xy_goal_tolerance` vs `terminal_reached_radius` | 见 §4.4 |
| 终点附近来回走 | `diagnostics` 的 `terminal_distance`、`terminal_active`、`stop_reason`、`command_owner` | 见 §4.6：`terminal_active=true` 且 `command_owner=terminal_stop` 表示已急停。若一直 false 而 `terminal_distance` 已在容差内，说明位置没进容差或路径末点不是目标 |
| 会话被反复重置 | controller_server 日志里的 "新导航会话 #N (… 原因: …; 末点 x, y, frame=…)" | 见 §4.7：正常刷新应当只出 DEBUG 的 "路径刷新" |
| 实际速度顶在某个更小的值 | 依次 echo `/cmd_vel_controller`、`/cmd_vel_chassis` | 找哪一层在钳 |

```bash
# 观察终点状态（不需要 rosbag，Ctrl-C 结束）
ros2 topic echo /FollowPath/diagnostics | grep -E "terminal_|command_owner|stop_reason|planning_result"
```
```

## 7. 回退与 A/B 对照

| 目标 | 做法 |
|---|---|
| 回到改动前的实车行为（Omni + fake_vel_transform） | `./script/start_real_nav.sh`（默认即此，无需任何额外参数） |
| Omni 但关闭 fake_vel_transform | `./script/start_real_nav.sh --no-fake-vel-transform` |
| MINCO + 保留旧速度链路（对照实验） | `./script/start_real_nav.sh --controller minco --fake-vel-transform`（会打印风险警告） |
| 运行中紧急停 | 物理急停（第一手段）；或 `./script/start_real_nav.sh --stop`（先发零速再退进程） |
| 回到 Nav2 原生到点判定 | 把 `general_goal_checker.plugin` 改回 `nav2_controller::StoppedGoalChecker` 并补回 `trans_stopped_velocity` / `rot_stopped_velocity`（**预检会因此报阻塞**，这是有意设计的：那条路会让自转中的哨兵判不了到点） |

A/B 对照建议：同一张图、同一组起终点，在 **MINCO** 与 **Omni** 下各跑一遍，
录同一条话题清单，比较 `FollowPath/diagnostics` 与 `cmd_vel_chassis`。
MINCO 侧的 `diagnostics` 是 Omni 没有的额外证据，这是迁移的主要收益之一。

## 8. 与方案的偏离、以及“本次没做”的部分

**偏离**：

1. 没有新增 `nav_srm_real_launch.py`（理由见 §1）：换成 `--controller` 参数选择参数文件，
   保持实车入口唯一。
2. 没有把首发限速降到 0.5 m/s 作为**默认配置**（按你的选择沿用 1.5 m/s），
   但 §6 S3 把降速闭环作为**建议的首动档位**给出，并且是一条可复制粘贴的临时副本命令，
   不改仓库配置。

**没做（明确记录，不当作已完成）**：

1. **P4：SE(2) / `NAV_SE2`**。`yaw_policy.mode` 仍是 `xy_only`，MPC 没有自转权限；
   `spin` / `narrow_track` 两条路径在当前构建下产生零角速度
   （`setSpinRequest()` / `setNarrowTrackActive()` 没有调用者）。
2. **实车自转链路**。`srm27_chassis_control`（`srm_cmd_mux` / `rotation_controller` /
   `rotation_test_sender`）只在仿真被 launch；实车没有 `cmd_spin` 发布者。
   哨兵导航时的自转目前只能来自下位机或 `srm27_behavior` 的 `PublishSpinSpeed`。
3. **实数辨识**（§5 全部）。本批只提供流程与自洽性检查，没有也不可能在无车条件下给出实测值。
4. **下游实际执行命令反馈**。MPC 的“上一条命令”仍是插件自己返回的值，
   不是经 smoother/串口限幅后的真实执行值。
5. **局部代价地图里的 `NO_INFORMATION`(255) 是否真的出现**未确认。
   这决定 `safety.unknown_is_obstacle: true` 是否真的在起作用。
   现场验证：`ros2 topic echo --once /local_costmap/costmap` 后检查原始栅格里有 255。
6. **`planning_deadline_ms` 是死参数**（只做范围校验，不起作用）。
   真正的时间门槛是 `mpc.qp_deadline_ms`，而它**超过只计数不停车**。已写进配置注释。
7. **`cross_track_error` / `along_track_error` 在 `diagnostics` 里恒为 0**（从未赋值），
   不能拿它们做验收证据。
8. **本轮没有做终点行为的事后复盘录制**。终点状态已经直接进了 `diagnostics`
   （`terminal_distance` / `terminal_active` / `terminal_tolerance` /
   `command_owner` / `stop_reason`），会话重置也带了原因与末点坐标，
   因此判定上述修复是否生效不需要 rosbag；只有需要还原“每一次往返的实际轨迹”时才需要录。
9. **`sensor_scan_generation` 里“/odometry 与点云做 ApproximateTime 同步后才发布”这一结构没有动**。
   日志里的 0.2~0.5 s 间隔可能来自这一层同步，也可能来自上游 LIO 丢帧；本轮只把
   长间隔的**处理**改成给出真实差分速度（§4.9），没有改输出结构。
   要定位间隔来源，需要单独观测 `lidar_odometry` 与 `registered_scan` 各自的到达节奏。

## 9. 复跑命令速查

```bash
# 离线预检
python3 src/srm27_navigation/srm27_nav_bringup/scripts/srm_minco_real_preflight.py \
    --params src/srm27_navigation/srm27_nav_bringup/config/real/nav2_params_srm_minco.yaml \
    --expect-minco
# 预检自身的回归测试（17 用例，含“实车配置必须零阻塞”这条守门员用例）
python3 src/srm27_navigation/srm27_nav_bringup/test/test_minco_real_preflight.py
# 单元测试（终点急停 15 用例 + 到点判定 10 用例；本项目用 gtest，不是 pytest）
source install/setup.bash
colcon test --packages-select srm27_minco_core srm27_minco_controller srm27_nav_plugins
colcon test-result --verbose
# 解析检查（不启动）
./script/start_real_nav.sh --controller minco -n
# 上电运行
./script/start_real_nav.sh --controller minco
# 停车
./script/start_real_nav.sh --stop
```

关于 lint：`colcon test --packages-select srm27_nav_bringup` 里 `copyright` 的 2 个失败
（`launch/real_mapping_launch.py`、`launch/real_robot_state_publisher_launch.py` 缺版权声明）
与 `pep257` 的 D213/D301/D400/D406/D407/D413/D415 都是**本次改动之前就存在**的全仓库问题
（中文 docstring 的 D400/D415 尤其普遍）。本次新增文件按包内既有风格书写并带版权头，
**没有新增规则类型**，也没有改 lint 配置。
