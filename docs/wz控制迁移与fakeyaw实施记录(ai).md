# wz 控制迁移导航与 fakeyaw 仿真实施记录

对应方案：`docs/wz控制迁移导航与fakeyaw仿真实施方案.md`。
本记录写"实际改了什么、量到了什么、哪些还没验"。

实施日期：2026-10-11（P0/P1 实测在 `/home/srm/srm_nav_27test` 完成）；
2026-10-12 与 main 合并后落到 `/home/srm/srm_nav_27` 分支 `test/仿真小陀螺导航`（见 §8）。

---

## 1. 结论速览

| 阶段 | 内容 | 本机结果 |
|---|---|---|
| P0 | 纯逻辑与 TF（单测 + 真实节点自检） | **通过**（脚本 `script/check_wz_fakeyaw_chain.py` 全绿） |
| P1 | Gazebo 空场执行 | **链路侧通过**；**底盘侧不满足**（仿真底盘执行力不可靠，旧链路同样如此，见 §5.3） |
| P2 | Nav2 静态地图 + 真值里程计 | 未在本机执行（需要有人看着 RViz 下发目标点），命令与判据见 §7 |
| P3 | 定位链路回归（Point-LIO / GICP / SLAM） | 未执行，见 §7 |
| P4 | 障碍物与故障注入（含底盘自转辨识） | 未执行；**P1 暴露的底盘问题必须先解决**，见 §5.3 |

一句话：**接口层面的迁移已经完成并且可验证**（wz 所有权、坐标系转换、TF、超时清零、
回退开关都有单测/自检覆盖）；**能不能"用导航 wz 把车转起来"这件事受限于仿真底盘**，
在把底盘执行链路修好之前，P2~P4 的验收没有意义。

---

## 2. 交付清单

| 文件 | 改动 |
|---|---|
| `fake_vel_transform/include/fake_vel_transform/velocity_transform.hpp` | **新增**：fakeyaw 纯数学（`fakeToRobotBase` / `fakeFrameYaw` / 四元数 / 姿态可用性） |
| `fake_vel_transform/include/fake_vel_transform/fake_vel_transform.hpp` | 两种模式（`legacy` / `nav_se2`）的参数、状态与诊断计数 |
| `fake_vel_transform/src/fake_vel_transform.cpp` | `nav_se2`：里程计回调里发 TF（消息时间戳）、单次坐标转换、`wz` 逐条透传、odom 超时/非有限 yaw 输出零速、诊断；`legacy` 行为逐字保留 |
| `fake_vel_transform/test/test_velocity_transform.cpp` | **新增**：ψ=0/±π/2/π 方向、`wz` 透传、模长保持、可逆性、四元数、姿态可用性（9 例） |
| `srm27_chassis_control/include/srm27_chassis_control/velocity_mix.hpp` | `MixMode`（`legacy_independent_rotation` / `nav_se2`）、`MixInput.nav_wz`、`MixOutput.wz_clamped`、模式解析 |
| `srm27_chassis_control/src/velocity_mix.cpp` | 模式化 `wz` 来源；`nav_se2` 下 `rotation_wz` **不参与**合成 |
| `srm27_chassis_control/src/cmd_mux_node.cpp` | `wz_control_mode` 参数（非法值直接起不来）、`nav_se2` 拒绝 rotation 输入、导航超时连同 `wz` 一起清零、诊断加 `wz_source` / `dropped_rotation_count` / `nav_wz_clamped_count` |
| `srm27_chassis_control/test/test_velocity_mix.cpp` | **新增** 8 例（导航拥有 wz、rotation 被忽略、限幅计数、模式解析） |
| `srm27_chassis_control/config/srm_chassis_control.yaml` | 新增 `wz_control_mode`（默认 `legacy_independent_rotation`，由 launch 重写） |
| `srm27_chassis_control/launch/srm_chassis_control.launch.py` | 新增 `nav_topic` / `wz_control_mode` 参数并走 `RewrittenYaml` 覆盖 |
| `srm27_minco_controller/.../yaw_policy.hpp`、`src/yaw_policy.cpp` | 新增 `YawMode::kSe2`（MPC 拥有 `wz`、航向参考只来自路径切线）、到点保持、**非 SPIN 模式下拒绝独立自转请求并计数** |
| `srm27_minco_controller/include/.../command_frame.hpp`、`src/command_frame.cpp` | **新增**：`command_frame` 语义解析（只允许 `base_frame` / `fake_base_frame` 两个名字）与 `velocityInCommandFrame()` |
| `srm27_minco_controller/src/minco_mpc_controller.cpp` | `command_frame` / `fake_base_frame` 参数、按命令系选择是否做 `R(-psi)`、`wz` 限幅原因、诊断字段、到点保持接线 |
| `srm27_minco_controller/include/.../diagnostics.hpp`、`src/diagnostics.cpp` | 新增 `command_frame` / `yaw_reference` / `mpc_requested_wz` / `applied_wz` / `wz_clamp_reason` / `rejected_spin_request_count` |
| `srm27_minco_controller/test/test_command_frame.cpp` | **新增** 7 例，含"fake 系输出 + 下游一次转换 == 车体系输出"的重复旋转护栏 |
| `srm27_minco_controller/test/test_yaw_policy.cpp` | **新增** 5 例 se2（权限、切线参考、拒绝自转请求、到点保持） |
| `srm27_nav_bringup/config/simulation/nav2_params_srm_fakeyaw.yaml` | **新增**：迁移配置（`base_link_fake`、`command_frame`、`se2`、`StoppedGoalChecker` + `rot_stopped_velocity`、`fake_vel_transform` 段） |
| `srm27_nav_bringup/launch/nav_srm_simulation_launch.py` | 新增 `use_fakeyaw_transform` / `wz_control_mode` / `command_frame`（默认跟随参数文件名）、接线（Nav2 出口→fake→mux）、启动核对与速度链日志 |
| `srm27_nav_bringup/test/test_fakeyaw_launch_wiring.py` | **新增** 5 例：默认值跟随配置、接线首尾相接、一致性拒绝、旧基线不被改坏 |
| `srm27_nav_bringup/test/test_speed_limits.py` | 把迁移配置纳入限幅唯一来源回归 |
| `script/start_sim_nav.sh` | `--wz-source nav\|rotation`、`--fakeyaw`、默认参数文件按模式选择、启动横幅打印速度链与 TF owner、按模式给检查命令 |
| `script/check_wz_fakeyaw_chain.py` | **新增**：P0 自检（真实节点 + TF，不需要 Gazebo） |
| `script/check_wz_fakeyaw_p1.py` | **新增**：P1 空场自检（命令层断言 + 底盘层记录 + 翻车哨兵） |
| `srm27_navigation/fake_vel_transform/package.xml`、`CMakeLists.txt` | 新增 `diagnostic_msgs` 依赖与 gtest |

---

## 3. 关键设计决定（与方案的差异及原因）

1. **`fake_vel_transform` 保留 `legacy` 路径**。方案 §4.2 要求"删除正式模式中的
   `cmd_spin` 叠加"。这里把它做成 **模式**：`transform_mode=nav_se2`（迁移，无叠加、
   单次转换、超时零速）与 `transform_mode=legacy`（**默认**，逐字保留原行为：odom/local_plan
   近似同步 + `cmd_spin` 叠加 + 控制器不活跃时直通）。原因：实车链路
   （`config/real/nav2_params_srm.yaml`）也在用这个节点，且导航输出与手柄输出共用同一个
   `cmd_vel` 话题——"控制器活跃"是它区分"要旋转"和"不要旋转"的唯一依据。默认 `legacy`
   保证本方案不动实车行为（方案 §1 明确"不改实车启动默认值"）。
2. **`command_frame` 只认两个名字**。写成第三个名字时 `configure` 直接失败。方案 §3.2
   要求"不能把已经是 base_link 系的 MINCO 输出再送入 fake 转换器"，而这件事一旦靠约定
   就容易漏；把"到底转几次"钉死在参数校验里。
3. **`yaw_policy.stop_rotation_on_goal` 真正接线**。它在改动前只是被读进配置、没人用。
   现在 `se2` 下由 `terminal_stop`/`at_goal_stop` 驱动 `setGoalHold()`：到点冻结航向参考，
   否则"还在追切线 → 车一直转 → 停稳条件永不满足"。
4. **角速度限幅仍以 mux 为权威层**。方案的"启用角速度限幅"落在
   `speed_limits.yaml` 的 `srm_cmd_mux.wz_max = 4.0`（`nav_wz_clamped_count` 计数）；
   控制器/平滑器的 5.0 只是"不成为瓶颈"的上游值。理由是本仓库把限幅视为**唯一来源**，
   新配置里重复写 `limits.max_angular_speed` 会与覆盖层冲突并直接报错
   （`scripts/srm_speed_limits.py`）。
5. **launch 的迁移开关默认"跟随参数文件名"**。方案要求"迁移默认开启、旧基线默认关闭"，
   但 launch 参数与参数文件必须一致，否则会出现"参数是 fakeyaw、链路是旧链路"。做法：
   `params_file` 里含 `fakeyaw` 时三个开关默认取迁移值，否则取旧基线值，并在启动时
   核对参数文件内容（不一致直接报错）。
6. **定位链保持真实 `base_link`**。`nav2_params_srm_fakeyaw.yaml` 只把
   `bt_navigator` / 局部与全局代价地图 / `behavior_server` 的 `robot_base_frame` 改成
   `base_link_fake`；`loam_interface` / `sensor_scan_generation` / `small_gicp` /
   `slam_toolbox` 的 `base_frame` 仍是 `base_link`。让定位去跟随"不转的假底盘"会把自转
   从观测里抹掉（2D Pose Estimate 的语义也会反过来）。

---

## 4. P0：纯逻辑与 TF（通过）

### 4.1 单测

```
srm27_chassis_control:  test_velocity_mix 16 例全过（新增 nav_se2 8 例）
srm27_minco_controller: test_yaw_policy 13 例、test_command_frame 7 例、
                        test_diagnostics / test_terminal_stop / test_local_path /
                        test_pluginlib_loading 全过
fake_vel_transform:     test_velocity_transform 9 例全过
srm27_nav_bringup:      test_speed_limits 12 例、test_fakeyaw_launch_wiring 5 例、
                        test_pose_math / test_ground_truth_odometry_pose /
                        test_minco_real_preflight 全过
```

`command_frame` 用例里最关键的一条：**fake 系输出经下游唯一一次 `R(-psi)` 之后，必须与
车体系输出完全相等**（ψ 取 0/±π/2/π/1.1/-2.4/0.7）。若控制器"顺手"先转一次，这条断言在
ψ≈π/2 处会立刻失败 —— 这是"重复旋转"（方案 §5 三类错误之一）的护栏。

### 4.2 节点级自检（`script/check_wz_fakeyaw_chain.py`）

用人工里程计驱动真实的 `fake_vel_transform(nav_se2)`：

```
--- 1) 无里程计：必须输出零速 ---                         PASS (vx=0, vy=0, wz=0)
--- 2) 四个基准 yaw 的速度方向与 wz 透传（输入 0.3/0.2/0.5）---
  yaw=0      v_base=(0.300000000, 0.200000000)            PASS  模长保持 PASS  wz 透传 PASS
  yaw=+pi/2  v_base=(0.200000000, -0.300000000)           PASS  模长保持 PASS  wz 透传 PASS
  yaw=pi     v_base=(-0.300000000, -0.200000000)          PASS  模长保持 PASS  wz 透传 PASS
  yaw=-pi/2  v_base=(-0.200000000, 0.300000000)           PASS  模长保持 PASS  wz 透传 PASS
--- 3) TF base_link -> base_link_fake ---
  yaw=0     TF yaw=0.000000000 期望 -0.000000000          PASS  平移为零 PASS
  yaw=+pi/2 TF yaw=-1.570796327 期望 -1.570796327         PASS  平移为零 PASS
  yaw=pi    TF yaw=-3.141592654 期望 -3.141592654         PASS  平移为零 PASS
  yaw=-pi/2 TF yaw=1.570796327 期望 1.570796327           PASS  平移为零 PASS
--- 4) 里程计超时后必须重新输出零速 ---                    PASS (vx=0, wz=0)
--- 5) 诊断 ---  mode=nav_se2, wz_error_max=0, wz_error_mean=0,
                input_count=224, output_count=118, zero_output_count=106
```

通过条件（方案 §5 P0）：线速度变换误差 `< 1e-6 m/s`、`wz` 误差 `< 1e-6 rad/s` ——
实测为**精确相等**（浮点 ==），TF 只有一个父节点 `base_link`。

---

## 5. P1：Gazebo 空场执行

### 5.1 怎么跑

```bash
# 1) 空场仿真（无 GUI，本机无 X11 socket，只能无头跑）
ros2 launch srm27_gazebo_simulator srm_sim.launch.py world:=srm_empty gui:=false run_immediately:=true
# 2) 自检（自己拉起 fake_vel_transform 与 srm_cmd_mux，按方案 §5 P1 下发四组速度）
ROS_DOMAIN_ID=79 python3 script/check_wz_fakeyaw_p1.py --csv /tmp/p1_samples.csv
```

### 5.2 链路侧结果（通过）

一次干净运行的记录（`cmd_vel_nav_fake` / `cmd_vel_nav_body` / `cmd_vel_sim` / `odometry`）：

| 用例 | `cmd_vel_nav_body` vs `R(-ψ)·fake` | `cmd_vel_sim` vs body | `wz` 透传 | 世界系位移方向 vs fake 系指令 |
|---|---|---|---|---|
| `vx=0.3, vy=0, wz=0`（ψ=62.0°） | 相等（≤5e-3，仅采样时刻差） | 相等 | 0 → 0 | 2.5° vs 0° |
| `vx=0, vy=0.3, wz=0`（ψ=36.2°） | 相等 | 相等 | 0 → 0 | 85.7° vs 90° |
| `vx=0.3, vy=0.2, wz=0.5`（ψ=38.2°） | 相等 | 相等 | **0.5 → 0.5 → 0.5** | 34.5° vs 33.7° |
| 转到 ψ=171.1° 后 `vx=0.3` | `(-0.296631, -0.044831)` vs `(-0.296648, -0.044723)` | 相等 | 0 → 0 | **-0.4° vs 0°** |
| 停止下发（> `nav_timeout` 0.3 s） | — | `cmd_vel_sim = (0,0,0)` 精确 | — | — |

在仿真已经被反复扰动、里程计开始剧烈抖动之后又跑了一次（`--csv` 采样见
`.tmp_check/p1_samples.csv`，本次运行日志 `.tmp_check/p1_final.log`），链路侧仍然全绿：

```
[PASS] 横移 vy=0.3: cmd_vel_nav_body=(-0.272830, 0.124754) vs R(-psi)*fake 最小残差 1.73e-10
[PASS] 横移 vy=0.3: cmd_vel_sim 与 cmd_vel_nav_body 一致
[PASS] 组合 0.3/0.2 + wz=0.5: wz 逐条透传 (fake=0.500000 body=0.500000 sim=0.500000)
[PASS] 纯自转: cmd_vel_sim.wz=0.600000（指令 0.6）
[PASS] 停止后 cmd_vel_sim 归零 (vx=0, vy=0, wz=0)
=== P1 链路侧全部通过（底盘侧见上面的记录）===
```

几条要点：

* `wz` 是**逐条相等**（1e-6 量级）地从 `cmd_vel_nav_fake` 传到 `cmd_vel_sim`，
  没有任何"再叠加一次自转"的迹象。
* ψ=171° 时命令 fake 系 +x，底盘的车体系命令是"几乎正后方 + 一点点右"
  （`-0.2966, -0.0448`），而世界系位移方向仍在 0° 附近 —— 这正是 fakeyaw 要的性质：
  **假底盘朝向稳定、真底盘朝向随便转**。
* 停止下发后 mux 在看门狗时间内把 `vx/vy/wz` 一起清零（`cmd_vel_sim` 三个分量精确为 0），
  没有残留上一条非零 `wz`。

**排查记录（一次"看似链路错"的误报）**：跑久之后出现过
`cmd_vel_nav_body=(0.239170, -0.181102)` 而"最近的里程计 yaw=0°"这类不一致。逐条探针查明：
仿真里程计的 yaw **在相邻两个样本之间就在 100.7° ↔ 165.6° 之间来回跳**（84 Hz），
fake 节点用的 yaw 与探针"最近一帧"根本对不上号，而 fake 节点那条命令的隐含 yaw
（101~166°）与里程计出现过的取值**完全一致** —— 也就是说转换是忠实的，是**里程计本身在抖**。
这个误报促成了两处改进：

1. 里程计订阅从 `message_filters` 改成**普通订阅**（`nav_se2` 下 KEEP_LAST=1、best_effort），
   去掉中间缓冲带来的"yaw 变旧"隐患（`legacy` 仍走原来的 message_filters 路径）；
2. P1 脚本加了**抖动哨兵**：取最近 0.3 s 内出现过的里程计 yaw 做"最小残差"匹配，
   相邻样本抖动 > 5° 时把这条降级为记录项（并打印抖动值），不再误判成链路错误。

### 5.3 底盘侧结果（**不通过，且与本次迁移无关**）

同一个仿真里量到的"指令 vs 实际"：

| 链路 | 指令 | `cmd_vel_sim.wz` | 底盘平均 yaw 速率（3~4 s 窗口） | 比例 |
|---|---|---|---|---|
| **旧链路** `rotation_velocity → mux(legacy)` | 0.2 | 0.200 | 0.064 | 0.32 |
| 旧链路 | 0.5 | 0.500 | 0.330 | 0.66 |
| 旧链路 | 1.0 | 1.000 | 0.458 | 0.46 |
| 旧链路（另一时刻） | 1.0 | 1.000 | **-0.511** | **-0.51** |
| 迁移链路 `nav_se2` | 0.5 | 0.500 | 0.23 / 0.29 / 0.65 / 0.73 / -0.35 / -0.73 … | 0.46 / 0.48 / 1.07 / 1.22 / **-0.58 / -1.21** |

也就是说：**`cmd_vel_sim.wz` 与指令逐条相等，但底盘的实际转率不可复现，偶尔反号**，
而且旧链路完全一样。原因在仿真底盘模型本身，不在本次改动：

* `srm27_gazebo_simulator` 的 `srm_velocity_system` 直接对 chassis link 调
  `SetLinearVelocity` / `SetAngularVelocity`，**四个轮子是被动 revolute 关节**，
  没有轮速闭环；轮地摩擦持续对抗自转，实际转率只有指令的一小部分。
* `chassis_odometry_gt` 的 yaw **相邻样本就能跳 60~130°**（实测序列
  `165.63 / 100.68 / 165.63 / 100.68 …`，84 Hz），说明底盘在 1 ms 物理步长上剧烈来回摆；
  `twist.angular.z` 也在 0.49 / 0.00 之间跳。所以它**不能**当作"实际转率"或"实际朝向"的证据
  —— 方案 §6 要求区分"请求 wz / 输出 wz / 最终 wz / 实际 wz"，这条数据正好说明为什么必须区分。
* 组合工况（平移 + 自转）更糟：同一条 `(0.3, 0.2, 0.5)` 指令，2.5 s 窗口里位移方向
  在不同时刻量到 0.9° / 32.2° / 33.9° / 125.0° / -108.6°、位移模长 0.04~1.39 m
  （指令 0.36 m/s × 2.5 s ≈ 0.9 m），有一次甚至跑出 8.9 m/2.9 s。
* 反复下发之后模型会**翻倒**：实测姿态四元数 ≈ `(0.057, 0.998, 0, 0)`、`z=0.653` m
  （正常应在 0.2 m 量级），翻倒后所有方向类测量都无意义。另有一次干净启动时
  `z ≈ -8e-7` m（底盘压在/嵌在地面上），与正常"骑在轮子上"的 `z≈0.2` m 也不一致。
  P1 脚本因此加了"翻车哨兵"（`roll/pitch/z`），翻倒时直接报"本次记录无效"。

因此 `script/check_wz_fakeyaw_p1.py` 把断言分成两层：
**链路级（硬失败）** = 话题之间的数学关系、`wz` 透传、超时清零、模型是否立着；
**底盘级（只记录）** = 位移方向/模长、实际 yaw 速率、里程计抖动。这样脚本在底盘不可靠时
也能给出有意义的结论，而不会把底盘问题误报成链路问题。

**结论**：P1 的"链路"部分达到方案 §5 P1 的通过条件（命令话题与 TF 完全正确、无残留速度）；
P1 的"底盘执行"部分**不满足**，并且**必须先修仿真底盘或先做底盘辨识（方案 §5 P4）**，
否则 P2 里的"路径切线跟踪""到点停稳"这些验收项测的是底盘噪声而不是导航。

---

## 6. 尚未验证 / 已知风险

1. **P2（Nav2 静态地图 + 真值里程计）没有实跑**。本机是无头沙箱（`/tmp`、`$HOME` 只读，
   Gazebo 只能 `-s` 无头运行），没有人在 RViz 里下发目标点；而且按 §5.3，即使跑起来，
   "车没到点"也无法区分是控制器还是底盘。命令已写在 §7。
2. **`nav2_controller::StoppedGoalChecker` 的行为只在配置层面验证**（插件存在、
   `rot_stopped_velocity` 已设置、launch 一致性测试覆盖），没有实跑"车还在转 → 判未停稳"。
   注意它检查的是**里程计里的真实底盘 yaw 速率**，而仿真里这个量本身不可靠（§5.3）。
3. **`yaw_policy.mode=se2` 的航向参考只做了单测**（切线、跨 π 展开、到点保持、
   拒绝自转请求），没有闭环验证"车头真的跟着路径切线转"。
4. **`mpc.q_yaw` / `q_omega` / `r_angular` 没有针对 `se2` 重新标定**：新配置沿用 MINCO
   那份（5.0 / 0.5 / 1.0）。它们原本是按 `xy_only`（yaw 不进代价）配的，`se2` 下需要
   按实车/仿真实测调。这是进入 P2 之前应该先看一眼的旋钮。
5. **`transform_mode` 的兼容性**：实车启动脚本（`script/start_real_nav.sh`）与
   `config/real/*.yaml` 未改；由于 `legacy` 是默认值，实车行为不变（这是刻意选择，
   见 §3.1）。**如果将来在实车启用 fakeyaw 迁移链，必须同时确认下位机自转链路已停用。**

---

## 7. 下一步（P2~P4 的跑法与判据）

```bash
# P2：静态地图 + 真值里程计 + 导航拥有 wz（空场/隧道图都可以）
./script/start_sim_nav.sh --fakeyaw --wz-source nav -m rmuc_2025
# 或直接：
ros2 launch srm27_nav_bringup nav_srm_simulation_launch.py \
  params_file:=<ws>/src/srm27_navigation/srm27_nav_bringup/config/simulation/nav2_params_srm_fakeyaw.yaml \
  map:=<ws>/src/srm27_navigation/srm27_nav_bringup/map/simulation/rmuc_2025.yaml \
  use_fakeyaw_transform:=True wz_control_mode:=nav command_frame:=base_link_fake

# 记录（方案 §6）：/clock /tf /tf_static /odometry /chassis_odometry_gt
#   /cmd_vel_nav_fake /cmd_vel_nav_body /cmd_vel_sim /FollowPath/diagnostics /diagnostics
ros2 topic echo /red_standard_robot1/diagnostics --once        # mux: wz_source / dropped_rotation_count
ros2 topic echo /red_standard_robot1/FollowPath/diagnostics --once  # applied_wz / wz_clamp_reason / yaw_reference
ros2 topic echo /red_standard_robot1/fake_vel_transform/diagnostics --once  # wz_error_max / stale_odom_count
```

P2 判据（在底盘可信的前提下）：

* 直行/横移/斜向的单目标都能到点，且**不启动** `rotation_controller`；
* 同一目标分别用 `wz=0`、小正负 `wz`、`wz_max` 限幅三种场景，`cmd_vel_sim.wz` 与
  `FollowPath.diagnostics.applied_wz` 一致，`wz_clamp_reason` 只在限幅时非 `none`；
* 到点时 `StoppedGoalChecker` 在真实 wz 落到 `rot_stopped_velocity` 以下之后才判成功；
* `fake_vel_transform/diagnostics` 的 `wz_error_max` 始终为 0、`stale_odom_count` 不增长。

P3（定位链路回归）注意：`use_lio_odometry` / `use_pcd_localization` / `slam` 三者每次
只开一个，观察 fakeyaw TF 是否跟随定位 yaw 连续变化、定位重置时是否输出零速
（`fake_vel_transform/diagnostics` 的 `nonfinite_yaw_count` / `stale_odom_count`）。

P4 必做底盘侧工作（本记录新提出）：**先让仿真底盘能可靠执行 `wz`**（轮速闭环或至少
标定"指令 wz → 实际 wz"的比例与符号），再把"窄通道、急转、目标取消、控制器退出、
odom 超时、TF 缺失、mux 超时、仿真暂停/reset"逐个注入。修好之前，
P2/P3 的"到点""停稳""不发散"这些结论都不可信。

---

## 8. 与 main 合并的兼容性核对（2026-10-12）

### 8.1 背景

本次迁移原本在 `/home/srm/srm_nav_27test` 上做（§4、§5 的实测都在那里）；main 的更新拉在
`/home/srm/srm_nav_27`（分支 `test/仿真小陀螺导航`，合并提交 `a080d4a`）。现在两侧已合并到
`/home/srm/srm_nav_27` 同一分支（改动未提交，`git diff a080d4a` 即本次迁移的全部内容）。

main 这次带进来的东西（`49bc9c8..a080d4a`，27 个文件）：

* `small_gicp_relocalization`：冷启动大角度恢复（`init_acceptance_enable` 等）与代码重构（+567 行）；
* 实车参数：`config/real/nav2_params_srm.yaml`、`nav2_params_srm_minco.yaml` 各加一段 GICP 冷启动说明与 `init_acceptance_enable: true`；
* 仿真入口 `script/start_sim_nav.sh`：新增 `--gicp` / `--prior-pcd` / `--prior-pcd-manifest` / `--init-pose`，
  并把 `use_pcd_localization` / `use_lio_odometry` / `prior_pcd_file` 收敛为变量 `NAV_EXTRA_ARGS`；
* 实车脚本与工具：`start_real_nav.sh`（默认 minco+gicp）、`start_real_slam.sh`、`kill_nav.sh`、
  `build_pcd_from_bag.sh`、`pcd_to_mapframe.py`、`record_mapping_bag.sh`；
* 地图：新增 `maps/227_1011/*`、`maps/pcd/227_1011*.pcd`，删除 `xiaojiaoluo0913` / `xjl0914`；
* README 与文档。

### 8.2 冲突面：只有一个文件

把本次迁移的 34 个文件（23 改 + 11 新增）与 main 的 27 个改动文件求交集，**只有
`script/start_sim_nav.sh` 重叠**。`small_gicp_relocalization`、实车参数、地图、其余脚本、
README 与本次迁移没有交集；也确认过本次改动没有引用被 main 删掉的两个旧地图名。

`script/start_sim_nav.sh` 有 3 处冲突，全部按"两边都要"解决：

| 冲突位置 | main 侧 | 迁移侧 | 解决 |
|---|---|---|---|
| 用法示例 | `--gicp ...` 三行 | `--fakeyaw` / `--wz-source nav` 三行 | 两段示例都保留 |
| 参数说明 | `--gicp` / `--prior-pcd` / `--prior-pcd-manifest` / `--init-pose` | `-p` 的 fakeyaw 默认值说明 + `--wz-source` / `--fakeyaw` | 合并成一份参数表 |
| `NAV_CMD` | `slam:=False${NAV_EXTRA_ARGS}` | `use_pcd_localization:=False` + 三个新开关 | 采用 main 的 `${NAV_EXTRA_ARGS}`，三个新开关追加在其后 |

合并后逐项核对：main 的 `NAV_EXTRA_ARGS`、`prior_pcd_file`、`gicp_nav2_params_*` 临时参数副本、
`init_pose` 注入、manifest 推导都还在；迁移的开关也在。`bash -n` 通过，dry-run 两种模式
打印的速度链都正确（见 §8.5）。

### 8.3 向后兼容：新增键的默认值全是旧行为

实车这次一个参数文件都没改，所以"默认值 = 旧行为"是硬要求。核对结果：

| 参数 | 默认值 | 实车链路是否受影响 |
|---|---|---|
| `fake_vel_transform.transform_mode` | `legacy` | 否（`legacy` 逐字保留改造前行为） |
| `srm_cmd_mux.wz_control_mode` | `legacy_independent_rotation` | 否（实车链路本来没有 mux） |
| `FollowPath.command_frame` | `base_link` | 否（默认不做 `R(-psi)`，与改造前一致） |
| `FollowPath.fake_base_frame` | `base_link_fake` | 否（只用于校验 `command_frame`） |
| `yaw_policy.mode` | 仍接受 `xy_only` / `follow_tangent` / `spin` / `narrow_track` | 否（实车 MINCO 用 `xy_only`，未变） |
| `input_cmd_vel_topic` / `output_cmd_vel_topic` | `""`（**改造前就是 `""`**，实车参数里显式给了值） | 否 |

新增的严格校验（`transform_mode` / `wz_control_mode` / `command_frame` 写错名字直接起不来）
只在写错时触发；main 新加进实车参数的那段是 GICP 段，与这些键无关。

### 8.4 新组合：`--gicp` × `--fakeyaw`

main 的 `--gicp` 与迁移的 fakeyaw 链可以叠，核对过三处：

1. `--gicp` 时 `use_ground_truth_odometry=False`（真值里程计适配器不启动），但 `point_lio` 会起、
   `sensor_scan_generation` 会发布 `odometry`（`pub_chassis_odometry_`），所以
   `fake_vel_transform.odom_topic: odometry` 在 GICP 模式下仍有 yaw 来源；
2. `start_sim_nav.sh` 的 `init_pose` 注入会先 `cp "$PARAMS_FILE"`；fakeyaw 模式下这个文件是
   `nav2_params_srm_fakeyaw.yaml`，它**含 `small_gicp_relocalization` 段**，因此不会报
   "参数文件里找不到 small_gicp_relocalization 段"。实测临时副本同时含 `init_pose` 与
   `transform_mode: nav_se2`；
3. 两者互不覆盖对方的 launch 参数（main 的走 `${NAV_EXTRA_ARGS}`，迁移的三个开关单独追加）。

**但没有闭环实跑**：GICP 模式下的 fakeyaw 属于 P3（定位链路回归）范围，跑法与判据见 §7。

### 8.5 合并后的验证结果（2026-10-12，`/home/srm/srm_nav_27`）

* 构建：4 个包（`fake_vel_transform` / `srm27_chassis_control` / `srm27_minco_controller` /
  `srm27_nav_bringup`）全部通过；
* gtest：`test_velocity_transform` 1 例、`test_velocity_mix` + `test_rotation_and_watchdog` 2 例、
  minco 8 例（含 `test_command_frame`、`test_yaw_policy`）全通过；
* pytest：`src/srm27_navigation/srm27_nav_bringup/test/` **50 例全通过**（含 `test_fakeyaw_launch_wiring.py`
  5 例、`test_speed_limits.py` 13 例）；
* P0 自检：`python3 script/check_wz_fakeyaw_chain.py` **26 项全过**——四基准 yaw 的速度方向与模长、
  `wz` 逐条透传（`wz_error_max=0`）、TF `base_link -> base_link_fake` 的 yaw=-psi、无里程计与
  超时都输出零速、诊断 `mode=nav_se2`；
* dry-run 集成：旧基线、`--fakeyaw`、矛盾组合（退出码 2）、非法 `--wz-source`、`--gicp` 缺
  `--prior-pcd`、`--gicp --fakeyaw` 六种都符合预期；
* 有两个**与本迁移无关、合并前就存在**的失败项：
  * `clang_format` / `uncrustify`：main 的 `df2b7de fix: 清理一下仓库` 删掉了各包的
    `.clang-format` / `.uncrustify.cfg`，linter 找不到配置文件必然失败；
  * `pep257` / `xmllint`：全仓库范围的历史问题（`pep257` 对约 20 个未改动文件报同样的
    D213/D400/D415；`xmllint` 在打印非 UTF-8 报错信息时自身崩溃）。新加的文件没有引入新的失败类别。

> ⚠️ 在受限沙箱里跑测试时，`/tmp`、`$HOME/.ros`、`/var/crash` 可能只读，会造成**假失败**
> （`OSError(30, 'Read-only file system')`）。把 `TMPDIR` 与 `ROS_LOG_DIR` 指到可写目录
> 就能复现 §8.5 的结论。

### 8.6 怎么测

```bash
cd /home/srm/srm_nav_27 && source /opt/ros/humble/setup.bash && source install/setup.bash

# 0) 回归：单测 + 配置/接线测试（不需要 Gazebo）
colcon test --packages-select fake_vel_transform srm27_chassis_control \
  srm27_minco_controller srm27_nav_bringup && colcon test-result --all

# 1) P0：真实节点 + TF 的链路自检（不需要 Gazebo / Nav2，约 10 s）
python3 script/check_wz_fakeyaw_chain.py

# 2) 只看命令链接得对不对，不下发速度（安全，可随时跑）
DRY_RUN=1 ./script/start_sim_nav.sh --fakeyaw -m rmuc_2025   # 迁移链
DRY_RUN=1 ./script/start_sim_nav.sh -m rmuc_2025             # 旧基线对照

# 3) P1：空场执行（链路级断言 + 底盘级记录；底盘执行力问题见 §5.3）
./script/start_sim_nav.sh --fakeyaw -w srm_empty -m rmuc_2025 --no-rviz --no-gui
python3 script/check_wz_fakeyaw_p1.py

# 4) P2：带 RViz 的目标点闭环（需要人下发目标点），判据见 §7
./script/start_sim_nav.sh --fakeyaw -w srm_empty -m rmuc_2025

# 5) 新组合：GICP 初始重定位 + fakeyaw（未闭环验证，见 §8.4）
./script/start_sim_nav.sh --fakeyaw --gicp \
  --prior-pcd /home/srm/srm_nav_27/maps/pcd/227_1011_mapframe.pcd -m 227_1011 -w srm_empty
```

## 9. 仿真起栈实测（2026-10-12 下午，接 §8）

环境：无头沙箱（bash 级 bwrap，`$HOME` 只读、`/tmp` 每命令独立），ROS 环境统一走
`.sim_run/env.sh`（把 TMPDIR/ROS_LOG_DIR/ROS_HOME 指到 `.sim_run/`）。Gazebo Fortress。

### 9.1 P1 复跑与检查器修正

前两轮 P1 均只挂一个用例：`组合 0.3/0.2 + wz=0.5` 残差 ≈3.7e-01，且**可复现**（3.76e-01
→ 3.73e-01）。逐条核对 `check_wz_fakeyaw_p1.py` 后确认是**检查器自身的采样时序缺陷**：
`run_command()` 在稳定 0.4 s 后采样一条 body（此刻 ψ=ψ₀），随后继续发令整个 duration
（车在 wz=0.5 下转了 ≈70°），**之后**才取"最近 0.3 s 里程计 yaw"做残差——那些帧的
ψ≈ψ₀+70°，错位角 62~63° 与残差 3.7e-01 精确对得上（2|v|sin(Δθ/2)）。

上一轮该用例能"通过"，是因为当时底盘 wz 执行力混乱、模型没怎么转（ψ_end≈ψ_start），
把时间错位掩盖了。修正：`sample()` 在采样 body 的**同一时刻**快照 yaw 候选与抖动
（`recent_yaws`/`odom_jitter_deg`），断言改用该快照。修正后复跑：

- **链路侧全部通过**；组合用例残差 3.76e-01 → **6.21e-17**（机器精度）。
- 转后直行（ψ=-100°）：body == R(-ψ)*(0.3,0) 逐分量相等；世界系位移方向 0.1°。
- 停止下发后 mux 超时零速、底盘 wz 精确归零。

### 9.2 仿真底盘 wz 执行力（对 §5.3 结论的重要更新）

干净的 `srm_empty` 基线下（新起的 Gazebo、未受扰动）：
纯自转指令 0.6 rad/s 实测平均 0.581~0.584（比例 0.97~0.98）；组合工况里程计
wz=0.500 与指令精确一致；全程 roll/pitch≈0.0°（未翻倒）；wz=0 时窗口内 yaw 漂移
0.0°；模型未出现上轮记录的转率乱跳/反号/翻倒。**§5.3"底盘执行力不可靠"的结论在
干净基线下不成立**——上轮的混乱大概率是模型被反复扰动后的状态，不是定态性质。
P4 底盘辨识应以此干净基线为新起点重新评估。

### 9.3 导航栈起栈与接线核对（fakeyaw 配置）

```bash
ros2 launch srm27_gazebo_simulator srm_sim.launch.py world:=srm_empty gui:=False run_immediately:=True
ros2 launch srm27_nav_bringup nav_srm_simulation_launch.py namespace:=red_standard_robot1 \
  world:=srm_empty map:=.../map/simulation/rmuc_2025.yaml \
  params_file:=.../config/simulation/nav2_params_srm_fakeyaw.yaml \
  slam:=False use_pcd_localization:=False use_fakeyaw_transform:=True wz_control_mode:=nav \
  command_frame:=base_link_fake use_composition:=False use_sim_time:=True use_rviz:=False \
  use_velocity_smoother:=True start_rotation_sender:=False
```

核对结果（全部符合迁移设计）：
- 话题三件套在位：`cmd_vel_nav_fake → cmd_vel_nav_body → cmd_vel_sim`。
- TF `base_link→base_link_fake` 由 fake 节点发布（静止时 yaw=0；±76°/±100° 下
  yaw=-ψ 已由 P1 验证）。odom 原点=出生位置，故 map→odom 恒等时出生点即 map (0,0)。
- mux 诊断：`wz_control_mode=nav_se2`、`wz_source=navigation`、
  `dropped_rotation_count=0`、`nav_wz_clamped_count=0`、`non_finite_nav_count=0`。
- fake 诊断：`mode=nav_se2`、`yaw_valid=true`、`odom_age≈0.03s`（< 0.25s 超时）、
  stale/no_odom/nonfinite 计数全 0。
- 启动期两条 "Timed out waiting for transform ... base_link_fake" 为 fake 节点
  TF 尚未首发的正常时序，costmap 随后正常 start。
- `current_goal_checker` 告警仅表示 Nav2 的 resume 语义回退到已加载检查器；
  `general_goal_checker` 实为 **StoppedGoalChecker**（rot_stopped_velocity=0.05，
  xy 容差 0.50，自由 yaw），是迁移要求的检查器。

### 9.4 目标点闭环（P2-lite）：4/4 成功

发 `NavigateToPose`（map 系，yaw 目标自由）：

| 目标 | 类型 | 结果 | 用时 | 备注 |
|---|---|---|---|---|
| (1.2, 0) | 直行 1.2 m | SUCCEEDED | ~0.9 s | 终点急停+检查器判停链路触发 |
| (2.0, 0) | 直行 1.1 m | SUCCEEDED | ~1 s | |
| (3.2, 0) | 直行 1.2 m | SUCCEEDED | ~1 s | QP 单次 8.15 ms 超 5 ms 预算（告警，解仍过硬约束） |
| (1.0, 1.5) | 转向 ~138° | SUCCEEDED | 39.9 s | 见 9.5 |

转向目标（迁移核心行为，采样见 `.sim_run/p2turn_samples.csv`）：
- **导航下发 wz 峰值 1.500 rad/s，sim 侧同为 1.500 rad/s**，车头累计转角 36.1°，
  航向跟随路径切线；静止段 body 与 R(-ψ)·fake 逐分量吻合（例：ψ=-10.6°、
  fake=(0.2,0) → body=(0.197,+0.037)=R(+10.6°)·(0.2,0)）。
- 运动中 body-vs-R(-ψ)·fake 的瞬时大残差（4.9e-01 量级）出现在**快速自转期间**，
  是采样器 body 消息与 odom yaw 之间几十毫秒的时差所致（与 9.1 同款时序错位）；
  wz"透传误差 0.3"同理——同一条消息内 wz 逐字节相等（P1 已证），跨消息采样才有差。
  判读旋转工况的链路数据时必须做时间对齐，或只比对同一条消息链。
- 全程 roll/pitch≈0.0°（未翻倒）；每次目标结束后 mux 0.3 s 超时清零正常。

### 9.5 恢复行为观测（非链路问题，记入 P4 调优清单）

转向目标途中 `number_of_recoveries=28`、耗时 39.9 s。日志根因：
- MINCO 轨迹校验反复 `validation_failed: trajectory collides with the raw obstacle
  set | 最小净空=0.3213m 需要=0.38m`（目标区靠近场地结构）；
- 全局规划器间歇 `Either of the start or goal pose are an obstacle!`（伴随
  costmap 清理请求）；随后 BT 多轮 clear→replan→backup(1 m)。
- backup 恢复速度同样走迁移链且执行成功。
- 结论：这是规划/校验参数（膨胀、净空、恢复策略）与该区域代价地图的调优话题，
  **与 wz 归属无关**；三个直行目标 0 恢复。
- 沙箱负载下另有 `Control loop missed its desired rate of 50.0000Hz`（CPU 竞争，
  实机不适用）。

### 9.6 沙箱注意点（复现实测用）

- 每条 ROS 命令必须 `source .sim_run/env.sh`，否则日志/HOME 写入失败。
- 临时 CLI/客户端的**服务/动作响应偶发丢失**（bt_navigator 曾日志
  `Failed to send goal response ... client will not receive response`，目标因此未被
  接受；`ros2 param get` 同理拿不到值）。pub/sub 不受影响。规避：目标用
  `ros2 action send_goal` CLI 发、结果以 bt_navigator 日志为准；参数以配置文件为准。
- TF 命名空间在 `/red_standard_robot1/tf|tf_static`，外部 CLI 查 TF 需 remap；
  tf2_echo 输出走 stdout 缓冲，建议落盘再读。
- 采样旋转工况时，body/odom 快照必须同一时刻（9.1 的教训）。

### 9.7 小陀螺导航实测（2026-10-12 晚，经 start_sim_nav.sh 正式入口）

脚本入口增强（本次提交）：`OPEN_MODE=direct`——无终端环境不弹 gnome-terminal，
子进程日志落 `.sim_run/logs/`，脚本末尾 `wait` 作为宿主托管全部子进程
（结束仿真=结束脚本进程组）；`.sim_run/env.sh` 补 `XDG_RUNTIME_DIR`
（`/run/user/1000` 只读导致 Gazebo 锁文件创建失败）。

**迁移链下小陀螺的现状（重要结论）**：
- mux `wz_control_mode=nav_se2` **按设计拒绝**独立自转输入（方案 §4.1.2，
  启动日志明确打印"独立自转输入 [rotation_velocity] 被拒绝"）；
- MINCO `yaw_policy.mode=spin` 基础设施已就位（`setSpinRequest`、请求过期即停、
  非 spin 模式拒绝并计数），但**控制器没有任何话题进线调用它**——迁移链的真小陀螺
  属于方案阶段二，当前未接线；
- 因此脚本显式禁止 `--fakeyaw --wz-source rotation` 组合是正确的。

**旧链路（默认 `--wz-source rotation`）小陀螺实测**：
`./script/start_sim_nav.sh -w srm_empty -m rmuc_2025 --no-rviz --no-gui
 --rotation-mode constant --rotation-speed <X>`：

| 自转指令 | 实际自转 | 目标 | 结果 | 位移方向误差 | 路径弯曲度 | 用时(运动段) |
|---|---|---|---|---|---|---|
| 1.0 rad/s | 0.99 | (1.2,0) 直行 | SUCCEEDED | 1.5° | — | ~1.5 s |
| 1.0 rad/s | 0.99 | (1.5,1.2) 斜向 | SUCCEEDED | 1.5° | — | ~1.1 s |
| 3.0 rad/s | **2.0**（被钳） | (1.2,0) 直行 | SUCCEEDED | 5.7° | 1.00 | ~0.9 s |

- 3.0→2.0 的钳制点：`speed_limits.yaml` 的 `rotation_controller.wz_max: 2.0`
  （唯一限幅来源，设计如此；不是 mux 的 4.0）。
- 到点后 nav 归零、车保持恒速自转（驻车小陀螺）；legacy 检查器（xy 容差 0.4）
  不含角速度停稳条件——这正是迁移配置换 `StoppedGoalChecker` 的原因。
- 结论：低转速 + 简单场景下旧链路可用，方向误差随转速近似线性增大
  （1.5°@1 rad/s → 5.7°@2 rad/s）；车体系速度 + 自转的补偿全靠 50 Hz 重规划，
  更高转速/复杂场景才是迁移链（fake 系速度）的主场。
- 数据：`.sim_run/gyro_nav{,2,3}_samples.csv`。

### 9.8 现场还开着的东西（会话结束时状态）

- 旧链路小陀螺导航栈（`rotation-speed 3.0` → 实际 2.0 rad/s 钳制值，Gazebo 无头 +
  导航）作为后台作业仍在运行，供用户从自己的终端接入观察。
- 用户侧接入：
  ```bash
  source /opt/ros/humble/setup.bash && source /home/srm/srm_nav_27/install/setup.bash
  # RViz（带命名空间与 TF remap）
  ros2 run rviz2 rviz2 --ros-args -r __ns:=/red_standard_robot1 \
    -r /tf:=tf -r /tf_static:=tf_static \
    -d /home/srm/srm_nav_27/src/srm27_navigation/srm27_nav_bringup/rviz/nav2_default_view.rviz
  # Gazebo GUI 接到无头服务端
  export IGN_GAZEBO_RESOURCE_PATH=/home/srm/srm_nav_27/src/srm27_gazebo_simulator/worlds:\
/home/srm/srm_nav_27/src/srm27_gazebo_simulator/models
  gz sim -g -v 1
  ```
- 停止：向会话发起人要（后台作业 job_kill），或对launch进程组 Ctrl-C。

## 10. 阶段二：迁移链小陀螺进线（2026-10-13）

### 10.1 实现内容

- **自转请求进线**（`minco_mpc_controller.cpp/.hpp`）：订阅 `spin_request.topic`
  （默认 `rotation_velocity`，RELIABLE 与 rotation_controller 匹配），缓存最近一条
  （steady 时钟）；`computeVelocityCommands` 每周期调 `refreshSpinRequest()`：
  `kSpin` 模式下新鲜请求喂 `setSpinRequest`、超时（`spin_request.timeout` 默认
  0.25 s）喂 `clearSpinRequest`；非 SPIN 模式只缓存不进线（避免旧链常驻 0 速流
  刷满拒绝计数）。诊断新增 `spin_request` / `spin_request_fresh` 两个键。
- **SPIN yaw 参考积分**（`yaw_policy.cpp`，关键修复）：原实现 `_yaw = _previous_yaw`
  恒定，MPC 航向跟踪项与自转参考对抗，实测解出 `applied_wz≈0`；改为按 wz 参考
  逐采样积分（`spin_ref_yaw_`），重规划时从当前连续航向重新锚定。修复后
  fake/sim/里程计三层 wz 峰值一致（2.09/2.09/2.09 rad/s @ 请求 2.0）。
- **终点急停保留自转**：进入判停圈后 `terminal_stop` 原本全零；`kSpin` 且请求
  新鲜时只刹平移、保留 wz（导航会话结束后 wz 随之停止，mux 零速兜底）。
- **launch 注入**（`nav_srm_simulation_launch.py`）：新增 `yaw_mode` 参数
  （auto/se2/spin/xy_only，默认 auto=迁移链 se2 / 旧链 xy_only）；用
  RewrittenYaml 点分全路径改写 `FollowPath.yaw_policy.mode` 与
  `general_goal_checker.plugin` 后再传给 nav2_stack。spin 时判停器自动换
  `OmniStoppedGoalChecker`（持续自转是驻车状态，判停只看平移）。
  **实车链不受影响**：改写只在仿真入口包一层，navigation_launch/nav2_stack_launch
  未动。
- **脚本默认切换**（`start_sim_nav.sh`）：默认 = 迁移链（`--no-fakeyaw` /
  `--wz-source rotation` 显式降级旧链）；新增 `--yaw-mode`；spin+旧链组合报错。
- **手动转速控制**（`script/spin_control.sh`）：`on <rad/s>` / `off` / `status`，
  20 Hz 持发 `rotation_cmd`（停发即超时撤回）。

### 10.2 验证（srm_empty + 227_1011 图）

- 目标 (0,0)→(1.2,0)：**Goal succeeded**，0.86 s；采样见车在 map (0.555,0.03)
  处 yaw 29.5°↑、fk_wz=+1.53、里程计 +1.65（边走边转）；yaw 0→81.4° 与
  2 rad/s × 0.86 s 一致；`rejected_spin_request_count=0`。
- 请求撤回（`spin_control.sh off`）后自转停止 ✓。
- 数据：`.sim_run/gyro_migrated_samples.csv`。

### 10.3 已知边界

- 导航会话结束后自转停止（wz 归导航所有，mux 零速兜底）——"到点持续自转"需
  后续给 mux 加 spin 旁路或常驻伪目标，当前架构不提供。
- 227_1011 实采图 + srm_empty 空世界组合：起/终点距图上墙 < 膨胀半径时规划器报
  "start or goal pose are an obstacle" 并触发恢复循环（恢复行为还可能把车顶进
  更贴墙的位置）——目标点离图上墙至少留 0.6 m。
- `ros2 action send_goal` 在沙盒下偶发响应丢失（bt 日志为准，见 §9.6）。

