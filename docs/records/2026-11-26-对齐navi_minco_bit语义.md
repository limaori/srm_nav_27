# 2026-11-26 对齐 navi_minco_bit 的参数语义（MINCO 仍留在控制器里）

> 目标：除了"MINCO 放在规划器而不是控制器"这一条结构差异之外，其余能对齐的语义全部对齐到
> `~/navi_minco_bit`（参考工程）。本文记录**读了什么、改了什么、实测到什么**，并明确列出
> 哪些差异是刻意保留的、为什么。

## 0. 结论先给

| 结论 | 证据 |
| --- | --- |
| 弯道"切内道"的根因是 MINCO 代价函数里**缺参考吸引项**，不是转角降速没调好 | 关掉/打开 `w_reference`：弯中偏离中位 0.497 m → **0.053 m**（最大 0.660 → 0.195 m） |
| 参考工程的"转角降速"移植过来后**几何上几乎不生效** | 本项目输入是平滑后的密集路径，弦长窗口量到的转角只在 8/36 帧 >25°；逐段速度上限最低 0.73 m/s，但轨迹峰值仍是 2.47 m/s（起点速度边界决定） |
| 视野 3 m → 6 m（参考 `lookahead_dist`）有正收益 | 与 `w_reference` 同时开：弯中 0.044/0.122 m、最小净空 0.47 m、总时长 17.1 s（基线 16.9 s） |
| 直道居中问题在**平滑层**，不在 MINCO | 直道 MINCO-参考只有 0.002~0.02 m；居中靠 `corridor_shrink`（0.40）已修 |

## 1. 参考工程的真实结构（2026-11-26 逐文件读）

- `minco_planner::MincoPlanner` 是一个 `nav2_core::GlobalPlanner`。`createPlan()` **只记录目标**并
  返回一条假的两点路径；真正的规划在 20 Hz 的内部 FSM 里：
  A*/SMAC2D（全局） → 按 `lookahead_dist 6.0` 裁剪 → ROGMap 边界裁剪 →
  LOS 稀疏化（保留真转角顶点） → 梯形/三角形时间分配（`PTAllocation`） →
  MINCO 优化（`local_vmaxs` 逐段速度上限） → 0.05 s 硬校验 → 自己发布 `/opt_path`。
- 控制器是 SE(2) MPC，直接订阅 `/opt_path`，**不订阅 Nav2 的 path**。
- 参考工程的 BT 里确实有 `SmoothPath`，但它的输入是上面那条**假两点路径**，输出对控制没有影响
  —— 也就是"参考工程没有真正的平滑层"。
- ESDF 来自 ROGMap（滚动占据栅格 + 光线投射，20 Hz），不是 Nav2 costmap。
- 转角降速在 `minco_utils::calCurvatureDecay`：`angle < deadzone` 不降速；否则
  `ratio = (min(angle,saturation) - deadzone) / saturation`，
  `v_local = min_vel + (v_max - min_vel) * exp(-ratio * decay_power)`；
  之后 `VelPropogation()` 按加速度上下界做前后向可达化，结果作为 `local_vmaxs` 传给优化器。

## 2. 逐项对照

| 项 | navi_minco_bit | 本仓库（改前） | 处理 |
| --- | --- | --- | --- |
| 参考吸引 `penalty_weight_att` | 100.0 | **没有这个参数**（`w_reference` 固定 0） | ✅ 暴露 `minco.w_reference`，设为 100 |
| 转角降速 | `turn_angle_deadzone 0.1` / `saturation 0.8` / `min_turn_vel 0.7` / `decay_power 1.6` | `corner_speed_ratio 0.45` + `corner_angle_threshold 0.35`（密集路径上恒不生效） | ✅ 四个参数同名移植 + 弦长窗口（本项目扩展）；地板加比例写法 |
| 逐段速度上限进优化器 | `optimize(..., local_vmaxs, ...)` | 只有全局 `max_linear_speed`，转角降速只存在于初值 | ✅ `TrajectoryInitialGuess::speed_limits` → 优化器逐段铰链阈值 |
| 视野 | `lookahead_dist 6.0` | `planning_horizon 3.0` | ✅ 6.0 |
| 恢复门限 | `fail_threshold 5` / `cooldown_sec 1.0` / `escape_speed 0.5` | 10 / 2.0 / 0.2 | ⚠️ 只对齐 `fail_threshold`（10→5）。`cooldown/escape` 保留实车已标定值：实车注释里 `escape_speed 0.2 × window 3 s = 0.6 m` 的盲推窗口是现场调过的，参考工程 0.5 会让窗口变 1.5 m，属于另一件事 |
| 重规划频率 | `opt_freq 20` | `replan_frequency 10` | ⚠️ 保留 10：单节点内 30 ms 预算要同时跑 MINCO + 50 Hz MPC，20 Hz 需先实测 CPU 余量 |
| 安全距离 | `safe_dist 0.35` / `collision_dist 0.2`（机器人半径 0.25） | `safety.robot_radius 0.245 + clearance_margin 0`（包络 0.245）+ 优化余量 0.02 | ⚠️ 保留：本车 CTG 半径 0.245，0.35 会切掉可行走廊 |
| 代价权重量纲 | `pos 50000 / vel 1500 / acc 1500 / time 100`（ESDF 距离场 + 平方罚） | `w_obstacle 100(m) / w_velocity 60(m²/s²) / w_acceleration 300 / w_time 25`（softplus 铰链） | ❌ 不可直接照抄：两类代价的定义与单位不同，只能按实测标定 |
| 速度上限 | `max_velocity 3.0 / max_acceleration 2.0` | 2.6 / 7.0（`speed_limits.yaml`，底盘能力） | ❌ 硬件参数 |
| 规划器 | SMAC2D + ROGMap ESDF | YAstar + MinimumSnap 平滑 | ❌ 结构差异（MINCO 位置迁移后才会统一） |
| 平滑层 | 名义上有、实际无（输入是假路径） | 真实存在（`/plan_smoothed` 喂 MINCO） | ⚠️ 保留：MINCO 留在控制器，就必须有平滑前端 |
| `time_allocation_iters 15` / `penalty_weight_time_barrier 50` | 有 | 无对应项（本仓库用解析速度剖面 + `w_time_ratio`） | ⚠️ 语义不同，未移植 |

## 3. 代码改动

### 3.1 前端（`srm27_minco_core`）

- `TrajectoryInitializer::Config` 新增 `turn_angle_deadzone`、`turn_angle_saturation`、
  `min_turn_vel`、`min_turn_vel_ratio`、`decay_power`、`corner_chord_window`；
  `corner_speed_ratio` / `corner_angle_threshold` 标注 `\deprecated`，代码不再读取。
- 转角检测改为**弦长窗口**：在重采样后的密集路径上，取前后各半窗的弦夹角，等价于参考工程在
  稀疏顶点上量到的转角。直道上的小幅横向抖动落在死区内，不会误触发。
- 新增 `curvatureDecay()`，公式与 `navi_minco_bit::utils::calCurvatureDecay` 逐字一致。
- 逐段速度上限：`speed_limit` 先做"可制动到达"的传播（先后向、再前向，起点抬到不低于实测速度），
  再按段取包络写入 `TrajectoryInitialGuess::speed_limits`。
- 新增 `TrajectoryInitialGuess::corner_angle_max`（诊断用）。

### 3.2 优化器（`srm27_minco_core/minco_optimizer`）

- 速度软约束的阈值从"全局一个值"改为"逐段可覆盖"：`piece_speed_sq_` 来自
  `TrajectoryInitialGuess::speed_limits`，只允许比全局上限更严。这是参考工程
  `optimize(..., local_vmaxs, ...)` 的等价物。

### 3.3 控制器（`srm27_minco_controller`）

- 声明并读取 `minco.w_reference`、`minco.turn_angle_*`、`minco.min_turn_vel[_ratio]`、
  `minco.decay_power`、`minco.corner_chord_window`，启动日志打印生效值。
- 诊断新增 `corner_angle_max_deg`、`corner_speed_limit_min`、`corner_limited_pieces`：
  现场可以直接看出"这条参考上有没有够尖的转角、上限压到多少、几段受限"。

### 3.4 配置

`config/real/current/nav2_params_srm_minco_fakeyaw.yaml`、`config/simulation/current/nav2_params_srm_minco.yaml`、
`maps/227to219_1125/simulation/nav2_params_yastar_realmatch.yaml` 同步：
`planning_horizon 6.0`、`w_reference 100`、`turn_angle_deadzone 0.10`、`turn_angle_saturation 0.80`、
`min_turn_vel 0.12`、`min_turn_vel_ratio 0.23`、`decay_power 1.6`、`corner_chord_window 0.40`，
`recovery.fail_threshold 5`。仿真镜像文件（realmatch）与实车文件保持逐项一致，`escape_speed/cooldown`
沿用实车现场标定值。

### 3.5 测试

`test_trajectory_initializer.cpp` 新增 6 个用例（转角上限存在且不低于地板、直线不受影响、
`min_turn_vel >= v_max` 等价关闭、死区抑制、饱和值单调性、比例随速度档缩放、
起点段上限不低于实测速度）；`test_minco_optimizer.cpp` 新增 1 个用例
（逐段上限确实压低最优解的巡航速度）。`srm27_minco_core` 全量：initializer 20、optimizer 19、
validator 29 全绿。

## 4. 实测 A/B（同一目标、同一速度档 2.6 m/s）

做法：四个变体各跑一遍 `navigate_to_pose(-7.58, 20.75)` 并录包；指标取自 bag 的
`planning_input_path` / `minco_trajectory` / `odometry` 三层，脚本
`.diag_tmp/ab2/summarize.py`。

| 变体 | 直道 MINCO-参考 中位/最大 (m) | 弯中 MINCO-参考 中位/最大 (m) | 速度峰值/均值 (m/s) | 最小净空 (m) | 耗时 (s) |
| --- | --- | --- | --- | --- | --- |
| v0 基线（旧语义） | 0.013 / 0.385 | 0.497 / 0.660 | 2.40 / 1.61 | 0.38 | 16.9 |
| v1 + `w_reference 100` | 0.002 / 0.183 | **0.053 / 0.195** | 2.44 / 1.76 | 0.46 | 15.8 |
| v2 + 转角降速（0.23） | 0.020 / 0.392 | 0.324 / 0.684 | 1.93 / 1.00 | 0.35 | 28.4 |
| v3 + 转角降速 + 视野 6 m | **0.001 / 0.121** | **0.044 / 0.122** | 2.42 / 1.62 | **0.47** | 17.1 |
| v4 最终配置（= 仓库里已落地的 yaml，复测） | 0.001 / 0.122 | 0.044 / 0.122 | 2.40 / 1.81 | 0.46 | **15.3** |

读数：

1. `w_reference` 是唯一起决定作用的改动：弯中偏离降一个数量级，且**更快**（均值 1.61→1.76 m/s，
   停车时间 26%→23%）——它让轨迹贴着参考走，不用靠"切内道 + 反复重规划"。
2. 只加转角降速（v2）反而更慢（均值 1.00 m/s、28.4 s）且几何没改善。
3. v3 = v1 + 转角降速 + 视野 6 m，几何最好、净空最大、耗时与基线持平 → **最终采用**。

### 4.1 为什么转角降速在本项目里"不咬"

诊断实测（v2、v3 的 `FollowPath/diagnostics`）：

- 每帧裁剪后的参考上量到的最大转角：v2 有 13/36 帧 > 5.7°，8/36 帧 > 25°，最大 57.5°；
  v3（视野 6 m）只有 6/24 帧 > 5.7°、1 帧上限 < 2.0 m/s。
- 逐段速度上限：v2 最低 0.73 m/s、3 段受限，指令速度在弯区确实降到 ~1.1 m/s；
  v3 最低 1.93 m/s（6 m 视野下 12 段 → 每段 0.5 m，转角段的上限被同段内更快的采样拉高）。
- 几何没有跟着改善：切弯量由 MINCO 的"时间 ×  jerk"目标决定，降速只会让它在同样的内侧
  几何上开慢一点。

结论：参考工程的转角降速耦合在"稀疏折线 + 逐段上限"这套输入上；本项目的输入已被
MinimumSnap 平滑成圆弧（曲率半径 ≥1 m），真正决定几何的是参考吸引项。

![弯道几何 A/B](../../.diag_tmp/ab2/ab_corner.png)

左：基线，MINCO（蓝）每条都朝弯心切、实际轨迹（橙）贴内墙；右：最终配置，MINCO 与参考（灰虚线）
基本重合。两图同一范围、同一目标。

## 5. 仍未对齐 / 刻意保留

1. **MINCO 位置**：仍在控制器里（本次任务边界）。
2. **规划器与 ESDF 来源**：YAstar + costmap ESDF vs SMAC2D + ROGMap。
3. **平滑层**：本仓库保留（MINCO 在控制器时它是必要前端）。若要完全对齐，等价于把 MINCO
   迁进规划器并让 `SmoothPath` 变成名义步骤 —— 属于位置迁移那件事。
4. **代价权重**：量纲不同，不能照抄；只能逐项按实测标定。
5. **重规划频率 20 Hz**：待实测 CPU 余量（当前 10 Hz 下 MINCO 单次 10~20 ms，30 ms 预算）。

## 6. 复现

```bash
# 单次 A/B 单元（自动重试 map_server 偶发 configure 超时）
bash .diag_tmp/ab2/run_ab.sh .diag_tmp/ab2/v3_both_h6.yaml .diag_tmp/ab2/out_v3 v3_both_h6
# 汇总四组
python3 .diag_tmp/ab2/summarize.py
```

变体 yaml：`.diag_tmp/ab2/v{0..3}_*.yaml`；录包：`.diag_tmp/ab2/bags/`。
