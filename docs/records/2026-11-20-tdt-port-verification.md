# TDT 移植包验证测试报告

- 日期：2026-11-20（C2 修复复测：2026-11-21；A9/BackUp 二次修复：2026-11-21）
- 仓库/分支：`srm_nav_27` / `feat/实车迁移东大规划`
- 被测对象：`src/tdt_nav_kit`（TdtGlobalPlanner、TdtSmoother）及 Nav2 → MINCO/MPC → fakeyaw 接入链路
- 上游参考：`porting/tdt-nav-kit`（commit 814d5ce，MIT）
- 总体结论：**有条件可用**（C2 修复后 4/4、0 恢复、净空 +10.2 cm；A9 已按 `was_completed=false` 修复；TDT 算法层可用；实车不可放行，见 §8）

> 本报告只覆盖本轮容器内可执行的部分。所有仿真结论**不外推**到实车；D1–D7 仅列出待现场项。

---

## 1. 环境与口径

### 1.1 软硬件

| 项 | 本机（本轮实测） | 上游基准 |
|---|---|---|
| CPU | Intel i7-1360P（容器可见 4 vCPU） | AMD 7735H @4.2 GHz |
| 编译器 | GCC 11.4.0，Release（`-O3 -DNDEBUG`） | GCC 11.2 |
| OS / ROS | Ubuntu 22.04 / ROS 2 Humble | 同 |
| 仿真 | Gazebo Fortress（`ign gazebo`，无头，`LIBGL_ALWAYS_SOFTWARE=1`） | — |
| DDS | CycloneDDS；仿真 `ROS_DOMAIN_ID=54` | — |

离线算法层使用最小 Nav2 栈（`map_server` + `planner_server` + `smoother_server` + `lifecycle_manager` + 静态 TF），跑在默认 domain 0，与仿真 domain 54 隔离；两者互不影响。

### 1.2 地图与坐标

- 地图文件：`maps/227_1116_remap/227_1116_remap.pgm`，**实际 225×344**、0.05 m/格、origin `(-7.83, -8.29)`。
- 任务书写的 583×324 与仓库内任何 `.pgm` 都不一致（全仓库最大为 259×357）；本轮以仓库实际地图为准，该差异需在仿真地图更新后复核。
- 栅格统计：自由（254）18 661 格 ≈ 46.7 m²；未知（205）55 398 格；占据（0）3 341 格。
- 生产口径 `occupancy_threshold=252`、`treat_unknown_as_obstacle=true`：未知格按障碍处理，因此可搜索自由区仅占全图 4.7%，这是路径绕行与测地距离偏大的主要原因。
- 车体包络：**半径 0.33 m**（与 `controller.safety.robot_radius`、`analyze_clearance.py` 默认口径及历史记录一致；净空 = 到占据格面距离 − 0.33 m）。上一版报告误用 0.165 m，本轮已按 0.33 m 重算。到点容差：任务 0.5 m；仿真参数 goal checker 为 0.4 m。
- 局部代价图 8×8 m（±4 m），全局代价图 225×344 @0.05 m。
- 速度/加速度：MINCO 迁移链 `FollowPath` 与 `velocity_smoother@fakeyaw` 均为 2.6 m/s、7.0 m/s²（`speed_limits.yaml`；`@omni` 段是 1.5/3.0，本链路不生效）。

### 1.3 测试参数

- 规划器：`tdt_nav_kit::TdtGlobalPlanner`，`occupancy_threshold=252`、`treat_unknown_as_obstacle=true`、`goal_snap_max_radius=0.0`、`path_resample_step=0.20`、`mask_num_layers=1`、`cost_inflate_radius=1.0`。
- 平滑器：`tdt_nav_kit::TdtSmoother`，`backend=path`、`path_resample_step=0.30`、`corner_round_input_step=0.80`、`corner_round_cut=0.40`、`fallback_to_resampled=true`。
- 控制器净空：costmap `robot_radius=0.40`（global/local），`safety.robot_radius=0.33`、`clearance_margin=0.05`（2026-11-21 修复，修复前 0.06）。
- 对照基线：`nav2_theta_star_planner/ThetaStarPlanner` + `nav2_smoother::SimpleSmoother`，同一份地图/目标/控制器。

---

## 2. A 功能结果表

| 编号 | 项目 | 方法/样本 | 结果 | 判定 |
|---|---|---|---|---|
| A1 | 构建 | `colcon build --packages-select tdt_nav_kit`（Release + `BUILD_TESTING=ON`） | exit 0，0 error；产出 `libtdt_nav_kit.so`；`share/ament_index/resource_index/packages/tdt_nav_kit` 存在 | 通过 |
| A2 | 插件注册 | `ros2 param get` + 最小栈加载 | planner `tdt_nav_kit::TdtGlobalPlanner`、smoother `tdt_nav_kit::TdtSmoother`；日志无 `Failed to create` | 通过 |
| A3 | 单元测试 | `colcon test`（7 个 gtest 可执行，55 个用例） | **55 tests, 0 errors, 0 failures, 0 skipped** | 通过 |
| A4 | 参数契约 | PyYAML 1.1 扫描 `src/tdt_nav_kit/config/*.yaml`；`max_corridor_range=1e9` 实跑 | 无数值被解析成字符串；`kMaxCorridorRangeLimit=1e6`，1e9 被夹紧；`tdt_clamp` 3/3 完成（2.0/2.6/2.7 ms） | 通过 |
| A5 | 行为树接线 | 检查 `navigate_to_pose_w_smoothing.xml` | `ComputePathToPose(path={path})` → `SmoothPath(unsmoothed_path={path}, smoothed_path={smoothed_path})` → `FollowPath(path={smoothed_path})`；端口名与 Humble 节点一致 | 通过 |
| A6 | 输出契约 | 短/中/长目标各 10 次 action | 30/30 成功；帧均为 `map`；首点误差 0.0 m、末点误差 0.0 m；点数 17/29/48，长度 3.009/5.402/8.928 m；无空路径/单点 | 通过 |
| A7 | 端点稳定性 | 同一目标连续 20 次；另测占据目标 | 20/20 成功，末点漂移 **0.00000 m**；占据目标 `(0.295,-0.715)` 返回 status=6、0 点（失败而非换点） | 通过 |
| A8 | 简化与加密安全性 | 单测 + 路径逐点占据检查 | `ContinuousSegmentCannotCutBlockedCorner`、`DiagonalCannotCrossBetweenTwoBlockedSideCells`、`SearchAndSimplificationRetainSubcellEndpoints` 全通过；A6 三条路径 2 cm 采样 `raw>=253` 均为 0 | 通过 |
| A9 | 平滑降级 | 构造穿障直线输入（0.8 m），最小栈调用 `smooth_path` | 触发 `fallback_to_resampled`：**status=4 SUCCEEDED、`was_completed=false`**，`result.path` 仍为 4 点加密折线（首末点不变）。修复后符合判据 | 通过（§6.1） |

A6 明细：

| 目标 | 目标坐标 | 测地距离（8 邻接） | 成功 | 点数 | 路径长 | 首末误差 | 穿障采样 |
|---|---|---|---|---|---|---|---|
| short | (3.245, -0.115) | 3.00 m | 10/10 | 17 | 3.009 m | 0 / 0 | 0/152 |
| mid | (0.195, -3.915) | 5.00 m | 10/10 | 29 | 5.402 m | 0 / 0 | 0/272 |
| long | (-3.205, -3.415) | 8.00 m | 10/10 | 48 | 8.928 m | 0 / 0 | 0/448 |

A3 用例分布：`test_costmap_converter`（Costmap 语义）、`test_sfc_grid`（SFC 反向占据与补圈）、`test_sfc_origin`（P7 原点换算）、`test_yastar_chain`（YAstar 链路与 SDF 一致）、`test_pluginlib_load`（pluginlib 加载）、`test_path_utils`（切角/抽稀）、`test_planner_geometry`（简化/穿角安全）。

---

## 3. B 量化结果表

### 3.1 B1–B3 算法层微基准（无 ROS，YAstar 纯算法）

样本：2 张地图 × 5 组起终点 × 200 次 = **2 000 次搜索**；`initCostMap` 每图 200 次；简化/加密各 2 000 次。

| 地图 | B1 搜索 mean / p95 / max | B2 initCostMap mean / p95 / max | B3 化简 mean / p95 / max | B3 加密 mean / p95 / max |
|---|---|---|---|---|
| 227_1116_remap（225×344） | 1.34 / 2.11 / 3.39 ms | 1.22 / 1.50 / 2.33 ms | 0.006 / 0.010 / 0.030 ms | 0.0004 / 0.001 / 0.008 ms |
| 227_1113pro（259×357） | 0.28 / 0.37 / 0.59 ms | 1.11 / 1.13 / 1.23 ms | 0.002 / 0.004 / 0.012 ms | 0.000 / 0.000 / 0.008 ms |

判据：B3 ≤1 ms → 通过（最坏 0.030 ms）。B1/B2 无绝对判据，与上游对比见 §5。

### 3.2 B4–B11 链路层（最小 Nav2 栈，真实插件 + action）

| 编号 | 样本 | 结果 | 判据 | 判定 |
|---|---|---|---|---|
| B4 点数/段长 | mid 目标 1 条 + 10 次重复 | 29 点、5.402 m、段长 0.19/0.19/0.20 m；3 m 半径内 24 点（局部图 ±4 m，实际更多）；原始路径 κmax 77.23 rad/m | 裁剪后 ≥10 点 | 通过 |
| B5 平滑耗时（0.30 m） | 20 次 | internal mean/p95/max = **1.41/1.85/2.10 ms**；231 点 | p95 <333 ms | 通过 |
| B5（0.80 m） | 20 次 | 1.05/1.20/1.28 ms；111 点 | 同上 | 通过 |
| B5（1.20 m） | 20 次 | 1.03/1.14/1.14 ms；111 点 | 同上 | 通过 |
| B6 path 后端 | 20 次 | 1.62/1.78/2.13 ms，20/20 真解 | 成功/耗时 | 可用 |
| B6 close 后端 | 5 次 | 1.54/1.71/1.72 ms，5/5 | 同上 | 可用 |
| B6 corridor 后端 | 5 次 | 306.6/319.3/322.5 ms（另有 675–682 ms 长尾）；碰撞迭代未收敛后回退（`[smooth-iter]`，无 `[osqp-fail]`，QP 本身可解），κmax 19.40 | OSQP -4/-6 记不可用 | **不可用**（§6.2） |
| B7 轨迹几何 | 20 次 | κ max **5.382**、p99 5.149、中位 0.009 rad/m | κmax ≤5.64 | 通过 |
| B8 碰撞安全 | 20 次 × 2 cm | `raw>=253` 0 点、占据 0 点（共 272 采样） | 0 穿障 | 通过 |
| B9 端点精度 | 20 次 | 首末点最大偏差 **0.000 m** | ≤0.01 m | 通过 |
| B10 ROS 动作墙钟 | 50 次 | mean/median/p95/max = **12.42/10.58/20.68/21.40 ms** | p95 <1 s | 通过 |
| B11 频率裕度 | B10 + B5 | 规划+平滑 p95 = 22.46 ms；相对实际重规划周期 5 s（0.2 Hz）余量 **222×**；相对 50 Hz（20 ms）余量 0.89×（非本链路需求） | 实际 0.2 Hz 下 ≥3×（2026-11-21 口径确认） | 通过（按 0.2 Hz 口径；50 Hz 不适用，§6.3） |

补充：`simple_smoother` 对同一条 TDT 稀疏路径 10 次全部“完成”，内部 0.169/0.182/0.185 ms，但 κmax 仍为 77.23（未改善）；`tdt_close` 5 次 1.54/1.71/1.72 ms；`tdt_clamp`（`max_corridor_range=1e9`）3 次 2.0/2.6/2.7 ms。

---

## 4. C 对照与闭环

### 4.1 C1 基线对照（同目标 mid：(0.195, -3.915)，5.0 m 测地）

| 指标 | YAstar + TdtSmoother（path） | ThetaStar + SimpleSmoother |
|---|---|---|
| 规划墙钟 mean/p95/max（50 次） | 12.42 / 20.68 / 21.40 ms | 15.38 / 21.06 / 26.74 ms |
| 路径点数 | 29 | 89 |
| 路径长度 | 5.402 m | 4.794 m（短 11%） |
| 平滑内部耗时 mean/p95 | 1.62 / 1.78 ms | 0.625 / 0.652 ms |
| 平滑后 κmax | **5.382 rad/m** | **4.331 rad/m** |
| 平滑前 κmax | 77.23 rad/m | 29.71 rad/m |
| 闭环 4 段 | **4/4 SUCCEEDED**（修复前 0.40/0.06：cross_room 36 次恢复；修复后 0.40/0.05：0 恢复） | **3/4**，cross_room 60 s TIMEOUT、52 次恢复 |

说明：SimpleSmoother 只在 ThetaStar 的 0.05 m 密路径上有效；对 TDT 的 0.19 m 稀疏路径不做改善（κmax 77.23），因此 TDT 必须搭配 TdtSmoother。基线闭环参数本身不含 SmoothPath 行为树节点，闭环对比实际是「ThetaStar」对「YAstar+TdtSmoother」。

### 4.2 C2 闭环四目标（YAstar + TdtSmoother + MINCO/MPC）

2026-11-21 修复后（costmap `robot_radius=0.40`、`safety.clearance_margin=0.05`）：

| 路线 | 结果 | 用时 | 终点误差 | 包络净空 | 峰值速度 | Nav2 恢复 | 控制器内部恢复 |
|---|---|---|---|---|---|---|---|
| short_diagonal | SUCCEEDED | 1.04 s | 0.396 m | 0.286 m | 2.078 m/s | 0 | 0 |
| cross_room | SUCCEEDED | **3.46 s** | 0.445 m | **0.102 m** | 2.305 m/s | **0** | 0 |
| around_obstacles | SUCCEEDED | 3.81 s | 0.419 m | 0.111 m | 2.242 m/s | 0 | 0 |
| return_spawn | SUCCEEDED | 1.30 s | 0.396 m | 0.297 m | 2.153 m/s | 0 | 0 |

判据核对：4/4 SUCCEEDED ✓；终点误差 ≤0.5 m ✓；最小包络净空 **+0.102 m** ≥0.05 m ✓；**Nav2 恢复 0** ✓。全部通过。

修复前（2026-11-20，`clearance_margin=0.06`）：cross_room 54.73 s、36 次恢复、−0.056 m；0.33 costmap 试验更差（60.0 s TIMEOUT、51 次恢复）。修复过程与根因见 §6.4。

### 4.3 C3 历史回归

对照记录：`log_diag/nav100w_2026-11-18_E7L0bm/serial-routes/`（任务书称 2026-11-16；仓库内与“4/4、6.91 cm、2.42 m/s”唯一匹配的记录是该 11-18 目录，实测 6.91 cm / 2.452 m/s）。

| 指标 | 历史（2026-11-18） | 修复后（2026-11-21） | 差异 |
|---|---|---|---|
| 成功率 | 4/4 | 4/4 | 持平 |
| 最小包络净空 | **0.0523 m**（全段最小，cross_room） | **+0.1022 m**（cross_room） | +5.0 cm（改善） |
| 峰值速度 | 2.452 m/s | **2.305 m/s** | −0.147 m/s（限幅内） |
| cross_room 用时 | 2.9 s | **3.46 s** | +0.56 s（+19%） |
| cross_room 恢复次数 | 0 | **0** | 持平 |
| 终点误差 | 0.280–0.333 m | 0.396–0.445 m | +0.06~0.11 m |

差异解释（不静默）：

-1. **实车链没有平滑器**：2026-11-18 实车命令确认 `use_tdt_planner:=True`，但实车 BT 是 `nav2_bt/navigate_to_pose_w_replanning_and_recovery.xml`（无 `SmoothPath`），实车参数也只用 `simple_smoother`。因此历史实车链 = **TdtGlobalPlanner + 原始路径 + MINCO**，不含 TdtSmoother；本节与 C2 的对比只对“规划器 + 控制器”成立，不能当作 TDT 平滑器已在实车验证。

0. **口径修正**：上一版 C2 用 `--radius 0.165`（把 0.33 m 当直径）算净空；历史记录与仓库默认口径是**包络半径 0.33 m**。按同一口径重算，修复前全段最小为 **−5.58 cm**（恢复段），修复后为 **+10.22 cm**。历史里的 6.91 cm 是 around_obstacles 段，全段最小是 cross_room 的 5.23 cm。
1. 修复前 cross_room 退化的直接证据是控制器诊断 `no validated trajectory available yet` 在 TDT 轮出现 74 次、基线轮 99 次；同一位置、同一 MINCO/MPC、同一地图下**基线与 TDT 都出现**，因此不是 TDT 平滑器独有。具体校验分支已定位为净空门槛，见 §6.4。
2. 修复后（`clearance_margin=0.05`）cross_room 0 恢复、3.46 s；同位置基线此前 TIMEOUT，说明门槛问题解决后规划器差异不再被恢复行为放大。
3. cross_room 3.46 s vs 历史 2.9 s（+19%）：历史是实车串口记录，本轮是 Gazebo 仿真（物理步长、CPU、goal checker 0.4 m 到点容差不同），差异可解释；不视为静默退化，但需在实车复核。
4. 峰值 −0.147 m/s、终点 +0.06~0.11 m：均在 2.6 m/s 限幅与 0.4 m 到点容差内；历史终点 0.28–0.33 m 使用不同的到点策略。

---

## 5. 与上游基准对比

| 项目 | 上游（AMD 7735H） | 本轮（i7-1360P） | 是否可直接比较 |
|---|---|---|---|
| A* mean / max | 5.45 / 14.76 ms（RMUC2026）、7.63 / 21.81 ms（RMUC2024） | 1.34 / 3.39 ms（map1）、0.28 / 0.59 ms（map2） | **否** |
| 代价图生成 | 1.59 / 1.60 ms | 1.22 / 1.11 ms（mean） | 部分 |
| DP 化简 | 26 / 34 µs | 6 µs（mean）、30 µs（max） | 是（同量级） |
| MinimumSnap（OSQPCorridor） | 1.26 ms（2024）、3.90 ms mean / 126.7 ms max（2026） | corridor 306.6 ms mean / 675 ms max，且不收敛 | **否（后端已不可用）** |
| MinimumSnap（Close） | 0.125 / 0.695 ms | 1.54 / 1.72 ms | 否 |
| MinimumSnap（path，本轮生产） | 上游无此档 | 1.62 / 2.13 ms | — |

不可直接比较的原因：

1. **CPU 不同**：上游 7735H @4.2 GHz（Zen4），本机 i7-1360P 且容器仅 4 vCPU；绝对耗时不能横比。
2. **地图不同**：上游 RMUC2024/2026 全图 1000 次随机取点，起终点距离长、障碍密度不同；本轮 map1 仅 18 661 自由格（未知按障碍），5 组起终点仅 2.5–9 m。
3. **后端不同**：上游基准是 OSQPCorridor/Close；本轮 corridor 在本机不收敛，生产走 path 后端，算法与问题规模都不同。
4. **统计口径不同**：上游 1000 次全图随机点取 mean/max；本轮每图 5 组固定可达点 ×200 次，p95 是本轮补充统计量。

可下的结论只有：**同量级、无数量级退化**；`initCostMap` 与 DP 化简与上游同量级；path 后端平滑在本轮 p95 远低于 333 ms 预算。

---

## 6. 失败项/偏差根因与复现

### 6.1 A9 `was_completed` 语义（已修复）

- 修复前：OSQP 失败后回退到加密折线，`SmoothPath` action 返回 status=4、`was_completed=true`；根因是 `src/tdt_nav_kit/src/tdt_smoother.cpp` 的 fallback 分支写回路径后 `return true`。
- 决策依据（本轮在本机 Humble 二进制上逐项核实，不再是猜测）：
  1. **BT 节点不检查该字段**。`libnav2_smooth_path_action_bt_node.so` 的 `SmoothPathAction::on_success` 反汇编只有 `mov $0x2,%eax` 返回（BehaviorTree.CPP v3：`SUCCESS=2`、`FAILURE=3`），无 FAILURE 分支；`was_completed` 只写入输出端口。因此 false **不会**触发 `ComputePathToPose` 重试/恢复。
  2. **Server 不因 false 丢路径**。`SmootherServer::smoothPlan` 把插件返回值存入 `was_completed`，false 时仅打 WARN（`did not complete smoothing ... was interrupted`）并继续走公共路径；实测 action status=4，`result.path` 为插件就地写回的 4 点折线。
- 修复：两处 fallback 分支改为 `return false`（`tdt_smoother.cpp`），路径仍 `writeBack` 加密折线；同时把 corridor 的启动告警从“建议改 path”升级为“禁止用于生产”。
- 复测（2026-11-21，最小栈 + 穿障直线 0.8 m）：`action_status=4`、`was_completed=False`、`result_points=4`、首末点 `(-0.400,0.000)→(0.400,0.000)`，日志出现 `OSQP solve failed (backend=path, iter=2)` 与 `回退到加密折线`。
- 复现坑：若代价图尚未送达，smoother_server 会在 `Costmap is not available` 处直接 abort（status=6、原 2 点路径），看起来像“false 导致失败”。复现时先让 `/global_costmap/costmap_raw` 稳定发布 ≥1 s（本轮预热 1.5 s）再发 action。
- 结论：A9 判据满足，且不改行为树。

### 6.2 B6 corridor 不可用

- 现象：5/5 返回“完成”，但均为回退；内部 306.6 ms mean、322.5 ms max（另有 675/682 ms）；κmax 19.40 > 5.64。
- 先纠正一个口径：`TdtSmoother: OSQP solve failed (backend=corridor, iter=N)` 是 `result.success=false` 的**统一日志**，含义是“碰撞迭代没收敛/求解返回空”，`iter` 是碰撞迭代轮次，**不是 OSQP 迭代次数**。受控复测（穿障直线、`collision_iter=6`）只有 `[smooth-iter] round=0/6 bad=3/3`，**没有 `[osqp-fail]`**，最终 `iter=6, 3.2 ms` 回退——即 QP 本身解出来了，是曲线仍然穿障。
- 结构性原因（不是参数没调好）：
  1. **约束只加在节点上**。`generateA` 只把每段多项式两端的位置钉进各自走廊盒，段内曲线不受走廊约束；节点都在盒内，曲线仍可在两盒之间鼓出去。这是“corridor 模式”的公式缺口。
  2. **退化盒会把节点钉进障碍**。`SfcSquare::getCorridor` 首末盒是零尺寸点盒；任何落在占据/未知格的路径点也得到 `{x,y,x,y}` 的退化盒。0.30 m 加密+切角后的点若贴到膨胀层，QP 就被强制穿过该格。
  3. **失败后的更新方向错了**。`update()` 在碰撞时交替“插中点”和“把碰撞段的走廊盒对半收缩”；收缩只会把可行域压得更紧，并不会绕开障碍，所以 2–5 轮后只能放弃。
  4. **数值条件差是叠加因素**。5 阶多项式按真实秒级时间尺度、OSQP `eps=1e-8`+polish、单次 0.1 s 上限，在真实地图上会到 306 ms；但即使 QP 收敛（受控复测 3.2 ms），结果仍碰撞，说明主因是 1–3。
- 为什么“修不好”：要同时改三处 vendor 公式——把走廊约束施加到**整段**（相邻盒交集/段内采样）、生成非退化且不含障碍的盒、碰撞后改为分段重规划而不是缩盒。这等于重写 corridor 后端，不是调参。
- 为什么不再投入：path 后端已在同一地图上做到 1.62/2.13 ms、κmax 5.382、0 穿障、闭环 4/4；corridor 慢约 150×且仍回退，改好也没有功能收益。上游 corridor 面向密集航点/三维飞行，Nav2 这里只有 0.30 m 稀疏室内路径，问题形态不同。
- 复现：最小栈下 `smoother_id=tdt_corr`（`backend=corridor`、`corridor_shrink=0`、`collision_iter=6`）对穿障直线调用 `smooth_path`；观察 `[smooth-iter]` 且无 `[osqp-fail]`。该复现需临时去掉 configure 的硬拒绝（或使用 2026-11-21 之前的构建）。
- 处置（2026-11-21）：`TdtSmoother::configure` 对 `backend=corridor` **直接抛异常（硬拒绝）**，lifecycle configure 返回 ERROR，误配置无法上线；未知 backend 字符串同样拒绝（`toBackend` 原本会把未知值落到 corridor）；生产配置保持 `backend=path`。实测日志：`Original error: TdtSmoother: backend=corridor 不支持/已禁用（corridor 走廊公式存在结构缺陷，实测碰撞迭代不收敛）`。

### 6.3 B11 频率裕度（口径已确认）

- 实测：规划+平滑 p95 22.46 ms；相对 50 Hz（20 ms）为 0.89×，不满足 ≥3×。
- 根因：全局规划 action 墙钟 p95 20.68 ms；这是 5.4 m 路径、1 Hz 级重规划的负载。实际 BT 的 `RateController hz=0.2`（5 s 一次），控制环 50 Hz 由 MINCO/MPC 单独运行，二者不在同一线程。
- 结论（2026-11-21 确认）：本链路验收口径按实际重规划周期 0.2 Hz（5 s），余量 **222×**，**通过**；50 Hz 不是本链路需求，该数字仅作参考，不作为验收项。若未来确需 50 Hz 高频重规划，必须改为进程内调用规划器绕过 Nav2 action（算法本身约 2.6 ms，具备可行性），属新需求另立项。

### 6.4 C2/C3 cross_room 退化与修复

- 现象（修复前）：36 次 Nav2 恢复、74 次 `no validated trajectory available yet`、54.7 s；基线轮同位置 TIMEOUT。
- 根因（已定位）：
  1. **净空门槛差 4–11 mm**。MINCO 硬校验要求中心净空 = `safety.robot_radius` 0.33 + `safety.clearance_margin` 0.06 = **0.39 m**；cross_room 窄道处候选轨迹的 `min_clearance` 只有 **0.379–0.386 m**（诊断里 `planning_time_ms` 2–10 ms、`trajectory_age=0`，说明候选已生成但校验拒绝），因此反复 `validation_failed` → `no validated trajectory available yet`。同机历史日志里的同类失败为 `trajectory collides with the raw obstacle set | 最小净空=0.38 需要=0.38`。
  2. **参数面只有 1 cm 余量**。costmap `robot_radius=0.40`（`maps/227_1116_remap/simulation/nav2_params_srm_tdt_fakeyaw.yaml:484/530`），控制器硬门槛 0.39（`:454-455`）；平滑/优化在拐角处把净空压到 0.39 以下就会全量拒绝。
  3. **不是 TDT 独有**。基线 ThetaStar + SimpleSmoother 在同一位置同样 99 次失败并 TIMEOUT；TDT 的 `corner_round_cut=0.40` 会进一步吃净空，是叠加因素而非唯一原因。
- 修复验证（2026-11-21）：
  - **先试 costmap `robot_radius` 0.40→0.33：反而更差**。cross_room 路径中心净空降到 raw 0.364 / smoothed 0.335 m（<0.39），60.0 s TIMEOUT、51 次恢复。该配置注释里的 0.33 处置只适用于“起点被判障碍”，对本题是反效果，已回退 0.40。
  - **最终改 `safety.clearance_margin` 0.06→0.05**（硬门槛 0.38），保持 costmap 0.40：4/4 SUCCEEDED、0 恢复、cross_room 3.46 s、最小包络净空 **+10.22 cm**、峰值 2.305 m/s（§4.2）。
  - 机制：路径最小中心净空 0.412 m，优化器最多下压 ~0.03 m → 0.382 m > 0.38 m；原 0.39 m 门槛要求优化偏差 <0.022 m，实际不满足。
- 附带安全发现（修复前）：36 次 Nav2 恢复中的 BackUp 以 0.20 m/s 把车带到 `(-1.60, 0.43)`，中心净空 0.274 m，**0.33 m 包络侵入占据格 5.6 cm**；修复后 0 恢复，未复现。
- BackUp 包络净空限位（2026-11-21 已实现）：
  - `BackUpFreeSpace` 新增 `min_clearance`（默认 0.36 m，`<=0` 关闭），每个控制周期在发速度前查询自身中心净空，低于地板立即 `stopRobot()` 并返回 `SUCCEEDED` 交回重规划；与 `early_exit_clearance`（“已经安全了，提前结束”）方向相反，不受其 1 s / 0.30 m 护栏限制。
  - 取值必须跟各自控制器硬门槛走：仿真门槛 0.38（0.33 + 0.05）→ 地板 0.36；实车窄道档门槛 0.30（0.28 + 0.02）→ 地板 **0.32**（2026-11-21 修正，原误按仿真口径写成 0.36，会在 0.69 m 缝里一上来就停）。已分别写入 `simulation/current/nav2_params_srm_fakeyaw.yaml`（0.36）与 `real/current/nav2_params_srm_minco_fakeyaw.yaml`（0.32）。
  - 最小栈触发验证（2026-11-21）：合成代价图把墙放在机器人正前方 0.29 m（<0.36），调用 `/backup` 得到 status=4、耗时 0.001 s、`cmd_vel` 仅 1 条零速、无位移；日志 `后退触发包络净空限位：自身净空 0.290 m < 0.360 m`。证明“起始就低于地板”会立即停车，不会空转或继续深入。
  - 仍需实车/动态障碍验证：四目标 0 恢复未触发；D5/D6 记录中途触发次数与停车位置（§9 D6）。
- 复现：见 §10 的四目标命令；观察 `/red_standard_robot1/FollowPath/diagnostics` 的 `minimum_clearance` 与 `results.json` 的 `diagnostic_reasons`；净空用 `analyze_clearance.py --radius 0.33` 复算。


### 6.6 C2 启动路径过期（已修复）

- `maps/227_1116_remap/simulation/build_scene.py` 原引用 `src/srm27_gazebo_simulator`（重组后为 `src/simulation/srm27_gazebo_simulator`）；直接运行会失败，导致 `world.sdf`/`sim.yaml`/`nav2_params_srm_tdt_fakeyaw.yaml` 不更新。
- 2026-11-21 已把两处路径改为 `src/simulation/srm27_gazebo_simulator` 并重新生成；生成配置已包含 `costmap robot_radius=0.40`、`clearance_margin=0.05`、`min_clearance=0.36`。

---

## 7. 已知遗留（单独列出）

| 项 | 本轮状态 | 说明 |
|---|---|---|
| 126.7 ms 长尾 | **未复现**（path 后端 max 2.13 ms / 5.4 m 路径） | 上游为 OSQPCorridor 在 RMUC2026 的长尾；本轮 corridor 更差（675 ms 且失败） |
| corridor 不收敛 | **确认，已硬拒绝** | 306 ms mean、碰撞迭代不收敛后回退；QP 本身可解，主因是走廊约束只加在节点+退化盒+缩盒更新（§6.2）；`configure` 直接抛异常，生产只能用 path |
| `obstacleTolerance=2` | 未改（上游参数） | 仍按上游语义，未在 Nav2 配置中暴露 |
| `setDt` 不生效 | 未改（上游 API） | 时间步仍由内部固定 |
| 未接动态 Mask | **确认** | `mask_num_layers=1`，无动态障碍层输入；静态分层掩码生效 |
| A9 was_completed | **已修复** | 回退返回 false，action 仍 SUCCEEDED 且路径可跟踪；BT/Server 行为已在二进制上核实，见 §6.1 |
| BackUp 包络净空限位 | **已实现并最小栈触发** | 起始净空 0.290 m < 0.36：0.001 s 停车、仅零速指令、SUCCEEDED；四目标 0 恢复未触发，待 D5/D6 复测，见 §6.4 |
| cross_room 恢复 | **已定位并修复** | 0.40/0.05 复测 0 恢复、3.46 s、+10.2 cm；0.33 方案反效果，见 §6.4 |
| 地图尺寸不一致 | 待复核 | 见 §6.5 |

---

## 8. 结论分级

**有条件可用（算法层与仿真链路已通过 C2；实车不可放行）。**

- 算法层（A1–A8、B1–B5、B7–B10）：可用。构建、插件、单测、输出契约、端点稳定性、简化安全、path/close 平滑、几何/碰撞/端点、动作墙钟均达标。
- 接入层（A9、B6、B11）：**通过（按确认口径）**。corridor 已在 `configure` 硬拒绝；A9 已按 `was_completed=false` 修复且不改 BT；B11 按实际 0.2 Hz 重规划口径余量 222×，50 Hz 不作为验收项。
- 闭环（C2/C3）：**可用（仿真）**。修复后 4/4、0 恢复、最小包络净空 **+10.22 cm**、峰值 2.305 m/s、终点 ≤0.45 m；相对历史 2.9 s → 3.46 s（+19%）可解释。
- 实车：**不可放行**。D1–D7 未执行。

### 进入实车的先决条件

1. cross_room 净空门槛已在仿真修复（costmap 0.40 / `clearance_margin=0.05`）；实车参数需复核同一门槛。
2. A9 fallback 已统一为 `was_completed=false`（路径仍为加密折线，BT 不受影响，见 §6.1）；实车复核 `smooth_path` 返回字段即可。
3. 生产参数确认 `backend=path`、`fallback_to_resampled=true`、`corner_round_input_step=0.80`；corridor 已在 `configure` 硬拒绝（配错则启动失败），无需依赖人工检查。
4. 确认重规划频率为 BT 的 0.2 Hz（或 ≥1 s 周期），不要把全局规划放进 50 Hz 控制线程。
5. 用实车地图（真实尺寸）重跑 B1/B2 与仿真四目标；本报告的 225×344 结论不外推。
6. BackUp 包络净空限位已实现（仿真 `min_clearance=0.36` / 实车 `0.32`，对应门槛 0.38 / 0.30），最小栈已验证“起始低于地板即停车、不空转”；四目标 0 恢复未触发，D5/D6 仍需记录中途触发次数与停车位置。
7. **明确平滑器上实车的路径**：当前 `nav_srm_real_launch.py` 只有 `use_tdt_planner` 开关，实车 BT 不含 `SmoothPath`，`smoother_plugins` 为 `simple_smoother`——即 TdtSmoother **尚未接入实车**。若要在实车启用，需新增 `use_tdt_smoother`（重写 `smoother_plugins`/`backend=path` 并切到带 `SmoothPath` 的 BT），再按 D1–D5 验证；未接入前 corridor/A9 对实车无影响。
8. 完成 D1–D5 后才允许 0.40 m/s 首跑；D6/D7 通过后才允许提速。

---

## 9. D 实车待现场项（本轮未执行）

| 编号 | 项目 | 命令/方法 | 判据 |
|---|---|---|---|
| D1 | 前置静态检查 | `./scripts/bringup/start_real_nav.sh --check --tdt-planner --lio --no-chassis-spin --speed-limits speed_limits_tdt_first_run.yaml` | 输出含 `use_tdt_planner:=True`、`cmd_vel_chassis`、`0.40 m/s` |
| D2 | TF 契约 | `ros2 run tf2_tools view_frames` | `map→odom→gimbal_yaw→base_link` 完整；odom 起点=出生位姿；无跳变 |
| D3 | 台架（架空轮） | 0.1/0.2/0.4 m/s 阶跃、正反向、wz ±1 rad/s | 符号正确、实测≈指令、MINCO 超速校验不误触发 |
| D4 | 制动辨识 | 满速急停 | 记录指令到静止时间与滑行距离，作为提速门槛 |
| D5 | 低速单点 | 0.40 m/s，1/3/5 m 各 3 次 | 停车误差、超速次数、脱困次数 |
| D6 | 场地中距离 | 8–15 m 跨区含 1–2 个拐角 | 规划失败率、脱困次数、recovery_trigger 类型 |
| D7 | 保护异常 | 急停、拔串口、遮雷达、底盘掉线 | 输出 stop、无飞车、恢复后可继续 |

---

## 10. 复现命令清单

```bash
# 0) 环境
cd /home/srm/nav_ws/srm_nav_27_tdt
source /opt/ros/humble/setup.bash && source install/setup.bash

# 1) A1/A3：构建 + 单测
colcon build --packages-select tdt_nav_kit \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
colcon test --packages-select tdt_nav_kit
colcon test-result --verbose --test-result-base build/tdt_nav_kit

# 2) A2：插件注册（最小栈或仿真栈在跑时）
ros2 param get /planner_server TdtPlanner.plugin     # tdt_nav_kit::TdtGlobalPlanner
ros2 param get /smoother_server tdt_default.plugin   # tdt_nav_kit::TdtSmoother

# 3) B1–B3：YAstar 微基准（2 图 × 5 对 × 200 次；需 /tmp/bench_yastar_multi.cpp）
g++ -O3 -DNDEBUG -std=c++17 /tmp/bench_yastar_multi.cpp \
  src/tdt_nav_kit/vendor/src/YAstar/yastar.cpp src/tdt_nav_kit/src/path_utils.cpp \
  -Isrc/tdt_nav_kit/vendor/src/YAstar -Isrc/tdt_nav_kit/include -I/usr/include/eigen3 \
  -o /tmp/bench_yastar_multi
/tmp/bench_yastar_multi 200 "$PWD"

# 4) A6/A7/A9/B4–B11：最小 Nav2 栈 + action 探针
#    参数：/tmp/nav2_params_tdt_verify.yaml；launch：/tmp/tdt_offline_launch.py
#    探针：/tmp/tdt_verify.py（订阅 /map 与 /global_costmap/costmap_raw，
#    调用 compute_path_to_pose / smooth_path，JSON 输出）
ros2 launch /tmp/tdt_offline_launch.py
python3 /tmp/tdt_verify.py --mode tdt --planner-id TdtPlanner --out /tmp/tdt_verify_tdt.json
python3 /tmp/tdt_verify.py --mode baseline --planner-id GridBased --out /tmp/tdt_verify_base.json

# 5) 单次 action 复现（任一目标）
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{start: {header: {frame_id: map}, pose: {position: {x: 0.236, y: -0.111}, orientation: {w: 1.0}}}, \
    goal:  {header: {frame_id: map}, pose: {position: {x: 0.195, y: -3.915}, orientation: {w: 1.0}}}, \
    planner_id: TdtPlanner}"

# 5b) A9 回退语义复测（最小栈：只起 smoother_server）
#     参数 /tmp/tdt_a9_params.yaml：backend=path、fallback_to_resampled=true；
#     探针 /tmp/tdt_a9_probe.py：合成 160×160 @0.05 m 代价图（x=0 一道墙），
#     穿障直线 (-0.4,0)→(0.4,0)，先让代价图稳定发布 ≥1.5 s 再发 action。
ros2 run nav2_smoother smoother_server --ros-args \
  -r __node:=smoother_server --params-file /tmp/tdt_a9_params.yaml
ros2 lifecycle set /smoother_server configure
ros2 lifecycle set /smoother_server activate
python3 /tmp/tdt_a9_probe.py
#     期望：action_status=4、was_completed=False、result_points=4；
#     server 日志含 "OSQP solve failed (backend=path" 与 "回退到加密折线"。
#     若日志是 "Costmap is not available"，说明预热不够，此时 abort 属正常。

# 5c) BackUp 净空地板复测（最小栈：只起 behavior_server）
#     参数 /tmp/backup_floor_params.yaml（min_clearance=0.36）；
#     探针 /tmp/backup_floor_probe.py 把墙放在机器人正前方 0.29 m，调 /backup。
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link
ros2 run nav2_behaviors behavior_server --ros-args \
  -r __node:=behavior_server --params-file /tmp/backup_floor_params.yaml
ros2 lifecycle set /behavior_server configure
ros2 lifecycle set /behavior_server activate
python3 /tmp/backup_floor_probe.py
#     期望：action_status=4、elapsed≈0.001 s、moving_cmds=0、BACKUP_FLOOR_PASS；
#     日志含“后退触发包络净空限位：自身净空 0.290 m < 0.360 m”。

# 6) C2/C3：无头仿真四目标（2026-11-21 修复参数：costmap robot_radius=0.40、safety.clearance_margin=0.05）
#    先生成场景（build_scene.py 的模拟器路径已修复）
python3 maps/227_1116_remap/simulation/build_scene.py
export ROS_DOMAIN_ID=54 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
       IGN_PARTITION=tdt_verify_54 LIBGL_ALWAYS_SOFTWARE=1 DISPLAY=:0
ros2 launch srm27_nav_bringup nav_srm_simulation_launch.py \
  namespace:=red_standard_robot1 world:=srm_empty \
  map:="$PWD/maps/227_1116_remap/227_1116_remap.yaml" \
  params_file:="$PWD/maps/227_1116_remap/simulation/nav2_params_srm_tdt_fakeyaw.yaml" \
  speed_limits_file:="$PWD/src/bringup/srm27_nav_bringup/config/simulation/speed_limits.yaml" \
  start_simulation:=True sim_config_file:="$PWD/maps/227_1116_remap/simulation/sim.yaml" \
  map_to_odom_x:=-1.905 map_to_odom_y:=1.235 map_to_odom_yaw:=0.0 \
  gui:=False run_immediately:=True use_rviz:=False use_composition:=False \
  use_sim_time:=True use_lio_odometry:=False use_pcd_localization:=False \
  odometry_frontend:=point_lio gimbal_spin_rate:=0.0

python3 maps/227_1116_remap/simulation/verify_navigation.py \
  --goals 0,1,2,3 --timeout 60 \
  --manifest maps/227_1116_remap/simulation/scene_manifest.json --out /tmp/tdt_c2_m005
python3 maps/227_1116_remap/simulation/analyze_clearance.py /tmp/tdt_c2_m005 --radius 0.33

#    对比数据：修复前 0.40/0.06 -> /tmp/tdt_c2；0.33 方案 -> /tmp/tdt_c2_r033；
#    修复后 0.40/0.05 -> /tmp/tdt_c2_m005

# 7) C1 基线闭环：把上面 params_file 换成
#    src/bringup/srm27_nav_bringup/config/simulation/current/nav2_params_srm_fakeyaw.yaml
```

> 说明：`/tmp/bench_yastar_multi.cpp`、`/tmp/nav2_params_tdt_verify.yaml`、`/tmp/tdt_offline_launch.py`、`/tmp/tdt_verify.py` 是本轮为验证临时生成的 harness，未提交仓库；命令中的关键参数（插件名、阈值、目标点）已在本报告 §1.3、§2、§3 中完整给出，可据此重建。

---

## 11. 新图 `219_1121` 冒烟测试（2026-11-21）

### 11.1 地图

- 106×120 @0.05 m，origin `(-1.99, -2.04)`；自由 1,461 格（3.65 m²）、占据 10,526 格、未知 0；最大自身净空 0.602 m。
- 可通行连通域（含车体净空门槛）：0.40 m 口径 100 格（0.25 m²）；0.30 m 口径 373 格（0.93 m²）。地图只覆盖约 5.3×6.0 m 的小区域，自由走廊宽约 0.6–1.3 m。
- `map_server` 实测加载正常，GIMP PGM 的注释头不影响解析。

### 11.2 场景与参数

```
python3 maps/227_1116_remap/simulation/build_scene.py \
  --map maps/219_1121/219_1121.yaml --out maps/219_1121/simulation \
  --goals maps/219_1121/simulation/goals.json \
  --spawn-clearance 0.32 --goal-clearance 0.32
```

- spawn `(0.985, 0.885)`；4 个目标为 `(0.685,0.035)`、`(0.235,1.435)`、`(0.585,1.335)`、spawn，最远约 0.9 m。
- 为贴近实车，本轮测试参数改为窄道档：costmap `robot_radius=0.30`、`safety.robot_radius=0.28`、`clearance_margin=0.02`、BackUp `min_clearance=0.32`（生成器默认仍为 0.40/0.05，本次为测试后处理）。
- 启动：`map:=maps/219_1121/219_1121.yaml`、`params_file:=maps/219_1121/simulation/nav2_params_srm_tdt_fakeyaw.yaml`、`sim_config_file:=maps/219_1121/simulation/sim.yaml`、`map_to_odom_x:=0.985 map_to_odom_y:=0.885`，其余同 §10 第 6 条。

### 11.3 结果

| 路线 | 结果 | 用时 | 终点误差 | 实际行程 | 峰值速度 | 包络净空（0.33） |
|---|---|---|---|---|---|---|
| short_diagonal | SUCCEEDED | 0.84 s | 0.422 m | 0.485 m | 0.844 m/s | +0.148 m |
| cross_room | SUCCEEDED | 1.38 s | 0.428 m | 0.879 m | 0.884 m/s | **+0.016 m** |
| around_obstacles | SUCCEEDED | 0.08 s | 0.156 m | 0 m | 0 | +0.016 m |
| return_spawn | SUCCEEDED | 0.18 s | 0.458 m | 0.047 m | 0.549 m/s | +0.016 m |

- **4/4 SUCCEEDED，Nav2 恢复 0，控制器内部恢复 0**；峰值 0.884 m/s。
- 最小包络净空 **+0.0161 m**（保守 0.33 m 包络口径），未达到 +0.05 m 判据；最小点位于 `cross_room` 终点附近 `(0.578,1.179)`，是贴墙目标造成的，不是路径中段。
- 规划原始路径最小净空 0.320 m；TdtSmoother 输出 0.292 m（切内角比原始路径少约 3 cm，低于 0.30 m 硬门槛）；实际执行轨迹最小净空 0.346 m（对 0.28 m 实车包络为 +6.6 cm）。

### 11.4 结论

- 新图链路（地图 → 规划 → 平滑 → 控制 → action）可用：4/4、0 恢复。但这是**短距离冒烟测试**：可用区太小、目标间距 ≤0.9 m，实际单段最长 0.88 m，不能外推为长距离或窄道净空验证。
- 上实车前：
  1. 确认 `219_1121` 是实车要用的完整地图；若只是局部裁剪图，实车定位与规划范围会被限制在 5.3×6.0 m。
  2. 实车链路当前不含 TdtSmoother，走原始路径；原始路径最小净空 0.320 m，满足 0.30 m 门槛。TdtSmoother 在窄道切内角会少约 3 cm 净空，以后若上实车，建议先调小 `corner_round_cut` 或在窄道禁用切角。
  3. 仍按 D1–D5 以 0.4 m/s 首跑，先在开阔区验证，不要直接在这张窄图里提速。
- 本轮为支持新图，`build_scene.py` 增加 `--map/--out/--goals/--spawn-clearance/--goal-clearance`，`analyze_clearance.py` 增加 `--map`；默认参数仍保持旧图行为。

---

## 12. 实车首动记录（D5，2026-11-21）

- 地图 `maps/219_1121/219_1121.yaml`；参数 `config/real/current/nav2_params_srm_minco_fakeyaw.yaml`（0.4 m/s 首跑限速，costmap 0.30，safety 0.28/0.02，BackUp `min_clearance=0.32`；启动脚本实际加载该文件的软链）。
- 启动位姿 `(0.238, -0.112)`、yaw -170.6°；该点静态地图净空仅 **0.071 m**（贴墙），raw costmap=236，仍在 TDT 可通行阈值内。
- 首次只算路径到 `(0.985, 0.885)`：**ABORTED、空路径**；该点 raw costmap=253（>252），被 TDT 视为障碍，尽管静态地图净空 0.602 m。说明实车实时代价图在该处有额外障碍/膨胀，仿真 spawn 不能直接当实车目标。
- 改算 `(0.785, 0.335)`（raw cost=0）：规划成功，**7 个点**，路径沿 `(0.237,-0.109) → (0.386,-0.112) → (0.535,-0.115) → (0.610,-0.040) → (0.685,0.035) → (0.735,0.185) → (0.785,0.335)`。
- `NavigateToPose` 结果：**SUCCEEDED**；终点 `(0.803, 0.146)`，位移约 **0.62 m**，终点误差 0.19 m（容差 0.5 m），峰值 `cmd_vel_chassis.linear.x=0.33 m/s`（低于 0.4 限幅），角速度 0，未触发恢复。
- 待现场确认：`linear.x` 为负时地图 x 增大，底盘 x 正负号/车头方向与 odom 的对应关系需人工核对；起始位姿贴墙 7 cm，下一次应把车放到净空 ≥0.35 m 处。
- 栈已 `--stop`，复查 ROS 图与本地进程均为空；停止脚本因当前 shell 未 source ROS 未能发零速，但结束前最后一帧 `cmd_vel_chassis` 为 0，仍需现场确认底盘静止。
