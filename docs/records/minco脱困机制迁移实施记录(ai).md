# MINCO 脱困机制迁移实施记录（AI 生成）

把北理 `navi_minco_bit` 里 `minco_planner` 自带的脱困能力（`RecoverServer` + `MincoFsm::RECOVERING`）
迁移到本工作区的 Nav2 链路。**只做代码与参数改动，未做运行期验证**（仿真/实车标定见 §9）。

---

## 1. 迁移前的差距

参考实现（`navi_minco_bit/src/navigation/minco_planner`）的脱困链路是：

```
FSM 重规划失败 -> RecoverServer::handleReplanFailure（判定 + 算逃逸方向）
  -> FSM 切 RECOVERING -> 每 tick publishEscapeCommand（0.5 s 伪轨迹发 /opt_path）
  -> 自家 MPC 跟踪伪轨迹 -> cmd_vel
  -> RECOVERING 每 tick 判 ESDF > 0.40 成功 / 3 s 超时
```

本工作区（`srm_nav_27`）**没有这条链路**，逐项对照：

| 参考实现的依赖 | 本工作区现状 |
|---|---|
| `MincoFsm`（5 状态机，含 `RECOVERING`） | **不存在**。重规划失败只在 `harvestPlanningResult` 里打日志 |
| `/opt_path`（`MpcPositionCommand` 伪轨迹接管 MPC） | **不存在**。控制器是 Nav2 `FollowPath` 插件，唯一速度出口是 `computeVelocityCommands` 返回值 |
| `RecoverServer`（判定 / 逃逸方向） | **不存在** |
| ESDF 距离查询（4 次有限差分求梯度） | 已有 `srm27_minco_core::Esdf2D::query()`，返回 **距离 + 解析梯度 + valid** |
| 脱困动作 | 已有 `srm27_nav_behaviors/BackUpFreeSpace`（代价图直方图法选方向），精度远不如 ESDF 梯度 |

因此**不能逐行搬运**，必须把三类职责重新落到 Nav2 的架构上。

---

## 2. 落点决策

| 职责 | 参考实现位置 | 本工作区落点 |
|---|---|---|
| **脱困判定**（连续失败计数 / 门限 / 冷却 / 卡死分诊） | `RecoverServer::onReplanFailure` + `shouldTryRecovery` + FSM 的 `ESDF < 0.25` 分诊 | `srm27_minco_core::RecoveryDecider::decide()`，由控制器在 `harvestPlanningResult` 的失败分支调用 |
| **脱困路径规划**（逃逸方向） | `RecoverServer::calculateEscapeVelocity`（ESDF 梯度 / 环形搜索） | 同一个 `RecoveryDecider::planEscapeDirection()`（静态函数），**控制器与 `BackUpFreeSpace` 共用** |
| **脱困成功判定** | FSM `RECOVERING` 分支：`ESDF > 0.40` 成功 / `window` 超时 | `RecoveryDecider::updateRecovery()`，由控制器每个控制周期调用 |
| **脱困执行** | `publishEscapeCommand` 伪轨迹 -> MPC | 控制器**旁路 MPC** 直接返回逃逸 `TwistStamped`；行为树的 `BackUpFreeSpace` 作为兜底 |

关键取舍说明：

* **为什么控制器内部旁路 MPC，而不是只抛异常交给行为树**：参考实现的脱困时间尺度是
  1.5~3 s；行为树的 recovery 要等 `build_grace_period`(1 s) 超时 + `ClearEntireCostmap`×2 +
  `BackUpFreeSpace`(`backup_dist` 1.0 m / `backup_speed` 0.2 m/s ≈ 5~7 s)，一个来回 10 s 量级。
  把控制器的快链路保留下来，行为树的慢链路退化为兜底。
* **为什么方向要每周期重算**：参考实现的 FSM 在 `RECOVERING` 里每次 tick 都调用
  `calculateEscapeVelocity`。方向一旦冻结，"环形搜索选出的最开阔方向只能推一步"——
  车往前挪之后那个方向可能已不再最开阔，而脱困要的恰恰是朝最开阔处持续收敛。
  这一点有专门的回归用例（`UpdateRecoveryRecomputesDirectionFromCurrentPosition`）。
* **为什么 `BackUpFreeSpace` 也改**：它原本按"哪条射线扫不到致命栅格"取扇区中点，
  分不清"0.26 m 的缝"和"2 m 的空地"。改用 ESDF 后与控制器同源。

---

## 3. 新增 / 修改的文件

### 3.1 新增：`srm27_minco_core::RecoveryDecider`

* `src/srm27_navigation/srm27_minco_core/include/srm27_minco_core/recovery_decider.hpp`
* `src/srm27_navigation/srm27_minco_core/src/recovery_decider.cpp`
* `src/srm27_navigation/srm27_minco_core/test/test_recovery_decider.cpp`（20 个用例）

纯计算类，**不依赖 ROS / rclcpp / Eigen 之外的任何东西**：位姿是裸 `(x, y)`，
距离场是 `const Esdf2D &`，不做任何 I/O（环形搜索日志写 stderr）。

主要接口：

```cpp
enum class RecoveryDecision { kNone, kEscape, kNoEscapeDirection };
enum class RecoveryExit { kNone, kEscaped, kTimeout };

bool onPlanningFailure(double now_sec);
void onPlanningSuccess();
RecoveryDecision decide(double now_sec, const Eigen::Vector2d & position,
                        const Esdf2D & esdf, Eigen::Vector2d * escape_velocity);
RecoveryExit updateRecovery(double now_sec, const Eigen::Vector2d & position,
                            const Esdf2D & esdf, double clearance,
                            Eigen::Vector2d * escape_velocity, bool log_transitions = false);
static bool planEscapeDirection(const Esdf2D & esdf, const Eigen::Vector2d & position,
                                const RecoveryConfig & config, Eigen::Vector2d * direction,
                                double * best_clearance, bool log_transitions = true);
```

与参考实现的**语义等价项**与**有意差异**：

| 项 | 参考实现 | 本实现 | 说明 |
|---|---|---|---|
| 梯度 | 4 点中心差分，`h = 0.05` | `Esdf2D` 双线性插值的**解析梯度**，1 次查询 | 数值更准、开销更低；语义等价 |
| 梯度平台判据 | `\|grad\| < 1e-3` | `gradient_epsilon`（默认 `1e-3`） | 同值，参数化 |
| 环形搜索 | 36 方向 / 半径 0.4 m | `search_samples` / `search_radius` | 同值，参数化 |
| 时间合法性 | `isfinite(now) && now >= 0` | 同 | — |
| 非法参数 | 夹到安全默认 | 同 | — |
| 成功判据 | 单帧、单点、`> 0.40` | 同（`escaped_clearance`） | 语义刻意保持一致 |
| 超时边界 | `<= window` | 同（含边界仍在窗口内） | — |
| **算不出方向时** | 返回 `ENTER_EMER_STOP`，但该分支已被注释成"回 `GENERATE_TRAJ`"，且**留下 `recovery_active_ = true` 永不复位** | 返回 `kNoEscapeDirection`，**主动关掉会话**、保留失败计数 | 见 §4.1 |
| **冷却是否生效** | `finishRecovery` 写的 `last_recovery_end_time_` 立刻被 `changeState` 里的 `reset()` 抹成 `-1`，**冷却实际失效** | 会话结束时刻由 decider 自己持有，`reset()` 只在换目标/失活时调用 | 见 §4.2 |

### 3.2 修改：`srm27_minco_controller`

* `include/srm27_minco_controller/minco_mpc_controller.hpp`
* `src/minco_mpc_controller.cpp`
* `include/srm27_minco_controller/diagnostics.hpp`
* `src/diagnostics.cpp`

接入点：

1. `harvestPlanningResult(const State2D &, double)` — 改成接收当前状态；失败分支调用
   `handlePlanningFailure`（多了一个"卡死分诊"：自身净空 `< recovery.stuck_clearance`
   才计入失败，否则判"前方路障"直接让重规划绕路）；成功提交轨迹后调
   `onPlanningSuccess()` 清零计数。
2. `computeVelocityCommands` 新增 step 5.5：`updateRecovery(...)`，**放在取回结果之后、
   轨迹复验与重规划请求之前**。取回结果之后是因为要让本周期拿到的成功结果先把失败计数
   清零；复验之前是因为脱困期间把在脱困的轨迹判死、或和后台线程抢算力都没有意义。
3. `fail()` lambda 里的建轨宽限：**脱困期间持续重置 `build_start_stamp_`**，等价于
   "脱困窗口 + 建轨宽限"都走完才把 FollowPath 判失败抛给 BT。这是防止两条脱困链路
   （控制器内部 + 行为树）同时输出速度互相打架的关键。
4. 新增 `makeEscapeCommand` / `leaveRecovery` / `publishRecoveryDebug`。
   * 逃逸速度是规划坐标系（odom）下的世界速度，**必须转车体系**再返回
     （`odomToBodyVelocity`），否则非零航向下会走错方向。
   * 自转恒为 0：与参考实现"yaw 伪轨迹锁存当前 yaw"一致。
   * `leaveRecovery` 清 `has_previous_input_` / `has_last_control_stamp_`
     （脱困命令与 MPC 命令不连续，退出后必须重建差分基准）并重置 `build_start_stamp_`。
5. 新增诊断字段（`recovery.active/trigger/exit/failures/enter_count/...`）与
   可视化话题 `FollowPath/recovery_markers`（净空球 + 逃逸方向箭头）。
6. `activate` / `deactivate` / `setPlan`（新目标）三处复位 decider 与运行期状态。

### 3.3 修改：`srm27_nav_plugins`（`BackUpFreeSpace`）

* `include/srm27_nav_plugins/behaviors/back_up_free_space.hpp`
* `src/behaviors/back_up_free_space.cpp`
* `package.xml`（新增 `srm27_minco_core` 依赖）
* `README.md`

新增 `buildGridSnapshot()`（`GetCostmap` -> `GridSnapshot`）与 `planEscapeDirectionByEsdf()`，
`onRun` 里按 `escape_direction_mode` 分派；ESDF 分支失败时**回退**到原 `findBestDirection`。

**数据布局结论**（本次静态确认，避免踩错一半格）：`nav2_costmap_2d::Costmap2DPublisher::costmap_service_callback`
直接返回 `Costmap2D::getCharMap()`，索引是 `my * size_x + mx`（行优先、自左下角起），
与 `GridSnapshot` 完全一致，**不需要翻转 y**。注意 `nav_msgs/OccupancyGrid` 才是自上而下，
本服务不是。（依据：`objdump -dC` 该符号只调用 `getCharMap()` 后直接拷贝，无循环翻转；
`Costmap2D::getIndex(mx,my) = my*size_x_+mx`。）

语义与 `CostmapAdapter::snapshot` 对齐：只有 `LETHAL_OBSTACLE`(254) 与
`NO_INFORMATION`(255) 进距离场，**膨胀代价 1..252 一律视为自由**——机器人半径已经
在净空阈值里单独计入，重复计入会让有效通道凭空变窄。

### 3.4 参数

`config/simulation/nav2_params_srm_minco.yaml` 与 `config/real/nav2_params_srm_minco.yaml`
都新增了 `controller_server.FollowPath.recovery.*` 与 `behavior_server` 的
`escape_direction_mode` / `esdf_*`。另两份配置（`nav2_params_srm.yaml`、`nav2_params_upstream.yaml`、
`nav2_params.yaml`）用的是 `srm27_omni_pid_controller`，**不含脱困参数**；
但它们共用 `BackUpFreeSpace`，所以行为侧的新参数对它们同样生效（用代码默认值）。

| 参数 | yaml 值 | 代码默认 | 参考实现对应 |
|---|---:|---:|---|
| `recovery.fail_threshold` | 5 | 5 | `fail_threshold: 5` |
| `recovery.cooldown_sec` | 1.0 | 1.0 | `cooldown_sec: 1.0` |
| `recovery.window_sec` | 3.0 | 3.0 | `recovery_window_sec: 3.0` |
| `recovery.escape_speed` | 0.5 | 0.5 | `escape_speed: 0.5` |
| `recovery.stuck_clearance` | 0.25 | 0.25 | FSM 里硬编码的 `0.25` |
| `recovery.escaped_clearance` | 0.40 | 0.40 | FSM 里硬编码的 `0.40` |
| `recovery.search_radius` | 0.4 | 0.4 | 硬编码 `0.4` |
| `recovery.search_samples` | 36 | 36 | 硬编码 `36` |

> 参考实现 yaml 给的是 `fail_threshold 5 / cooldown 1.0 / window 3.0 / escape_speed 0.5`，
> 而代码默认是 `3 / 2.0 / 1.5 / 0.4`。本迁移**取 yaml 值作为参数默认**，
> 让不写这几个参数的配置也拿到与参考实现实车一致的行为。

阈值相对关系（必须保持）：

```
碰撞判据 ~0.2  <  stuck_clearance 0.25  <  轨迹净空要求 0.38  <  escaped_clearance 0.40
```

即"判卡死"发生在车已经贴到几乎要撞的时候，而不是"净空略紧"的时候。
`configure()` 会强制 `escaped_clearance > stuck_clearance`（不足时抬到 `stuck_clearance + 0.15`），
并在启动日志里打印**实际生效**的一组值——防止日志里的阈值与真正跑的不是同一个数。

> ⚠ **调这几个参数必须改 yaml 后重启导航栈，`ros2 param set` 无效。**
> `MincoMpcController` 只在 configure 期（`loadParameters()`）读一次参数，且**没有注册
> `onSetParameters` 回调**（本包全仓搜索无 `onSetParameters` / `add_on_set_parameters_callback`）。
> 对 `controller_server` 执行 `ros2 param set ... FollowPath.recovery.*` 会**成功返回但毫无效果**。
> 对比：`rotation_test_sender` 有自己的回调，所以现场能热改它——两者行为不同，别类推。
> 改完用 `./script/kill_nav.sh`（或直接重跑 `start_sim_nav.sh`，它启动前会自己杀一遍）
> 再起一次，然后核对启动日志里那行 `MincoMpcController 脱困参数: ...` 的实际值。
>
> 现场调参建议顺序：先把 `fail_threshold` 降到 2、`stuck_clearance` 抬到 0.60 确认链路通，
> 再改回 5 / 0.25 复验一次——**正式值才是交付状态**。

---

## 4. 迁移时主动修掉的两个参考实现缺陷

这两条是静态阅读参考实现时发现的真实 bug。按"保持语义一致"的原则本可以照抄，
但它们都会让脱困**永久失效**，所以选择修掉并在此明确记录。

### 4.1 `recovery_active_` 被永久锁死

参考实现 `handleReplanFailure` 在"算不出逃逸方向"时返回 `ENTER_EMER_STOP`，
而 FSM 里 `ENTER_EMER_STOP` 分支已被注释成"回 `GENERATE_TRAJ`"：

```cpp
// 参考实现 minco_fsm.cpp
if (decision == RecoverServer::RecoveryDecision::ENTER_EMER_STOP) {
  changeState(emer_reason, State::GENERATE_TRAJ);   // 只切状态
  return;
}
```

但 `startRecovery` 已经把 `recovery_active_ = true`，而它只在**离开 `RECOVERING`** 时
才被 `reset()` 清掉（`changeState` 里的 `state_ == RECOVERING && new_state != RECOVERING`）。
上面的分支根本没进过 `RECOVERING`，于是 `recovery_active_` 永远为真，
`shouldTryRecovery` 从此恒为 `false`——**后续所有卡死都不再脱困**，
直到换目标（`setMissionGoal`）或取消（`clearMissionGoal`）才恢复。

本实现：`decide()` 在 `planEscapeDirection` 失败时返回 `kNoEscapeDirection`，
**主动关掉会话**并打上冷却参考点，同时**保留失败计数**在门限之上——
于是冷却一过就能立刻重试，而不用从头再累计 `fail_threshold` 次失败。

### 4.2 冷却窗口实际失效

参考实现 `finishRecovery` 写入 `last_recovery_end_time_ = now`，
紧接着 FSM 从 `RECOVERING` 切到 `GENERATE_TRAJ` 时会调 `recovery_server_->reset()`，
把 `last_recovery_end_time_` 抹回 `-1.0`。于是 `shouldTryRecovery` 里
`last_recovery_end_time_ < 0.0` 的分支恒成立，`cooldown_sec` 从来没起过作用——
理论上可以 `RECOVERING <-> GENERATE_TRAJ` 高频往复。

本实现：`reset()` 只在"换目标 / 取消 / 失活 / 定位重置"时调用，脱困会话结束时
只走 `finishRecovery`，冷却参考点得以保留。有专门用例
（`CooldownBlocksImmediateReentry`）钉住这个行为。

---

## 5. 与行为树 recovery 的边界

`navigate_to_pose_w_replanning_and_recovery.xml` 的 `RecoveryFallback` 里
`RoundRobin` 是**空的**（只有 `ClearEntireCostmap`×2 + `BackUpFreeSpace`），
且 `FollowPath`/`NavigateRecovery` 的 `number_of_retries` 是 10。两条链路的先后关系：

```
规划连续失败 N 次且自身净空 < 0.25
  -> 控制器进入脱困（最多 recovery.window_sec = 3 s）
     成功 -> 回重规划，正常导航
     超时 -> 回重规划；若仍失败，等 build_grace_period(1 s) 到期抛异常给 BT
              -> BT: ClearEntireCostmap×2 + BackUpFreeSpace（ESDF 梯度版）
```

**控制器在脱困期间不会抛异常**（`fail()` 里持续重置 `build_start_stamp_`），
所以两条链路不会同时输出速度。这是本次集成里最需要留意的一处副作用：
`build_grace_period` 在脱困期间形同被延长到 `window_sec + build_grace_period`。

---

## 6. 构建与测试

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select srm27_minco_core srm27_minco_controller srm27_nav_plugins \
  --parallel-workers 4
colcon test  --packages-select srm27_minco_core srm27_minco_controller
```

结果（本次）：

* 三个包全部编译通过。
* `srm27_minco_core`：263 个断言级用例全过，其中 `test_recovery_decider` **20/20**。
* `srm27_minco_controller`：65 个用例全过（`test_state_adapter` 需可写的 `ROS_HOME`，
  这是它自身的环境要求，`colcon test` 已通过 `set_tests_properties` 处理）。
* **唯一失败项是 `clang_format`，且是既有问题**：两个包的 `CMakeLists.txt` 都指定了
  `${CMAKE_CURRENT_SOURCE_DIR}/.clang-format`，但该文件在 `df2b7de "fix: 清理一下仓库"`
  里被删掉且从未被 git 跟踪。与本次改动无关。

覆盖的关键行为（`test_recovery_decider.cpp`）：

* 门限计数、非法时间不计、成功清零、冷却挡住立即重入；
* 参考实现缺陷回归：`NoEscapeDirectionDoesNotLockRecoveryActive`；
* 解析梯度分支（竖墙右侧给出 +x）、梯度平台转环形搜索、**环形搜索选到唯一开口**；
* 地图外 / 距离场无效时不给出方向（不得盲目乱冲）；
* 成功 / 超时 / 窗口边界 / NaN 净空不判成功；
* **方向随位置重算**（`UpdateRecoveryRecomputesDirectionFromCurrentPosition`）——
  这条就是钉住 §2"为什么方向要每周期重算"的回归用例，若把方向冻结在进入时刻它会失败。

---

## 7. 观测入口

| 入口 | 内容 |
|---|---|
| `FollowPath/diagnostics` | `recovery_active` / `recovery_trigger`(`gradient`/`circular_search`/`no_escape_direction`) / `recovery_exit`(`escaped`/`timeout`) / `recovery_failures` / `recovery_enter_count` / `recovery_command_count` / `recovery_entry_clearance` / `recovery_clearance` / `recovery_direction_{x,y}` / `recovery_escape_{vx,vy}` |
| `FollowPath/recovery_markers` | **`recovery_escape_path`（绿线，LINE_STRIP）**：脱困路径规划结果 —— 沿方向场前推的轨迹，见 §7.1；`recovery_escape`（箭头）：本周期实际要执行的那一步；`recovery_clearance`（球）：净空（绿=已达成功阈值、黄=仍在卡死分诊区、红=更危险） |
| `ROS_WARN`（每次进入/退出各一条） | 进入：触发分支、进入净空、方向角、速度、窗口、成功阈值；退出：成功/超时 + 用时 + 输出周期数 |
| `stderr` 黄字 | 走环形搜索分支时打印（进入脱困那一次） |
| `BackUpFreeSpace` 日志 | `mode=…, 净空 … m, 环形搜索/ESDF 梯度`；`visualize: true` 时另有 `escape_direction` / `escape_clearance` 标记 |

参考实现里 `ESCAPE_SUCCESS` / `ESCAPE_TIMEOUT` 这些 reason 字符串**没有任何对外输出**
（只在源码里作为 `changeState` 的第一个入参、且该函数 `(void)caller`）。本迁移把它们
落到了 diagnostics 的 `recovery_exit` 上。

### 7.1 RViz 里怎么看绿线

`FollowPath/recovery_markers` 加进 RViz 后，脱困期间会看到三个东西：

| 标记 | 含义 |
|---|---|
| **绿线**（`recovery_escape_path`） | **脱困路径规划的结果**：从当前位姿出发，沿脱困方向场一步步前推出来的轨迹 |
| 蓝箭头（`recovery_escape`） | 本周期实际下发的那一步：`方向 × escape_speed × window_sec` |
| 球（`recovery_clearance`） | 当前位置净空；颜色按 成功阈值 / 卡死阈值 分档 |

绿线**可能弯也可能直**，两种都不能直接判好坏：

* **直**：贴着墙/障碍时，ESDF 梯度就是墙面外法线，方向场沿路不变，路径自然是直线
  （本次用"紧贴左墙、净空 0.09 m"的算例验证过：路径严格沿 +x 单调推进，
  净空 0.09 → 0.19 → 0.29 → 0.39 → 0.44 后收尾）。
* **弯**：方向从梯度分支切到环形搜索分支、或环形搜索的最优方向随位置改变时，
  路径会随方向场折向。

现场判读方式：

* **绿线指向的是开阔方向** —— 梯度 / 环形搜索在工作，方向合理；问题在"车能不能按这个方向走"，
  去查底盘、限速、`escape_speed`；
* **绿线也是一条撞墙的直线** —— 距离场视角下确实没有出口（或出口在 `search_radius` 之外，
  环形搜索看不见），该查地图与膨胀半径，而不是继续调脱困参数；
* **绿线很短就断了** —— 走到了方向场给不出方向的区域（地图边界 / 全方向采样无效）。

⚠ **`traceEscapePath` 只看得见 `search_radius` 以内**：环形搜索只在半径
`search_radius`（默认 0.4 m）的圆周上采样，所以"0.4 m 之外有出口"这件事它探测不到。
这是参考实现 `calculateEscapeVelocity` 的固有局限，本迁移保持一致（没改语义）。
真遇到"绿线朝墙、车推不动"的现场，先怀疑这个半径而不是梯度本身。

实现细节（`RecoveryDecider::traceEscapePath`）：沿方向场**欧拉积分**，步长 0.1 m
（≈ 0.5 m/s @ 200 ms 一个控制周期走过的距离），步数上限
`ceil(2 × escape_speed × window_sec / 步长)`——按"窗口内理论可达距离"再留一倍余量，
避免画出一条窗口内根本走不完的线。净空一旦超过 `escaped_clearance` 就收尾
（再往前是正常导航该走的路，不属于脱困路径）。

绿线只在**脱困期间**发布（`updateRecovery` 判定本周期接管时调 `publishRecoveryDebug`），
退出脱困后 RViz 里会保留最后一条——与参考实现"离开 `RECOVERING` 时清可视化"不同，
见 §9 第 4 条。

---

## 8. 怎么在仿真里验证脱困

### 8.1 启动

```bash
./script/start_sim_nav.sh --controller minco
```

`--controller minco` 不能省：不给它时默认加载 Omni 那份参数文件，`recovery.*` 根本不在里面。
RViz 里已预置显示项 **`Recovery (MINCO)`**（话题 `FollowPath/recovery_markers`，相对名，
RViz 跑在 namespace 下会自动补全），不用手动加。

### 8.2 为什么需要专门造场景

脱困要**两个条件同时成立**：

```
条件 1  重规划连续失败 >= recovery.fail_threshold(5) 次
条件 2  机器人**自身**净空 < recovery.stuck_clearance(0.25 m)
```

条件 2 的净空是相对**原始致命栅格**算的，**膨胀层不计入**。而代价地图的膨胀层本来就不让车
靠近到 0.25 m 以内——所以"正常开车"几乎永远碰不到条件 2，手开车等它自己卡死会白等很久；
即使车被膨胀层裹住、规划一直失败，判的也是"前方路障"（只绕路，不脱困）。
**这就是本迁移最容易让人以为"脱困没生效"的地方。**

另外插件的参数只在 configure 期读一次，**没有 `onSetParameters` 回调**——
`ros2 param set ... FollowPath.recovery.*` 会成功返回但毫无效果（见 §3.4 的警告框）。

### 8.3 用 `script/inject_recovery_stuck.py` 确定性复现

脚本把致命障碍"注入"到机器人身上，一步造出两个条件。原理：订阅当前 `/map`
（transient_local），在机器人位置周围先清一块空地（否则真实地图自带的障碍会
"抢走"控制权——默认地图上出生点最近障碍在 0.65~0.73 m），再注入墙体，然后
transient_local 重发 `/map`；代价地图的 static_layer 下一个更新周期就并进来。

```bash
# 终端 A：起仿真
./script/start_sim_nav.sh --controller minco

# 终端 B：抓数据（只订阅、不控制车，事后可复盘）
python3 script/capture_minco_run.py

# 终端 C：注入（车还没动时可跳过 TF：出生时 map 系就是 (0,0)）
python3 script/inject_recovery_stuck.py --shape wall --wall-dir +y

# 验证完恢复现场（从注入前的自动快照恢复）
python3 script/inject_recovery_stuck.py --shape restore
```

脚本还会打印**注入后的净空**与**环形搜索预测的最开阔方向**，用来和 RViz 里的绿线对照。
本次实测标定（默认地图、车在 (0,0)）：

| `--shape` | 注入后车心净空 | 说明 |
|---|---:|---|
| `wall --wall-dir +y` | 0.115 m | ✅ 低于 0.25，能触发卡死；梯度方向 −90° |
| `wall --wall-dir -y` | 0.165 m | ✅ 梯度方向 +90° |
| `wall --wall-dir +x` | 0.165 m | ✅ 梯度方向 180° |
| `pocket --open-at +x` | 0.115 m | 三面围住，跑满窗口后超时退出 |

四个方向都实测核对过"墙在哪一侧 / 梯度指向哪一侧"（脚本内部统一用世界坐标矩形
`stamp_rect`，只有 `rect_from_world` 一处关心"行 0 = 最高 y"的行序）。

### 8.4 观察入口

```bash
ros2 topic echo /red_standard_robot1/FollowPath/diagnostics \
  | grep -E "recovery_(active|trigger|exit|clearance)"
```

期望的时间线：

1. 代价地图并进注入区 → `planning_result` 不再是 success；
2. 连续失败达门限 → 日志 `进入脱困（触发=..., 进入净空 ...）`、`recovery_active=true`；
3. RViz 出现绿线（脱困路径）+ 蓝箭头 + 净空球；绿线方向应与脚本预测的最开阔方向一致；
4. 走出去 → `recovery_exit=escaped`；跑满窗口 → `recovery_exit=timeout` 并回重规划。

### 8.5 两个读数陷阱

1. **一堵 1.2 m 的墙在 `search_radius` 0.4 m 的环形采样上是可以绕过去的**，
   所以环形搜索给出的方向会偏向墙的端头，而不是"正对墙的法向反方向"。
   想看到"正对背面"，用 `--wall-length 3.0`。这是参考实现 `calculateEscapeVelocity`
   的固有性质（本迁移保持语义一致），不是脱困逻辑的 bug。
2. **`/map` 是 latched 的**：注入脚本退出后，`map_server` 那份原始地图仍是
   "最近的发布者"之一，所以事后订阅 `/map` 未必能看到注入结果——要在**同一个进程里**
   订阅着观察，或者直接看代价地图/RViz。

---

## 9. 未完成 / 需运行期验证（本次仅静态阅读 + 编译 + 单测）

1. **未做仿真/实车运行验证**。以下都需要实测确认：
   * `escape_speed 0.5 m/s` 在实车底盘加速度斜坡下、3 s 窗口内实际能推出多远
     （理论 1.5 m，扣掉斜坡可能只有 1 m 量级）；
   * `stuck_clearance 0.25` 与实车 ESDF/膨胀半径组合是否会误判（把"能绕"判成"卡死"）；
   * 脱困期间旁路 MPC 会不会让 MPC 的热启动/差分状态出现可观测的跳变
     （已在 `leaveRecovery` 里清 `has_previous_input_`，但需实测确认无速度尖峰）。
2. **`BackUpFreeSpace` 走 ESDF 分支时每个 `onRun` 建一次距离场**。`global_costmap` 在实车
   是 200×200 量级（10×10 m / 0.05 m），估算 1~2 ms；未实测。若实测偏大，应改为
   缓存 / 增量或退化为只做环形搜索。
3. **`esdf_unknown_is_seed` 与控制器 `safety.unknown_is_obstacle` 必须人工保持一致**，
   目前是靠两处 yaml 各自声明、没有联动校验。后续可考虑启动时交叉检查。
4. **`publishRecoveryDebug` 只在脱困期间发布**，退出后不清 marker（RViz 里会留最后
   一帧箭头、绿线与球）。参考实现的 `clearRecoveryDebugVisualization` 是在离开 `RECOVERING`
   时清；本实现未做，因为 `KeepLast(1)` 的 marker 生命周期由 RViz 侧决定，且需要一次
   额外的 `DELETE` 发布。属于已知的小遗留。
5. **绿线是"预演"而非保证**：`traceEscapePath` 沿当前这帧距离场积分，不含底盘动力学
   （加速度斜坡、打滑）。所以绿线长度是"距离场意义上的可达"，实际位移会更短。
   它是判读工具，不能当轨迹用。
6. **`EMERGENCY_STOP` 语义未迁移**：参考实现的 `ENTER_EMER_STOP` /
   `publishEmergencyStop` 整块已被它自己注释（"Disabled to prevent zero-velocity deadlock"），
   所以本迁移也没有"先停再脱困"的路径——失败时要么推离、要么继续重试。
