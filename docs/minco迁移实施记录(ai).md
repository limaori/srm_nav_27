# MINCO + MPC 迁移实施记录（P0 / P1 / P2）

> 依据：[迁移minco实施方案(ai).md](迁移minco实施方案%28ai%29.md)（下称“方案”）。
> 本次范围：用户确认的 **P0 关键修复 + P1 数学核心与 ESDF + P2 低速 MINCO+MPC 闭环**。
> **P3–P6 未实施**（JPS 前端、报告式双次 MPC 的完整验收、SE(2) 与窄通道仲裁、实车辨识与全场回归）。
> 状态标记：【已实现】代码完成并通过编译与单元测试；【未验证】没有在 Gazebo/实车上跑过；
> 【未实施】本次没有做。

## 1. 交付物总览

| 包 | 作用 | 状态 |
|---|---|---|
| `src/srm27_navigation/srm27_minco_vendor` | GCOPTER 的 MINCO / Trajectory / root finder / L-BFGS 头文件（MIT，固定 commit） | 【已实现】 |
| `src/srm27_navigation/srm27_qpoases_vendor` | qpOASES QP 求解器（LGPL-2.1，固定 commit），编译为独立共享库 | 【已实现】 |
| `src/srm27_navigation/srm27_minco_core` | 无 ROS 依赖的算法核心：占用快照、二维 ESDF、MINCO 优化、轨迹验证、MPC、跟踪参考、重规划策略 | 【已实现】+ 单元测试 |
| `src/srm27_navigation/srm27_minco_controller` | Nav2 `FollowPath` 插件：状态/地图适配、后台规划线程、航向策略、诊断 | 【已实现】，【未验证】未在仿真/实车运行 |
| `srm27_navigation/sensor_scan_generation` | P0：odometry 语义修复（采样时间差、twist 坐标系、TF 失效标志） | 【已实现】 |
| `srm27_nav_bringup/config/simulation/nav2_params_srm_minco.yaml` | 仿真 MINCO 参数（新插件 + 统一限速 + StoppedGoalChecker） | 【已实现】，【未验证】 |
| `srm27_nav_bringup/config/real/nav2_params_srm_minco.yaml` | 实车 MINCO 参数 | 【已实现】，参数值**待实车辨识** |

依赖方向（与方案 §4.5.1 一致，未出现反向依赖）：
`controller → core → Eigen / GCOPTER 头 / qpOASES`；vendor 包不含 SRM 业务逻辑。

## 2. P0：前置修复

### 2.1 `sensor_scan_generation` 的 odometry 语义（方案 §2.3）

| 修正项 | 旧行为 | 新行为 |
|---|---|---|
| 差分时间基准 | 回调到达的 `steady_clock` 间隔 | **消息采样时间差** `t_k - t_{k-1}`；`steady_clock` 只保留给看门狗与耗时 |
| twist 坐标系 | 父系（`odom`）差分分量直接写入 `twist.twist.linear` | 按 `v_child = R(yaw)^T · (dp/dt)_parent` 旋转到 `child_frame_id` 后再写入 |
| 时间异常 | 无处理，重复/倒退时间戳会放大成速度尖峰 | 重复时间戳、时间倒退、间隔 > `max_sample_gap`、位姿跳变 > `max_position_jump`/`max_yaw_jump` 都不产生新速度 |
| TF 失败 | 退化为“返回单位变换”，伪装成有效定位 | `getTransform()` 返回 `bool` + 错误原因，失败时**跳过本次输出**；退化为“最新变换”时通过 `used_latest` 显式暴露并告警 |

新增参数（已写入四份 Nav2 配置）：`max_sample_gap`、`max_position_jump`、`max_yaw_jump`。

**仍属【未验证】**：真实 LIO/真值两条入口的频率、端到端延迟与时间戳一致性没有在运行系统上测量；
真值适配器（`simulation_ground_truth_odometry.py`）本身不在本次改动范围内。

### 2.2 速度链一致性（方案 §2.2）

- 仿真 MINCO 配置把 `FollowPath.limits`、`velocity_smoother` 上下界统一到 **平移合速度 0.5 m/s**（用户指定的测试起点），
  而 `srm_cmd_mux` 本来就是 `v_max: 0.5`，因此三层一致，正常运动不应再出现“上游要 2.5、执行端只给 0.5”的长期裁剪。
- 实车 MINCO 配置把 smoother 的 `min_velocity` 从 `[-2.5, -2.5, 0]` 改为对称的 `[-0.5, -0.5, -0.3]`，
  并在注释中写明：启用 SE(2) 阶段必须同步提高 yaw 界。
- `min_y_velocity_threshold` 在实车配置中由 0.5 调到 0.001（低速横移反馈不再被抹成零）。
- 目标检查器由 `SimpleGoalChecker` 改为 `nav2_controller::StoppedGoalChecker`（方案 §7.4：终点必须按停稳验收）。

**[未实施]**：`srm27_chassis_control` 的 mux 模式改造（`INDEPENDENT`/`NAV_SE2`/`RECOVERY`/`IDLE_ROTATION`/`STOP`）
属于 P4 的 SE(2) 阶段。本次阶段一 `yaw_policy.mode = xy_only`，插件输出的 `angular.z` 恒为 0，
与现有 mux“只采用 `rotation_velocity.angular.z`”的行为**天然兼容**，不存在双重自转或角速度所有权冲突。

## 3. P1：数学核心与 ESDF

### 3.1 二维占用快照与 ESDF

- `GridSnapshot`：与 costmap 解耦的行优先占用快照，单元语义只有 `kFree / kOccupied / kUnknown`。
- `CostmapAdapter` 由 costmap 复制时，**只有 `LETHAL_OBSTACLE` (254) 是原始障碍**，
  `NO_INFORMATION` (255) 记为未知；膨胀层产生的 253 及以下的代价值**不写入原始占用**，
  机器人半径只在碰撞校验里单独计入，避免“半径重复计入、通道被人为变窄”（方案 §6.1、§13 P1 风险）。
- `Esdf2D`：两遍精确 EDT（Felzenszwalb–Huttenlocher），分别计算到障碍与到自由区的距离平方，
  符号由占用语义给出；米制转换 `sqrt(d²) · resolution` 后再减去半格对角线做**保守化**；
  双线性插值与其解析导数一致；地图外返回 `valid = false`；无任何种子时返回 `no_obstacle_distance` 且梯度为零。
- 地图版本号由**内容哈希**决定，只有内容真正变化才递增：滚动地图每帧都在动，
  若按帧递增，“地图版本相等”的提交检查会永远失败（方案 §4.4）。

### 3.2 MINCO 表达与优化

- 使用上游 `minco::MINCO_S3NU`（最小 jerk、五次多项式、非均匀时间），z 固定为 0，只优化 x/y 与段时长。
- 外部 `Trajectory2D` 的系数顺序固定为 **c0→c5**（方案 §5.3），与 GCOPTER 的 `CoefficientMat` 行序（最高阶在前）
  通过显式翻转 + 往返测试隔离。
- 目标函数（方案 §6.4）：
  `w_jerk·∫‖jerk‖²` + `w_time·ΣT_i` + 段内 midpoint 数值积分的障碍/速度/加速度 softplus-hinge 软约束
  + 可选的参考吸引项；PRE 阶段额外加入 `T_i/mean(T)` 的比例惩罚（**对 mean(T) 求导**，不当作常数）。
- 段时长用 `T = Tmin + softplus(s)` 参数化，保证严格为正并防溢出。
- 梯度完整回传：路标点与段时长的导数都包含**积分权重与采样时刻对时长的依赖**
  （`∂v/∂T = -v/T`、`∂a/∂T = -2a/T`），再经 `propogateGrad()` 得到 `∂J/∂q`、`∂J/∂T`，最后按 `dT/ds = sigmoid(s)` 链式回到自变量。
- 报告式 FINELY 通道启发式（切向梯度剔除 + 方向探测 + 斜率阈值）已实现但**默认关闭**，
  并在头文件中明确标注它破坏“代价与梯度严格一致”的关系（方案 §6.5）。

### 3.3 前端初值与独立验证

- `TrajectoryInitializer`：清洗重复点/共线抖动/短小折返 → 弧长重采样 → 转角降速 →
  前向加速扫描 + 后向制动扫描 → 积分时长（两端速度均为零的短段用三角形解，避免除零）→
  近似等时间间隔取路标点，并限制最小/最大段时长与段数。
- `TrajectoryValidator`：系数/时长有限性、边界与段间 p/v/a 连续性、**多项式求根**得到的
  速度/加速度极值（`‖v‖²` 是 8 次多项式，求其导数实根即极值点，再与端点比较；另加 200 点加密采样兜底）、
  全段扫掠净空（采样步长同时受时间步长与最大空间间距约束，避免跳过薄障碍）、
  有效前缀长度与“MPC 窗口 + 停车时间”的覆盖比较、地图/轨迹时效与末端静止检查。
  优化器返回成功**不等于**轨迹可执行，发布前必须过这一关（方案 §6.6）。

## 4. P2：低速 MINCO + MPC 闭环

### 4.1 线程与数据一致性（方案 §4.4）

```text
控制线程（controller_server 调用 computeVelocityCommands，50 Hz）
  ├─ 取一致状态（odometry 采样时间戳 + 单调接收时刻）
  ├─ costmap 锁内复制快照 → 锁外建立 ESDF
  ├─ 取回后台结果 → 版本校验 → 提交为不可变轨迹快照
  ├─ 已提交轨迹在最新地图上复验
  ├─ 提交重规划请求（队列长度 1，忙时只保留最新）
  ├─ 跟踪参考（进度投影 + 单周期进度限幅）→ 单次/双次 QP
  └─ 输出前检查 → 返回车体系 TwistStamped（唯一速度出口）

后台线程（PlanningWorker，默认 10 Hz 事件/周期触发）
  └─ 局部路径裁剪 → 前端初值 → MINCO（PRE + FINELY）→ 独立验证 → 产出不可变轨迹
```

- 工作线程**不发布任何速度**，也没有自己的定时器；`deactivate()` 先 `stop()`（join）再作废轨迹，
  退出后不会回写结果。
- 规划任务携带 `goal_epoch / path_version / map_version / limits_version`，迟到的旧会话结果被丢弃并计数
  （`rejected_stale_result_count`）。
- 轨迹快照用 `std::atomic_store/load` 的 `shared_ptr` 交换，控制端拿到后无需持规划锁。

### 4.2 MPC（方案 §7.1–§7.4）

**求解器性能与热启动。** 首次实测发现：N=30、K=8、含输入差分约束时，每周期新建
`QProblem` 冷启动约 **7.5~8.5 ms**（P99 约 9 ms），在 50 Hz（20 ms 周期）下占用过大。
由于 H 与约束矩阵在运行期固定不变、每周期只有梯度与上下界变化，这正是 qpOASES
`hotstart()` 的适用场景，因此：

* 新增 `mpc.use_hot_start`（默认 `true`）：复用持久求解器并热启动，失败时自动回退一次冷启动；
  实测热启动平均 **0.48 ms**、P100 **0.73 ms**（100 次连续求解，N=30）。
* 约束行数改为**恒定**（没有上一条执行命令时，把首步差分限值放宽到全速域，而不是少发两行）。
  否则每次提交新轨迹都会让行数变化，从而每次都要重建并冷启动。
* 在 `activate()` 里 `warmUp()` 预热，把冷启动（约 8~15 ms）挪出控制回调。
* `qp_deadline_ms` 改为**性能门槛而非正确性门槛**：超过只累计 `missed_deadline_count` 并告警，
  **仍然使用该解**（它已通过有限性与硬约束校验）。丢弃一个合法解只会让机器人无故停下。
* 命令连续性约束的对象是“上一条实际执行的命令”，与轨迹编号无关：提交新轨迹时**不再**清
  `has_previous_input_`，只在新目标、取消、限速变化、定位重置与失活时清空。

**会话编号（goal_epoch）。** `setPlan()` 现在区分“新目标”与“同一目标的 3 Hz 路径刷新”：
终点位置变化超过 1 mm 视为新目标，开启新会话（递增 `goal_epoch`、复位重规划状态、作废轨迹）；
否则只递增 `path_version` 并保留当前轨迹与控制进度。

**[已知限制]** 取消后重新下发**完全相同坐标**的目标会得到同一个会话编号，无法与普通路径刷新
区分（Humble 的 `nav2_core::Controller` 没有 Action cancel 回调，真正的会话守护属于方案 §8.3 的 P4）。
实际安全性不受影响：插件没有独立的速度定时器，取消后 controller_server 不再调用
`computeVelocityCommands`，`srm_cmd_mux` 的 0.3 s 看门狗随后清零。

**MPC 的角速度权限。** `xy_only` 模式下，`yaw_policy.angularSpeedLimit()` 返回 0，并**真正**作用于
求解器的角速度上下界；否则求解器虽然参考为 0，仍保留 0.3 rad/s 的自转权限，与“阶段一 MPC 不控
自转、自转由独立链路负责”的契约不一致。


- 阶段一基线：`velocity_integrator`，`z = [x, y, yaw]`（odom 系）、`u = [vx, vy, ω]`，`z(k+1) = z(k) + h u(k)`。
  六维 `acceleration_double_integrator` 也已实现（`z=[x,y,ψ,vx,vy,ω]`、`u=[ax,ay,α]`、零阶保持 `0.5h²`/`h` 分块），
  通过参数 `mpc.model` 显式选择，不与速度层混用同一 B 矩阵。
- 目标与梯度按方案 §7.2 的 `H = 2(SuᵀQ̄Su + R̄ + DduᵀS̄Ddu)`、`g = 2(SuᵀQ̄(Sx z0 − Zref) − R̄Uref − DduᵀS̄ d_prev)`
  实现，两边的系数 2 一致。
- **已修正方案 §3.5 列出的移植问题**：
  1. 横纵向权重按 `Qxy = qa·t tᵀ + qc·n nᵀ` 构造（交叉项 `(qa−qc)·cosθ·sinθ`），
     并有 `θ=π/4` 下 `tᵀQt=qa`、`nᵀQn=qc`、`tᵀQn=0` 的单元测试；
  2. 首步输入差分使用**真实控制间隔**与上一条实际执行的命令，后续步才用模型步长；
  3. 合速度用保守内接正多边形 `n_jᵀv ≤ v_max·cos(π/K)`（默认 K=8）约束，
     不再只限制 `|vx|,|vy| ≤ v_max`（那会允许 `√2·v_max`）；
  4. 求解结果检查有限性、约束违背量、耗时预算与迭代次数，并区分不可行/超时/数值失败；
  5. 提供 `reset()` 接口，新目标/取消/失活时清除输入基准与热启动。
- 输出：先按固定时间步采样参考，QP 求解后用 `command_lookahead`（默认 0，未标定前关闭）
  从预测序列**线性插值**取执行时刻速度，再按**同一执行时刻的预测 yaw** 转到 `base_link`，
  最后检查有限性、合速度、角速度与相对上一条命令的变化量。

### 4.3 航向与自转所有权（方案 §8.1 阶段一）

- `yaw_policy.mode = xy_only`：航向参考恒等于当前连续航向、角速度参考恒为 0，
  且 `MpcSolver` 的角速度上下界与角速度参考都为 0 —— MPC **明确没有**自转权限。
- 自转仍由现有 `rotation_test_sender → rotation_controller → srm_cmd_mux` 链路负责，插件不与之叠加。
- `follow_tangent` / `spin` / `narrow_track` 三种模式已实现（含跨 ±π 的连续展开、反向切向选择、
  SPIN 需显式请求、请求撤销后按停止处理），但**不属于本次验收范围**，未做窄通道现场验证。

### 4.4 失败与停止语义

- 无状态 / 无地图 / 无通过验证的轨迹 / QP 失败时，返回**受控零速度**并开始计时；
  超过 `build_grace_period`（默认 1.0 s）仍无法恢复则抛出异常，让 BT 进入恢复行为，
  而不是永远返回零并假装正常（方案 §4.4）。
- 里程计重置（位姿跳变计数变化）→ 立即作废轨迹、热启动与参考进度。
- `setSpeedLimit` 支持 Nav2 的三种语义（绝对、百分比、0 表示解除限制），
  递增 `limits_version`，并在已有轨迹超出新上限时**立即作废**它（不允许继续执行）。

## 5. 构建、测试与运行

### 5.1 构建与单元测试

```bash
cd /home/srm/srm_nav_27
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-up-to srm27_minco_controller srm27_nav_bringup \
  --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
colcon test --packages-select srm27_minco_core srm27_minco_controller
colcon test-result --verbose
```

单元测试覆盖（方案 §11.1 的“数学与接口测试”行）。当前状态：**14 个 gtest 套件、200 个用例全部通过**，
`clang_format`（49 个受维护文件）与 `copyright` 检查通过：

| 文件 | 覆盖内容 |
|---|---|
| `test_kinematics.cpp` | 角度归一化/展开、坐标系旋转、限幅、制动距离、非有限输入 |
| `test_smooth_cost.cpp` | softplus / sigmoid / hinge 的值与**解析导数对中心差分** |
| `test_esdf_2d.cpp` | EDT 与暴力基准对照、单格墙、斜缝、未知/边界、解析圆障碍、梯度中心差分 |
| `test_trajectory_2d.cpp` | 系数顺序（含高阶在前往返）、段定位、连续性、切线、元数据校验 |
| `test_trajectory_initializer.cpp` | 清洗/重采样、时间分配、折角降速、边界与经过点 |
| `test_minco_optimizer.cpp` | **解析梯度对中心差分**、边界/经过点/连续性、时间优化、障碍改善、非法输入 |
| `test_trajectory_validator.cpp` | 多项式求根极值（含段内峰值）、碰撞扫描、时效与覆盖检查 |
| `test_mpc_solver.cpp` | 两种模型的物理关系、凝聚矩阵一致性、QP 解析解、多边形合速度约束、首步差分、`planarWeight` |
| `test_tracking_reference.cpp` | 进度投影与限幅、回折不跳分支、终点收敛、alpha 第二次采样、yaw 连续性 |
| `test_replan_manager.cpp` | 触发条件、版本校验、`canCommit`、状态跳变 |
| `test_state_adapter.cpp` | **集成测试**（真实节点/话题/TF）：采样时间差 vs 到达时间差、车体系 twist 旋转、乱序采样丢弃、跳变计数、超时、缺 TF 时明确失效 |
| `srm27_minco_controller/test/*`（其余） | pluginlib 导出/实例化、航向策略、诊断字段完整性 |

### 5.2 运行（阶段一，XY 模式）

```bash
PARAMS_FILE=/home/srm/srm_nav_27/src/srm27_navigation/srm27_nav_bringup/config/simulation/nav2_params_srm_minco.yaml \
ROTATION_MODE=stop \
USE_VELOCITY_SMOOTHER=False \
bash script/start_sim_nav.sh
```

启动后必须回读确认插件确实被加载，并检查可视化与诊断：

```bash
ros2 param get /red_standard_robot1/controller_server FollowPath.plugin
# 期望输出：srm27_minco_controller::MincoMpcController
ros2 topic echo /red_standard_robot1/FollowPath/diagnostics --once
ros2 topic hz  /red_standard_robot1/FollowPath/minco_trajectory
ros2 topic hz  /red_standard_robot1/FollowPath/mpc_prediction
```

**本次没有执行以上启动步骤**：没有跑 Gazebo，也没有实车验证。默认启动链仍然是 Omni 配置，
`nav2_params_srm.yaml` 保留为 A/B 对照与回退（方案 §12.3）。

### 5.2.1 首次仿真联调检查清单

按顺序确认，任一步不符就不要继续往下看：

1. **插件真的被加载**（不是只有 launch 没报错）：
   `ros2 param get /red_standard_robot1/controller_server FollowPath.plugin` → `srm27_minco_controller::MincoMpcController`
2. **状态链通**：`ros2 topic hz /red_standard_robot1/odometry` 有稳定频率；
   `ros2 topic info` 确认只有**一个** `odom -> base_link` 发布者（仿真真值或 LIO，不能同时两个）。
3. **规划产出轨迹**：`ros2 topic hz /red_standard_robot1/FollowPath/minco_trajectory` 有数据；
   RViz 里能看到 MINCO 曲线与 MPC 预测轨迹。
4. **诊断健康**：`ros2 topic echo /red_standard_robot1/FollowPath/diagnostics --once`
   关注 `planning_result=success`、`planning_time_ms`、`minimum_clearance>0`、
   `state_age` 与 `map_age` 都很小、`missed_deadline_count` 不持续增长。
5. **速度层一致**：`cmd_vel_nav`、`cmd_vel_sim` 的 `wz` 恒为 0（阶段一），
   合速度不超过 0.5 m/s，且 `cmd_vel_sim` 没有被 `srm_cmd_mux` 长时间限幅。
6. **急停可用**：`ros2 service call /red_standard_robot1/srm_cmd_mux/stop_all std_srvs/srv/Trigger '{}'`
   之后 `cmd_vel_sim` 确实为零、机器人停住；解除锁存后再继续。

出现下列现象时，先看诊断再调参，不要先改权重：

| 现象 | 首先检查 |
|---|---|
| 机器人不动，日志出现 “no validated trajectory available yet” 后转为控制器异常 | `planning_result` / `reason`：是 `no_path`（局部路径被裁空）、`validation_failed`（净空或覆盖不足）还是 `optimize_*` |
| 每周期都出现 “QP 求解 ... 超过预算” | `use_hot_start` 是否为真、`activate()` 的预热是否成功 |
| 机器人频繁走走停停 | 每次 `setPlan` 是否被判成“新目标”（终点是否抖动）、`build_grace_period` |
| 轨迹正常但输出速度为 0 | `yaw_policy.mode`、状态是否被判失效（`state_age`）、`trajectory_age` 是否超 `trajectory_max_age` |

### 5.3 方案 §4.5.5 审阅门槛自检

| 门槛 | 自检结果 |
|---|---|
| 依赖方向正确 | core 不包含 rclcpp/Nav2/TF；controller 依赖 core；无反向依赖 |
| 无旁路执行出口 | 插件只返回 `TwistStamped`，不发布速度话题、不直连串口/Gazebo；后台线程只产出轨迹 |
| 坐标/时间/停止契约一致 | odom 系规划、车体系输出；状态用采样时间戳、看门狗用单调时钟；停止路径统一走 `stopCommand()` |
| 参数唯一 | 每项参数在 YAML、`declare/get` 与注释中同名同义；未维护多套互相覆盖的入口 |
| 许可证/来源完整 | 两个 vendor 包各带 LICENSE 与 `THIRD_PARTY_NOTICES.md`，记录 URL、commit 与排除文件原因 |
| 改动只覆盖当前任务 | 未重排旧模块、未批量改名、未全仓格式化；只格式化了本次新增/修改的受维护文件 |
| 对应检查通过 | `colcon build` + `colcon test` 通过（200 用例），clang_format/copyright 通过；**未编译进仿真、未实车验证** |

## 6. 实施过程中发现并修复的真实缺陷

除了按方案照做，本次实现（含专门为数学核心与状态链编写的测试）定位并修复了以下真实缺陷。
它们都**不是**文档里预先列出的项，因此单独记录：

| 位置 | 缺陷 | 影响 | 处理 |
|---|---|---|---|
| `minco_optimizer.cpp` 段内求值 | 采样点速度/加速度被错误地除以 `T`、`T²`，把 MINCO 的**绝对局部时间**系数当成“归一化时间”系数 | 速度/加速度软约束阈值整体失效（`T≈0.5` 时相当于放宽 2 倍与 4 倍），且代价与交给执行端的轨迹互相矛盾 | 改为 `v = p'(τ)`、`a = p''(τ)`，并补上 `jerk = p'''(τ)`；用尾点位置/速度与 `getEnergy` 交叉验证 |
| `minco_optimizer.cpp` 时间梯度 | `∂J/∂T` 既沿用了错误的归一化约定，又**漏掉积分权重里的 `T` 因子** | 时间梯度残留 0.2%~0.7% 系统偏差；用合成多项式定点差分定位（解析 95.28 vs 差分 45.97）后修正 | 重新推导为 `∂J/∂T_i = Σ_j (1/n)L_j + Σ_j (T_i/n)[∂L/∂p·f_j v_j + ∂L/∂v·f_j a_j + ∂L/∂a·f_j jerk_j]` |
| `minco_optimizer.cpp` 参考吸引项 | 该项代价只在“需要梯度”的分支累加 | `runStage` 末尾用“只求值”分支算最终代价，与 L-BFGS 实际优化的目标不一致 | 代价统一累加，梯度单独补 |
| `minco_optimizer.cpp` `setWarmStart` | 长度校验要求 `内点数 == 段数`，而 `optimize` 要求 `内点数 == 段数 - 1` | 两者矛盾 ⇒ **热启动永远是死代码** | 按头文件契约改为 `内点数 + 1 == 段数` |
| `minco_optimizer.cpp` 输入校验 | `std::max(Tmin, NaN)` 会返回 `Tmin`，非有限段时长被静默替换 | 非有限输入没有按 `kInvalidInput` 语义被拒绝 | 在 `optimize`/`evaluateObjective` 显式拒绝 NaN/inf |
| `state_adapter.cpp` 差分回退路径 | 把 odom 系位姿差分先转车体系、再按**当前** yaw 转回 odom（`R(ψ_k)·R(ψ_{k-1})^T`） | 额外引入一个 `Δψ` 的错误旋转分量 | 直接用 `Δp/Δt` 作为 odom 系速度（平面刚体的正确定义） |
| `state_adapter.cpp` 时间戳乱序 | 乱序采样会覆盖更新位姿并把差分基准搞负 | 可能算出方向相反的速度，或让状态被判为不可用 | 时间戳不新于当前最新采样时**直接丢弃**并计数（`droppedSampleCount()`） |
| `grid_snapshot.cpp` `reset()` 失败路径 | 只把 `valid_` 置假，旧的尺寸与单元仍留在对象里 | `cellCount()/sizeX()/containsCell()` 会继续暴露上一次快照，与头文件“保持为空”的契约不一致 | 失败时清空尺寸与单元 |

梯度检查的实测（方案 §11.1 要求，容差未放宽）：位置分量最大相对误差 **2.8e-9 ~ 8.9e-9**，
时间分量 **8.7e-9 ~ 8.9e-9**（无 ESDF / 有 ESDF / 段时长比例惩罚 / 参考吸引四种场景）；
报告式启发式模式下 **6.8e-3**，测试明确断言“此模式下梯度与差分不一致是预期行为”。

## 6.1 首次仿真运行暴露的问题（已修复）

第一次真机（Gazebo）联调时，日志给出的结论是：**迁移链路本身是通的，卡在一处元数据 bug 上**。

日志证据（`~/.ros/log/controller_server_*.log`）：

```text
[INFO] Created controller : FollowPath of type srm27_minco_controller::MincoMpcController
[INFO] MincoMpcController 已配置: planning_frame=odom base_frame=base_link horizon=2.00 m
       replan=10.0 Hz v_max=0.50 m/s a_max=0.30 m/s^2 yaw_mode=xy_only
[INFO] MincoMpcController: QP 求解器预热完成
[INFO] MincoMpcController: 新导航会话 #1 (路径版本 1)
[WARN] MincoMpcController: 规划失败 (validation_failed): trajectory coefficients/durations/continuity check failed
[ERROR] MincoMpcController: no validated trajectory available yet      ← ×68
[WARN] [follow_path] [ActionServer] Aborting handle.                   ← ×68
```

即：插件加载、参数读取、QP 预热、会话编号、Theta* 全局规划**全部正常**，MINCO 也已经产出了
候选轨迹，只是候选轨迹在**独立验证**这一步被拒。原因是 `PlanningWorker` 把
`valid_until` 留成了 `0`，而 `generated_stamp`/`valid_after` 是当前 ROS 时间：

| 位置 | 缺陷 | 影响 | 处理 |
|---|---|---|---|
| `planning_worker.cpp` | `valid_until = 0.0`，而 `valid_after = request_stamp`（约 1.79e9） | `Trajectory2D::sanityCheck()` 判定“有效期早于生效时刻”，**每一条候选轨迹都被拒**，控制器永远拿不到轨迹 → FollowPath 反复 abort → BT 进入恢复行为（日志里能看到两次 BackUpFreeSpace） | 增加 `PlanningRequest::validity_window`，由控制器的 `trajectory_max_age` 填入，`valid_until = request_stamp + validity_window` |
| `trajectory_2d.cpp` | `sanityCheck()` 把 `valid_until == 0` 当作“有效期早于生效时刻” | 两个模块对 `0` 的语义不一致（`TrajectoryValidator` 里 `0` 表示“不检查过期”） | 统一为“`0` = 未设置有效期”，只有设置了非零有效期时才检查顺序 |
| `trajectory_validator.cpp` | 自检失败时把具体原因丢掉，只报 `coefficients/durations/continuity check failed` | 现场无法判断是时长、系数、连续性还是时间元数据的问题，只能靠猜 | 把 `sanityCheck` 的具体原因拼进 `reason` |

并补了回归测试 `Trajectory2DSanityCheck.UnsetValidityWindowWithFutureValidAfter_Passes`，
用例注释直接写明这个失败模式的现场表现。测试总数 **368**，全部通过。

顺带澄清一处**看起来像问题、实际不是**的现象：`/odometry` 只有 `simulation_ground_truth_odometry`
一个发布者，启动脚本已经按 `has_odometry_source` 把 `sensor_scan_generation` 的 `odometry`
remap 到了 `sensor_odometry`（该话题当前 0 个订阅者），不存在两个 odometry 发布者抢状态的问题。

## 7. 未完成项与后续阶段

| 阶段 | 内容 | 状态 |
|---|---|---|
| P3 | JPS 局部前端、报告式 `enable_report_fine_heuristic` 的实测标定、保留安全前缀的重规划（`kPrefixReuse` 目前只在策略层判定，未实现前缀拼接求解） | 【未实施】 |
| P4 | 报告式双次 MPC 的完整验收（代码路径已有 `two_pass_reference`，默认关闭且未实测）、mux 的 `NAV_SE2` 等五模式仲裁、窄通道 `APPROACH_ALIGN/NARROW_TRACK/EXIT_HOLD` 状态机、Action 会话守护 | 【未实施】 |
| P5 | 实车入口 `nav_srm_real_launch.py`、延迟/制动/外形的实车辨识、速度内环参数档案 | 【未实施】；当前实车 MINCO YAML 中的加速度、制动减速度、延迟均为**占位建议值** |
| P6 | 全场回归矩阵、性能报告、默认配置切换、移除 Omni 依赖 | 【未实施】 |

其它已知缺口（诚实记录，不当作已完成）：

1. **下游实际执行命令反馈**：`previous_applied_input_` 记录的是**插件自己返回的命令**，
   不是经过 smoother/mux 限幅后的真实执行值。方案 §3.5 修正项 3 要求的“applied-command”
   需要下游反馈通道，属于 P4。
2. **旧轨迹前缀复用**：`ReplanType::kPrefixReuse` 已定义并参与策略判定，
   但工作线程目前一律做完整（或热启动）优化，没有实现“保留安全前缀 + 只重规划后半段”的求解。
3. **窄通道几何**：阶段一使用圆形 0.33 m 包络，`yaw_policy.narrow_track` 的净空阈值只是占位；
   把雷达移到大 yaw 后的活动外参适配（方案 §5.5）完全没有实施。
4. **仿真/实车验证**：本记录中的所有“已实现”都只到“编译 + 单元测试”为止，
   没有一条闭环轨迹在 Gazebo 或实车上跑过。P2 的验收场景（空场前进/横移/斜移/停止/抢占）
   全部未执行。
5. **TF 无超时查询的日志噪声**：`StateAdapter` 在 `planning_frame != odom` 且变换不可用时会
   调用 tf2 的 `lookupTransform`（超时为 0，不会阻塞），tf2 自身会打印一条 ERROR 级
   “需要专用线程”提示。正常配置下 `planning_frame == odom`，不会走到这条路径；
   若后续确需跨系查询，应当在控制器侧自行管理 TF 线程模型再处理这条日志。
