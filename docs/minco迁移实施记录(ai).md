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

- 仿真与实车的有效限速已统一到**实车现有链路**（详见 §6.3）：平移 1.5 m/s、加速度 3.0 m/s²、
  角速度 1.0 rad/s；`FollowPath.limits`、`velocity_smoother`、`srm_cmd_mux` 三层用同一组值，
  正常运动不应再出现“上游要 1.5、执行端只给 0.5”的长期裁剪。
- 实车 MINCO 配置把 smoother 的 `min_velocity` 从 `[-2.5, -2.5, 0]` 改为与 `max_velocity` 对称的值，
  并在注释中写明：启用 SE(2) 阶段必须同步放开 yaw 界。
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
`clang_format`（49 个受维护文件）与 `copyright` 检查通过；另有 **2 个 Python 几何回归套件
（15 个用例）**，见 §6.8.4：

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

## 6.2 第二次仿真运行：软约束权重未标定导致候选轨迹被一律拒绝（已修复）

修掉 `valid_until` 之后，第二次运行换了一个失败点：

```text
[WARN] MincoMpcController: 规划失败 (validation_failed): max speed exceeds the effective limit   ×2
[ERROR] MincoMpcController: no validated trajectory available yet                                ×22
[WARN] [follow_path] [ActionServer] Aborting handle.                                             ×22
```

### 根因（离线复现的数据）

用一个与仿真一致的算例离线复现（2 m 局部路径、`v_max=0.5`、`a_max=0.3`、静止起步、
`horizon=2.0`、`nominal_piece_duration=0.3`），变更权重扫描：

| `w_velocity` | `w_acceleration` | 最大速度（上限 0.50） | 最大加速度（上限 0.30） | 总时长 |
|---:|---:|---:|---:|---:|
| 20 | 2（**原配置**） | **0.614 超** | **0.637 超** | 4.62 s |
| 60 | 20 | 0.488 | 0.446 超 | 5.54 s |
| 60 | 150 | 0.487 | 0.312 超 | 6.18 s |
| **60** | **300（新配置）** | **0.480** | **0.276** | 6.48 s |
| 100 | 500 | 0.453 | 0.253 | 6.83 s |

MINCO 的速度/加速度是**软约束**，其最优点由 `w_time`（把速度往上拉）与 `w_velocity`/
`w_acceleration`（往下压）的平衡决定。原配置下平衡点落在 0.61 m/s 与 0.64 m/s²，
**两条硬上限同时被越过**；而 `TrajectoryValidator` 按硬上限判定，于是每一条候选轨迹都被
丢弃 → 控制器永远拿不到轨迹 → FollowPath 反复 abort → BT 反复 recovery（用户看到的“20 多次”）。
这与方案 §13 里那条 P1 风险是同一件事：**上游一直产出一个下游永远不接受的速度**。

### 同时发现的收敛问题

原配置 `lbfgs_past = 0`（关闭基于函数值下降率的判据），只剩梯度阈值这一个停止条件，
而这个问题的量纲跨度极大，梯度阈值根本达不到 —— 实测**两个阶段都跑满迭代上限**
（302/302，耗时 17 ms）。打开 `lbfgs_past = 3` 后 **52 次迭代收敛、耗时 3.1 ms**，结果质量不变。

### 处理

| 位置 | 改动 |
|---|---|
| 两份 MINCO 配置 | `w_velocity: 20 → 60`、`w_acceleration: 2 → 300`（附上实测依据的注释） |
| `minco_optimizer` | 新增 `lbfgs_past` 配置并接通到 L-BFGS 的下降率判据；配置默认值 `w_velocity/w_acceleration` 同步提高 |
| `minco_optimizer` | 新增 `rescaleDurations()`：按倍数拉伸段时长并用**同一组内部路标点与首末边界条件**重建轨迹（位置几何基本不变，速度 ≈ /k、加速度 ≈ /k²） |
| `planning_worker` | 优化成功后先算极值：越限则做最多 2 次确定性时间拉伸修复，再进入验证；修复次数与累计倍数进入诊断（`speed_repair_count` / `time_scale`） |
| `minco_mpc_controller` | **失败路径也发布 `diagnostics`**（之前只有成功路径发布），现场可以直接 `ros2 topic echo .../diagnostics` 看原因 |
| `trajectory_validator` | 越限报错带上实测值与门槛（`max speed 0.614000 exceeds the effective limit 0.510000`），不再只说“超速” |

修复的兜底效果（离线实测）：

| 场景 | 优化输出 | 拉伸修复后 |
|---|---|---|
| 原权重 20 / 2 | 0.614 / 0.637 | 1 次 ×1.457 → **0.421 / 0.300** |
| 极端坏权重 5 / 1 | 0.976 / 0.839 | 1 次 ×1.951 → **0.500 / 0.220** |

新增回归测试 `MincoOptimizerRescale.PreservesBoundariesAndReducesExtrema`
（边界 p/v/a 不变、经过点不变、极值按 1/k 与 1/k² 下降、非法入参被拒）。测试总数 **369**，全部通过。

### 参数含义提醒

`limits.max_linear_accel = 0.30 m/s²` 是**很保守**的值：2 m 的局部路径在
`v_max=0.5` 下的时间最优梯形就需要约 5.7 s，而现在是 6.5 s。若希望动作更果断，
应当先做制动能力辨识，再提高 `max_linear_accel`（方案 §10 P5），而不是靠调软约束权重。

## 6.3 限速对齐：仿真 = 实车（本次改动）

**实车实际链路**（`script/start_real_nav.sh`）：

```text
controller_server ─cmd_vel_controller─► velocity_smoother ─cmd_vel_nav2_result─┐
                                                fake_vel_transform ─► cmd_vel_chassis ─► srm27_nav_protocol ─► 串口
```

实车**不使用 `srm_cmd_mux`**，因此其有效限速由 smoother 与串口协议决定：
smoother `max_velocity [1.5, 1.5, 0.0]`、`max_accel [3.0, 3.0, 0.0]`（yaw 钳 0），
串口 `max_vx/vy 2.5`、`max_wz 1.0` ⇒ **有效平移 1.5 m/s、有效导航 yaw 0**。

仿真链路是 `controller → cmd_vel_nav/cnt_vel_controller → srm_cmd_mux → cmd_vel_sim → Gazebo`，
唯一的限幅点是 mux，而它原本是 `v_max: 0.5` —— 这就是“仿真和实车不一样”的来源。

对齐后的结果：

| 层 | 实车 | 仿真（改前） | 仿真（改后） |
|---|---|---|---|
| controller 平移上限 | Omni `v_linear_max 1.5` | Omni `2.5` / MINCO `limits 0.5` | Omni `1.5` / MINCO `limits 1.5` |
| controller 平移加速度 | smoother `3.0` | MINCO `limits 0.3` | MINCO `limits 3.0` |
| controller 角速度 | Omni `v_angular_max 1.0`（smoother 再钳 0） | Omni `3.0` / MINCO `0.3` | Omni `1.0` / MINCO `1.0` |
| `velocity_smoother` | `[1.5,1.5,0.0]` / `[3.0,3.0,0.0]` | `[2.5,2.5,3.0]` / `[4.5,4.5,5.0]`（Omni）、`[0.5,0.5,0.3]` / `[1,1,1]`（MINCO） | 全部 `[1.5,1.5,0.0]` / `[3.0,3.0,0.0]` |
| `srm_cmd_mux` | 不使用 | `v_max 0.5`、`wz_max 2.0` | `v_max 1.5`、`wz_max 1.0` |
| 串口协议 | `2.5 / 2.5 / 1.0` | 不涉及 | 不涉及 |

**同时必须重新标定 `w_jerk`。** 限速提高后，原来按 0.5 m/s 档位看起来正常的 `w_jerk = 1.0`
会严重压制速度：最小 jerk 目标下，2 m 起停实测只跑到 **1.116/1.5 m/s、耗时 3.36 s**，
而该场景的理论时间最优是 1.83 s —— 也就是说仿真车会以 74% 的限速跑，无法代表实车行为。

| `w_jerk` | 最大速度（上限 1.5） | 最大加速度（上限 3.0） | 2 m 起停耗时 |
|---:|---:|---:|---:|
| 1.0（改前） | 1.116 | 1.023 | 3.36 s |
| **0.2（改后）** | **1.438** | **1.721** | 2.59 s |
| 0.05 | 1.469 | 2.342 | 2.27 s |

取 `w_jerk = 0.2`：速度用到 96% 的上限，加速度只用掉 57%（留出避障余量），
并且在低限速档（0.5/0.3）下仍然达标（0.481/0.289）——两档都验证过。

**停车距离复核**（方案 §6.1 的 `d_stop = v·latency + v²/(2a) + 外形`）：
1.5 m/s、latency 0.1 s、a = 3.0 m/s²、包络 0.33 m ⇒ `0.15 + 0.375 + 0.33 ≈ 0.86 m`，
仍小于 `planning_horizon = 2.0 m`，覆盖检查通过。**注意**：这里用的是配置里的 3.0 m/s²，
它来自实车 smoother 的既有值，**不是实车辨识得到的制动能力**；正式提速前仍必须按方案 §10 P5 做辨识。

**未一并改动、需要单独决策的三项：**

1. `min_y_velocity_threshold`：实车 Omni 配置是 `0.5`，仿真 Omni 是 `0.001`。**没有把仿真改成 0.5** ——
   方案 §2.2 明确指出 0.5 会把全向底盘的低速横移反馈抹成零；实车 MINCO 配置已是 0.001。
   建议把实车 Omni 也改到 0.001，而不是把仿真改坏。
2. `rotation_controller.wz_max` / `rotation_test_sender.wz_max` 仍是 `2.0`（仿真自转测试链路）。
   mux 已钳到 1.0，所以最终输出与实车一致；这两个是仿真测试工具，未动。
3. 实车侧自身存在一处**不一致**（本次未改，属于实车配置问题）：
   `src/srm27_nav_protocol/config/srm27_nav_protocol.yaml` 是 `max_vx/vy 2.5`，
   而 `src/srm27_bringup/params/node_params.yaml` 是 `0.5`，后者注释却写着“与前者保持一致”。
   `start_real_nav.sh` 实际加载的是前者（2.5）。需要确认哪个才是实车想要的串口上限。

## 6.4 第三次仿真运行：车完全不动（速度层 MPC 起步自锁，已修复）

修掉权重问题后，轨迹本身完全正常（`minco_trajectory` 最大速度 1.43 m/s、净空 0.587 m、
规划 4.5 ms），但**车一步都不往前走**：BT 报 `Failed to make progress`、反复 recovery，
实测 `cmd_vel_nav` 的 `vx` 在 **±0.01 m/s 之间来回变号**，机器人原地缓慢倒退 0.36 m。

现场抓到的关键数据（`FollowPath/diagnostics`）：

```text
planning_result      = success        minimum_clearance   = 0.587
maximum_speed        = 1.433          projection_progress = 0.0155 s   ← 进度卡在 0
requested_vx         = -0.0133        trajectory_age      = 0.012 s
```

### 根因：速度层 MPC 在“轨迹从静止起步”时必然自锁

离线把「参考 + MPC」整条链按同一配置闭环复现，结论明确：

1. 速度层 MPC 的决策量**就是速度**，参考按当前投影进度采样；MINCO 轨迹从静止起步，
   所以 `u_ref(0) = 0`。
2. 手工重建 QP（与 `MpcSolver` 逐位一致：`U*` 的 `vx[0] = -0.0065` 完全相同）算出三个候选解的
   目标函数值：**`U* = 0.0023`、`U = Uref = 0.0077`、`U = 0 = 6.32`** ——
   也就是说"先不动、把加速后置"**确实是数学上的最优**，求解器没有任何问题。
3. 于是每周期第 0 拍命令 ≈ 0 → 车不动 → 投影进度永远停在 0 → 参考永远是"从静止起步"，
   形成自锁。`projection_progress = 0.0155 s` 就是这个自锁的直接证据。

顺带排除了两个嫌疑：**热启动不是原因**（热启动与冷启动结果逐位相同，只是 0 次 vs 60 次迭代）。

### 修复：命令前瞻（方案 §7.4）

取预测序列里 `t_now + 前瞻` 那一拍作为输出速度，而不是第 0 拍。闭环实测（1.0 s 内）：

| 前瞻 | 1.0 s 位移 | 末速 |
|---:|---:|---:|
| 0（改前） | **-0.002 m** | ~0（自锁） |
| 0.10 s | 0.175 m | 0.217 m/s |
| **0.20 s（新配置）** | **0.325 m** | 0.279 m/s |
| 0.40 s | 0.630 m | 0.868 m/s |

也曾试过"参考前置时间"（把参考按 `progress + lead + i*h` 采样），**单独使用反而更差**
（2.0 s 才走 0.03 m、速度 −0.073 m/s），因此没有保留这个参数，只保留前瞻。

改动：`mpc.command_lookahead: 0.0 → 0.20`（代码默认值与两份 MINCO 配置同步）。
**实车必须按实测的执行延迟重新标定**，0.20 s 只是仿真（无内环延迟）下的标定值。

新增回归测试 `test_mpc_startup.cpp`（3 个用例）：

| 用例 | 断言 |
|---|---|
| `PositiveLookaheadStartsFromRest` | 前瞻 0.20 s 时 1.0 s 内前进 > 0.15 m、末速 > 0.10 m/s |
| `ZeroLookaheadStallsFromRest` | 前瞻 0 时几乎不动（< 0.05 m）—— 把失败模式固化，防止回归 |
| `CommandAtReturnsTheRequestedPredictionStep` | 前瞻取拍正确、两拍之间为线性插值 |

测试总数 **375**，全部通过。

## 6.5 第四次仿真运行：车来回走、不沿红线（验证阈值两侧不一致，已修复）

车能动之后的现象是**来回走、不沿规划红线**。抓到的数据：

| 量 | 值 | 说明 |
|---|---|---|
| `minimum_clearance` | **1.33 m** | 轨迹几何完全正常 |
| `maximum_speed` / `maximum_acceleration` | 1.43 / 1.72 | 都在上限之内 |
| `planning_time_ms` | 4~7 ms | 规划很快 |
| `requested_vx` | **0** | 控制器根本没输出速度 |
| `projection_progress` | **0** | 从未开始跟踪 |
| odom x 轨迹 | 5.55 → 4.10 | 被 recovery 一路往回拖 |

即：**工作线程算出的轨迹很好，却在验证阶段被拒**，控制器永远拿不到轨迹 → 输出零 →
BT 的 `BackUpFreeSpace` 反复把车往回带，看起来就是"来回走"。日志里的失败原因是
`safe prefix is shorter than the MPC window plus stopping time`。

### 根因：工作线程与控制线程用了两套校验配置

`planning_worker.cpp` 自己拼了一份 `TrajectoryValidatorConfig`，只填了半径/裕量/速度/加速度，
**没有**填 `braking_deceleration` 与 `reaction_latency`，于是用了 core 默认值
`a_brake = 0.3 m/s²`。要求的安全前缀变成：

```text
required = reaction_latency + v / a_brake = 0.1 + 1.5 / 0.3 = 5.1 s
```

而任何 2 m 局部视野的轨迹都到不了 5.1 s ⇒ **每条候选轨迹都被拒绝**。

这个坑在 0.5 m/s 档看不出来（要求 1.77 s，能过），**是提速到 1.5 m/s 才暴露的**；
也就是说它一直存在，只是被旧的测试速度掩盖了。这也正好是方案 §6.1 强调"制动能力必须是
实测可保证的值"的原因——它直接决定轨迹能否被放行。

### 修复

| 位置 | 改动 |
|---|---|
| `PlanningRequest` | 新增 `TrajectoryValidatorConfig validator_config`，由控制器填入 |
| `minco_mpc_controller` | 抽出 `makeValidatorConfig()`，控制周期内的复验与后台工作线程**共用同一份**配置（只有速度/加速度上限按当前有效约束覆盖） |
| `planning_worker` | 直接使用请求里的配置，不再自行拼装、不再依赖 core 默认值 |
| 新增参数 `safety.braking_deceleration` | 显式配置有效制动减速度，`0` 表示退化为使用 `limits.max_linear_accel`；**必须按实车制动辨识填写**（方案 §6.1、§10 P5） |

离线复核（同一场景：2 m 直线 + 前方厚墙）：

```text
修正前（a_brake=0.3） 要求前缀 5.1 s  -> 必然失败
修正后（a_brake=3.0） 要求前缀 0.6 s，实际有效前缀 2.59 s -> 通过
墙前绕行净空 0.490 m（要求 0.33+0.05=0.38 m），与 w_obstacle 在 100~1000 之间取值无关
```

因此这次没有调整障碍权重——障碍项本身够强，问题完全在验证阈值上。

## 6.6 第五轮：按现场取证修复恢复逻辑与感知链路

现场报告（20:53:45 `Goal failed`；7 条轨迹校验碰撞告警 → 121 次控制中止 / 115 次清图 /
5 次脱困；脱困成功后 FollowPath 约 1 ms 再次失败）。逐条核对源码后确认并修复：

### 6.6.1 恢复/重试沿用已超时的建轨计时（已修复）

`invalidateTrajectory()` 里对宽限计时的处理是：

```cpp
if (map_version_ == 0) { build_start_stamp_ = -1.0; }
else if (build_start_stamp_ < 0.0) { build_start_stamp_ = _now_stamp; }
```

地图建好之后就**再也不会清零**。于是：一次失败把 `build_start_stamp_` 设为 t0 →
1 s 后抛异常 → BT 脱困 → 同一个目标重新下发 FollowPath（终点未变，被判为"路径刷新"，
连 `invalidateTrajectory` 都不会调用）→ 下一拍 `now - t0` 早已大于 1 s → **立刻再次抛异常**，
BT 陷入 recovery 循环。这正是报告里"约 1 ms 就再次失败"的机制。

修复（两处）：

* `invalidateTrajectory()`：无条件 `build_start_stamp_ = -1.0`（该函数本身代表一次新的建轨尝试）；
* `fail()` 抛异常**之前**清零：本次 FollowPath 就此结束，下一次调用属于新尝试，应重新获得完整宽限。

### 6.6.2 地形感知链路：QoS 不兼容 + 坐标系错误（已修复）

发布端自己的日志给出了铁证：

```text
[WARN] ign_sim_pointcloud_tool: New subscription discovered on topic
       '/red_standard_robot1/velodyne_points', requesting incompatible QoS.
       No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY
```

在线端点也一致：`velodyne_points` 的发布端是 **BEST_EFFORT**（`ign_sim_pointcloud_tool`
用 `rclcpp::SensorDataQoS()`），而 `terrain_analysis` / `terrain_analysis_ext` 用默认的
**RELIABLE** 订阅 → 不兼容、完全收不到点云。

**QoS 修复**：两个地形节点的点云订阅改为 `rclcpp::SensorDataQoS()`。这样既与 BEST_EFFORT
发布端兼容，也与 RELIABLE 发布端兼容（实车 LIO 走 RELIABLE），无需再改发布端。

**坐标系修复**：`terrain_analysis` 的约定是"点云在 odom、用 `lidar_odometry` 的雷达位姿
逐点相减"（`point.x - vehicleX`，输出 `terrain_map` 的 `frame_id = "odom"`）。而仿真的
`velodyne_points` 是**雷达系**，直接喂进去会把雷达系坐标当 odom 用，生成错误地形图。
修复分两步：

1. `sensor_scan_generation` 的输出统一为 **odom 表达**：输入已是 odom（实车的
   `registered_scan`）直接透传；输入是雷达系（仿真的 `velodyne_points`）用**同一采样时刻、
   未经 `suppressJitter` 过滤**的雷达位姿变换到 odom；其它坐标系按采样时刻查 TF，查不到就
   跳过本次发布（不使用单位变换伪装）。
2. `navigation_launch.py` 中两个地形节点的输入从 `velodyne_points` 改为 `sensor_scan`
   （非组合与组合两条路径都改；`sensor_scan_generation` 自己的订阅仍保持 `velodyne_points`）。

顺带说明：`suppressJitter` 只服务于 RViz 显示，不能污染后端的几何——这也是"用未过滤位姿做变换"
的原因。

### 6.6.3 轨迹校验失败改为可定位（已改进）

原先只报一句 `trajectory collides with the raw obstacle set`。现在失败原因会带上
首次违例的时间与位置以及关键数值，例如：

```text
trajectory collides with the raw obstacle set | t=1.2400s/ 2.5900s pos=(1.31, -0.42)
| 最小净空=0.2810m 需要=0.3800m 有效前缀=2.5900s 需要=0.6000s 最大速度=1.4312 最大加速度=1.7204
```

这样复跑时可以直接在 RViz 里对照该点判断是"地图把某处标成了障碍"还是"优化器没绕开"，
不必再靠猜。

### 6.6.4 关于其余现象

* `Goal failed` 的直接原因很可能是"感知链路断了 ⇒ 局部代价地图没有障碍 ⇒ MINCO 在空白图上
  规划 ⇒ 候选轨迹被独立验证判定会撞上原始障碍"。6.6.2 修好后应显著减少；**但必须复跑确认**，
  本次没有运行仿真，不能声称已解决。
* QP 超预算（最高 10.43 ms）属性能门槛告警，解仍被采用（§6.2 已说明），不是导航失败的首要证据。
* RViz 丢帧与扫描里程计间隔告警同样不是首要证据。

## 6.7 第六轮：按现场取证修四类问题

现场报告（controller_server 日志 `controller_server_19301_1791381093930.log`）：
40 次"净空 0.3737~0.3798 m 对 required 0.38 m"的碰撞拒绝、9 次
`no_path: local path is empty after clipping`、运行中 mux 为 `v_max=0.5 / wz_max=2.0`
（源码期望 1.5 / 1.0）、QP 最高 15.6 ms 与 22 次 50 Hz 控制超时。

### 6.7.1 净空只差几毫米就被判碰撞（已修复）

优化器对障碍是**软约束**，最优点会停在安全距离附近；而验证器按
`robot_radius + clearance_margin = 0.38 m` **硬判**。两者取同一个值时，最优解经常落在阈值
下方几毫米（实测 0.3737~0.3798），于是轨迹被整条丢弃。这与 §6.2 的软约束问题是同一类。

修复：`MincoOptimizerConfig::optimization_clearance_margin`（默认 0.05 m），优化目标变为
`robot_radius + clearance_margin + optimization_clearance_margin = 0.43 m`，
比硬阈值高出一截，留出错动余量。配置项 `minco.optimization_clearance_margin`。

**没有放宽硬阈值** —— 安全门槛保持 0.38 m 不变，改的是优化目标。

### 6.7.2 终点附近 `no_path`（已修复）

`extractLocalPath()` 从最近路径点的**下一个**点开始取局部路径；当最近点已经是路径终点时，
一个点都取不到，局部路径退化成"只有当前位置"→ 判空 → 终点附近永远生不出减速/停止轨迹。

修复：局部路径为空时兜底补上**路径终点**（条件：终点在有效地图内、且与当前位置有实际距离），
让终点附近仍有一条"当前位置 → 终点"的短路径可用。
另外，路径为空时分两种状态返回：落在 `terminal_reached_radius`（默认 0.20 m）内返回
`already_at_goal`，否则才返回 `no_path`。控制器对 `already_at_goal` 的处理是**受控停车**：
不算控制失败、不计入建轨宽限、不抛异常，交给 `StoppedGoalChecker` 判定成功——
否则"到点"会被自己的超时逻辑判成控制器异常，把成功变成失败。

### 6.7.3 "改了 v_max=1.5 却仍被限到 0.5"（已加防呆，但归因更正）

现场归因是"底盘 launch 没传 `params_file`"。**实测否定了这个归因**：

```text
srm_cmd_mux 已启动: ... 平移上限 1.500 m/s (v_max 1.500), 自转上限 1.000 rad/s
```

* `srm_chassis_control.launch.py` 的 `params_file` 默认值就指向包内配置；
* `RewrittenYaml(root_key=namespace)` 的输出经核对为 `v_max: 1.5 / wz_max: 1.0`；
* 单独启动底盘链路实测，节点读到的就是 1.5 / 1.0。

而现场观察到的 `0.5 / 2.0` **正好等于 `cmd_mux_node.cpp` 里的代码默认值**，
说明那次运行加载到的配置不是本工作区的这份 —— 同机存在第二份工作区
（`~/srm_nav_27test`，其同名包配置就是 0.5 / 2.0）。若 `AMENT_PREFIX_PATH` 里它排在前面，
**除 MINCO 包以外的所有同名包（含底盘限速、Nav2 参数、行为树）都会从那边加载**，
形成"一半新一半旧"的混合环境。

处理：

1. `script/start_sim_nav.sh` 增加**工作区一致性检查**：启动前逐个核对
   `srm27_nav_bringup` / `srm27_chassis_control` / `srm27_minco_controller` 的解析路径，
   不在本工作区下就直接报错退出，并提示"当前环境还 source 了别的工作区"。
2. `nav_srm_simulation_launch.py` 显式传递底盘控制的 `params_file`（绝对路径），
   让参数来源在启动文件里可见，不依赖默认值。

### 6.7.4 QP 超时与控制循环超时（已改善）

两个来源：

1. `TrajectoryValidator::checkCollision()` 内部为了确定采样步长又做了一次
   `computeExtrema()`（含多项式求根）。改为直接使用**配置的速度上限**——上限必不小于实际
   速度，步长只会更保守，却省掉一次求根。
2. `refreshMap()` 在地图版本变化时把 `last_full_revalidation_stamp_` 置 -1，
   等于让 0.2 s 的复验节流**完全失效**：滚动局部地图 + 地形层的版本号每周期都在变，
   于是每个控制周期都跑一次全轨迹复验。改为只置一个 `map_changed_since_validation_` 标记，
   复验按 `max(0.2 s, 1/replan_frequency)` 节流执行。

**注意**：这是"安全与耗时的折中"——已在执行的轨迹每 0.2 s（1.5 m/s 下约 0.3 m 行程）在最新
地图上复验一次；新轨迹仍由工作线程在最新地图上验证后才提交。

### 6.7.5 新增诊断：画出"规划器实际吃进去的局部路径"

现象"车似乎只朝一个方向走、不沿红线"需要先分清是**规划器拿到的路径就不对**还是**轨迹对但跟踪不上**。
新增话题 `FollowPath/planning_input_path`（odom 系）：把 `extractLocalPath()` 的输出原样发布出来。
在 RViz 里同时显示三条曲线即可分辨：

| 观察 | 结论 |
|---|---|
| 全局红线（map）与 `planning_input_path`（odom）**形状不一致** | 局部路径抓取/坐标系问题（`extractLocalPath` 取到了路径的另一支） |
| 两者一致，但 MINCO 曲线偏离 | 优化器问题（参考吸引缺失、权重失衡） |
| 三条都一致，但车不沿曲线走 | 跟踪/执行问题（限速不一致、延迟、lookahead 标定） |

关于"只朝一个方向"还需要排除一个**设计后果**：阶段一 `yaw_policy.mode = xy_only`，
MPC **不控制航向**，机器人是**平移（含横移）沿路径走、机头不转**；而且仿真的
`srm_cmd_mux` 只采用 `rotation_velocity.angular.z`，会把导航命令的 `angular.z` 丢掉。
所以即使把 yaw 模式改成 `follow_tangent`，仿真里机头也不会转 —— 那属于方案 §8.1 的
`NAV_SE2`（P4）改造范围，本次未做。

## 6.8 第七轮：按坡道取证修复感知几何链路（只改几何，不动安全阈值）

现场取证与修复方案见 `log/diagnostics/penultimate_goal_20261008/`（`report.md`、`repair_plan.md`）。
方案要求按"修坐标变换 → 同坡道验证 → 再处理规划边界"推进；本批只做**修**这一步，
**没有**调整 0.38 m 碰撞阈值、MPC 权重、`optimization_clearance_margin`、摩擦或质量。

### 6.8.1 真值里程计只发 yaw，丢掉高度与 roll/pitch（已修复）

| 位置 | 修复前 | 修复后 |
|---|---|---|
| `SrmVelocitySystem.cc`（真值里程计姿态） | `Quaterniond(0, 0, chassisPose.Yaw())` | `chassisPose.Rot()`（完整姿态） |
| 同文件 twist | 只发 `angular.z` | `angular.x/y/z` 全发 |
| `simulation_ground_truth_odometry.py` | 相对位姿只算 x/y/yaw，`translation.z` 恒为 0 | `odom -> base_link` 为完整三维相对位姿（位置含 z、姿态含 roll/pitch） |

为什么必须改：`terrain_analysis` / `terrain_analysis_ext` 的 `odometryHandler`
（`terrainAnalysis.cpp:105-117`）**本来就是按三维雷达位姿写的**（`getRPY` 出 roll/pitch/yaw，
并读 `position.z`）；`sensor_scan_generation` 也用 `lidar_odometry` 的完整变换把点云变到 odom。
真值链路只喂二维位姿，等于让地形算法在**被压平的位姿**上工作。

补抓数据里的直接证据（复算脚本 `log/diagnostics/penultimate_goal_20261008/pose_chain_impact.py`，
输出 `pose_chain_impact.txt`，数据是同目录 `replay_samples.jsonl` 的 1019 个对齐样本）：

* 底盘 IMU（`chassis_imu` 挂在 `base_link` 上、安装位姿为单位）与真值里程计 `gt` 的 **yaw 最大差
  0.158°**——两者同系，说明坐标系约定没问题；
* 但同一时刻 `gt` 的姿态是**纯 yaw**（`q = [0, 0, 0.01133, 0.99994]`），与 IMU 姿态的最大夹角
  **10.007°**（roll 4.24°、pitch −9.07° 被丢掉）；
* 适配器输出 `odom.z` 恒为 `0.000`，而真值高度已到 0.36 m（相对出生点 0.16 m）。

odom 系定义（写下来避免再次漂移）：

* 原点 = 出生**位置**（含 z），姿态只取**出生航向** → odom 始终重力对齐、z 轴竖直。
* 高度基准 = 出生点地面高度，**不再每条消息把 z 归零**。
* `odom -> base_link` 的 TF 与 `odometry` 消息里的位姿是**同一个完整三维位姿**；只有 twist 保持
  平面（Nav2 的速度接口是平面的，高度方向与 roll/pitch 角速度留在 Gazebo 真值话题里）。

### 6.8.2 `_compose()` 把第二个变换的四元数分量当成了平移（已修复）

修复前：

```python
sx, sy, sz, sw = second.rotation.x, ...   # 取的是四元数分量
tx = 2.0 * (qy * sz - qz * sy)            # 却当成平移代入叉乘
```

即"用 first 旋转 second 的平移"这一步的**输入取错了字段**。方案里的最小复现（first 只 yaw 90°、
second 只有平移 `(0.15, -0.15, 0.22)` 且旋转为单位四元数）：正确 `(0.15, 0.15, 0.22)`，
缺陷实现返回 `(0.15, -0.15, 0.22)`，相差 0.30 m。

用真实外参（含 -4° 安装俯仰、-90° 安装偏航）离线复算旧实现与被测实现的雷达位置差：

| 底盘姿态 | 旧实现误差 |
|---|---|
| 平地、yaw = 0 | 0.000 m（所以长期没被发现） |
| yaw = 90° | 0.349 m |
| 坡面姿态（roll 4.36°、pitch −9.01°） | 0.161 m |
| roll 4.36°、pitch −9.01°、yaw 90° | 0.216 m |
| 一般姿态 | 最大 0.525 m |

对**点云**的影响远大于雷达自身那 0.16~0.17 m：两条链路之间是一个刚体变换，姿态差 10° 意味着
点云被整体转错了 10°，距离越远的点偏得越多。用补抓里姿态最陡的样本（roll 4.238°、pitch
−9.067°，实测雷达位置偏差 0.1715 m）复算，球面上最坏方向的点位移：

| 点到雷达的距离 | 旧链路点位误差 |
|---|---|
| 1 m | 0.35 m |
| 3 m | 0.70 m |
| 5 m | 1.04 m |

这足以把坡面点云放到错误的格子里（`terrain_analysis` 的 `intensity` 是"相对车体平面的高度"，
点云本身被转错就等于障碍/可通行判断跟着错）。

修复方式：把纯几何抽成 `scripts/srm_nav_pose_math.py`（**只依赖标准库**），`_compose()` 改为调用
`compose_pose()`——平移只允许用 second 的平移参与 first 的旋转，旋转用四元数乘法，两者是互相
独立的变量。`rotate_vector()` 走四元数三明治乘积而**不是**手写展开式：手写展开式正是这次出错
的地方，现在只有一个实现需要验证。

### 6.8.3 二维/三维边界核对（方案第 4 项，结论：无需改动）

| 检查项 | 结论 |
|---|---|
| 规划状态是否显式提取 x/y/yaw | 是。`StateAdapter::stateAt()` 只取 `position.x/y` 与 `tf2::getYaw()`（`state_adapter.cpp:207-210`） |
| 速度是否"先三维车体系、再取平面分量" | 是**单次**旋转：真值 twist 在车体系，`bodyToOdomVelocity(yaw, v)` 转到 odom，不存在二次旋转。已知近似：只按 yaw 旋转、忽略车体俯仰，误差量级 `1-cos(pitch)`（坡面 9° 约 1.3%）；本批不改，避免动已标定的速度链路 |
| 输出命令是否仍在真实车体系 | 是。`odomToBodyVelocity(yaw_at_actuation, v)` 一次旋转，`frame_id = base_link`（`minco_mpc_controller.cpp:1150-1200`） |
| 规划系转换是否引入额外旋转 | `planning_frame: odom` 与里程计同系，`stateAt()` 不走 TF 分支，速度也不做二次旋转 |

### 6.8.4 回归测试与验收条件

新增 2 个测试套件（用普通 Python 入口而不是 pytest：本机 pytest11 插件与
`launch_testing`/较新 `pluggy` 不兼容，与 `srm27_robot_description/test/test_model_builder.py`
的做法一致）：

| 测试 | 用例 | 覆盖 |
|---|---|---|
| `test/test_pose_math.py` | 8 | 单位变换、非零外参、90° yaw（含 0.30 m 复现）、±10° pitch、组合 rpy、非零初始 yaw、静态点世界坐标回环、500 组随机位姿对**独立旋转矩阵**参照 |
| `test/test_ground_truth_odometry_pose.py` | 7 | 用**真实节点**+合成 Gazebo 真值：TF 与 `odometry` 位姿一致且带 roll/pitch/z、`lidar_odometry` == TF 位姿 ∘ 几何 YAML 外参、外参随姿态旋转、静态点回环、平地行为与修复前一致（z=0、纯 yaw）、抖动保持整体位姿 |

参照实现刻意用**旋转矩阵**（不经过四元数）：否则"把四元数分量当平移"这类缺陷会在被测与参照里
同时出现而测不出来。

```bash
source install/setup.bash
colcon test --packages-select srm27_nav_bringup \
  --ctest-args -R "pose_math|ground_truth_odometry_pose"
# 期望：2/2 passed；每个套件逐条打印 PASS
```

验收条件：

* 车体转向或上下坡时，雷达外参平移跟着旋转；TF 与 `lidar_odometry` 给出同一个雷达位姿。
* 静态场景的点云世界坐标在上/下坡过程中保持一致（测试用固定点回环表达）。
* 平地行为与修复前完全一致（z = 0、姿态只剩 yaw），不影响已标定的二维导航。
* 碰撞阈值 0.38 m、MPC 权重、`optimization_clearance_margin` 全部未改。

`colcon test --packages-select srm27_nav_bringup` 会同时跑该包的 lint 用例，其中两个失败
**与本次改动无关且在本批之前就存在**：`copyright`（`launch/real_mapping_launch.py`、
`launch/real_robot_state_publisher_launch.py` 缺版权声明）；`pep257`（全仓库中文 docstring 的
D400/D415，`model_builder.py`、`navigation_launch.py` 等既有文件同样报）。新增文件的 docstring
风格与包内既有脚本一致，未新增规则类型，也未改 lint 配置。

### 6.8.5 本批未做

1. **同坡道对照重跑**（由用户在仿真里执行）：固定地图/起终点/参数，对齐 `cmd_vel_sim`、真值位移、
   底盘 IMU、`terrain_map`、`local_costmap`、原始局部路径、MINCO 曲线与拒绝坐标，判断"坡面误标"
   与"MINCO 切近真实障碍"哪一个是主因。
2. **终点短轨迹**（方案第三批）：末端无碰撞且满足停车条件时，把末端静止状态延拓覆盖剩余 MPC
   窗口；**不**降低全局 `required_prefix_duration`，也**不**因 `terminal` 标志跳过制动/净空检查。
3. 不换 MPC 模型、不调大加速度、不动轮地摩擦系数、不加电机扭矩模型。

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
4. **仿真/实车验证**：Gazebo 仿真已跑过多轮（§6.1–§6.8 都是现场运行后的修复），但 P2 的验收场景
   （空场前进/横移/斜移/停止/抢占）仍未逐项跑完；**实车一次都没跑过**。
   MINCO YAML 里的加速度、制动减速度、延迟都是占位/仿真标定值（§6.8.1 的 `command_lookahead`
   明确按仿真标定），上实车前必须按方案 §10 的 P5 重新辨识。
5. **TF 无超时查询的日志噪声**：`StateAdapter` 在 `planning_frame != odom` 且变换不可用时会
   调用 tf2 的 `lookupTransform`（超时为 0，不会阻塞），tf2 自身会打印一条 ERROR 级
   “需要专用线程”提示。正常配置下 `planning_frame == odom`，不会走到这条路径；
   若后续确需跨系查询，应当在控制器侧自行管理 TF 线程模型再处理这条日志。
