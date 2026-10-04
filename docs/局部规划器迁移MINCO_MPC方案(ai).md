# 局部规划器迁移方案：中科大 RoboWalker MINCO + MPC → 本项目

> [!NOTE]
> **文中提到的 `fake_vel_transform` / `base_link_fake` / `gimbal_yaw_fake` 属于旧步兵链路**，
> SRM 仿真已删除该链路（导航与代价地图统一使用真实随底盘自转的 `base_link`）。
> 涉及"虚拟不转底盘"的前提在 SRM 仿真下不再成立，迁移设计时需要重新评估；
> 现状见 [`SRM仿真与自转控制实现(ai).md`](./SRM仿真与自转控制实现%28ai%29.md)。

> 输入材料：
> - 报告：`/home/srm/下载/UTF-8__中科大哨兵2025技术报告.pdf`（28 页，重点 §5.5）
> - 本项目：`局部规划器相关(ai).md`（现控制器）
>
> 本文只回答一件事：**把报告那套「JPS + MINCO + ESDF 局部规划 + 稠密 QP MPC」搬到当前工作区，怎么落地、分几步、每步验收什么、哪里会翻车。**

---

## 0. 结论先行

1. **报告里其实有两级"局部规划"，优先级完全不同，不要一起搬。**
   - **A. 轨迹重规划器（MINCO+L-BFGS，10ms 级，1~10Hz）**：解决"全局路径是折线、且不避动态障碍"。这是报告 §5.5.2~5.5.4 的主体。
   - **B. 轨迹跟踪控制器（MPC→稠密 QP，1ms 级，100Hz+）**：解决"上位机延迟、全向底盘约束、窄缝横向精度"。这是 §5.5.5。
   - **B 的性价比远高于 A**，而且 B 可以**独立于 A 先上**：MPC 只要求输入一条"带时间戳的轨迹"，不需要这条轨迹是 MINCO 生成的。本项目现在有全局路径和 20Hz 控制循环 —— **Phase A 可先验证 MPC 跟踪收益**。
2. **本项目已经具备报告方案的两个关键前提，比报告当时的条件还好**：控制器输入频率和报告完全一致（LIO 100Hz / IMU 200Hz / 轮速 1000Hz，见 §5.5.5.4.1），且已有 `fake_vel_transform` 这层"虚拟不转底盘"抽象（报告是靠控制器自己算底盘 yaw，没有这层）。
3. **一个明确的短板**：本项目底盘串口只吃 `cmd_vel_chassis`（Twist，速度级，`standard_robot_pp_ros2.cpp:73`），**没有力矩/力前馈接口**。报告 §5.5.5.6 亲口说"没用规划的推力做前馈"导致隧洞入口起步没劲、并怀疑这是场上卡在洞口的直接原因 —— 我们**没有能力复现他们的补救（速度下限），也没能力实现他们认为正确的做法（力前馈）**，除非先改串口协议。这条要么补协议，要么接受低速精度损失。
4. **推荐路线**：先把 **Nav2 controller 插件换成 MPC（Phase A/B）**，不动 Nav2 全局规划/行为树/仿真链路；MINCO 重规划器作为**独立节点**并行开发（Phase C~F），最后再决定是否整体替换 Nav2 planner。理由见 §4 决策点 D1。

---

## 1. 报告方案拆解（只保留可迁移的部分）

### 1.1 规划器侧（§5.5.2 ~ §5.5.4）

| 模块 | 报告做法 | 可迁移性 |
|---|---|---|
| 轨迹表示 | MINCO（GCOPTER 简化版），轨迹**必然经过路标点** | 强，直接 port |
| 优化器 | L-BFGS **无约束**优化，梯度汇总到控制点 | 强 |
| 障碍梯度 | 静态场地图 ESDF（5cm 分辨率）；实测 0.1m 分辨率下**全量重建 2D-ESDF 仅 2ms**，因此放弃增量、每帧重建 | 强，但分辨率要按我们场地图重估（§3.2） |
| ESDF 插值 | **二次函数插值**（2×3 邻域 3 点拟合）替代双线性，解决"梯度无效化"（峡谷中双线性梯度为 0） | **强，这是他们优化失败的头号原因，必须抄** |
| 动态障碍 | 每障碍物独立 ESDF，与场地 ESDF 逐点取 min | 中，需要障碍物聚类+速度估计（我们目前没有） |
| 前端搜索 | JPS（无权重格子上最快） | 强 |
| 前端加运动学 | 思路一：`s = k1·Σ直线长 + k2·Σ转角` → 梯形加减速算总时长 T → 按期望 Δt 重采样 | 强 |
| 前端精修 | 思路二：再跑**一次放松碰撞/时间约束的 MINCO 优化**，否则折角处时间分配不合理→后端扭成麻花 | 中（多一层，但报告说"基本解决"稳定性问题） |
| 两步优化 | PRE（梯度不看方向，只求形状）→ FINELY（切向梯度剔除 + `gradPos` 归一化 + 梯度阈值 0.5 触发 `violaPos`） | 强，报告认为这是质量提升最大的一步 |
| PRE 时间约束 | 段时长约束在平均段时长的 **[0.9, 1.1]**，防止控制点被推出狭窄区 | 强，小改动大收益 |
| L-BFGS 收敛 | 抄 DDR-OPT 的 `param.past > 0 && |f_init-f|/(|f_init|+1) < delta/past` 提前退出；否则 line search 炸 | 强 |
| 性能预算 | 30m 轨迹 ≈ 80ms（单线程，放宽收敛条件后）；加两步优化后**大幅加快**且控制点数减少 | 需要我们在 NUC 上重测 |
| 重规划 | 三策略：全局重规划 / 仅优化 / 部分重规划；部分重规划 = 起点取当前位置在轨迹上的**最近投影点，再往回想一段时间**保留一小段旧轨迹拼到 JPS 结果前面 | **强，这是轨迹不跳变的关键** |
| 起终点 | 起点/目标点落在障碍里时的"排出障碍"策略 | 强，哨兵目标点常被占 |

### 1.2 控制器侧（§5.5.5）

- **模型**：状态 `[x, y, θ, vx, vy, ω]`，控制 `[Fx, Fy, Mz]`，线性定常离散；`A = [[I, dt·I],[0, I]]`，`B = [[dt²/(2m)…],[dt/m…]]`，转动部分用 `Izz`。
- **QP**：`min ½UᵀHU + fᵀU`，`H = BᵀQ̄B + R`、`f = BᵀQ̄(Ax₀ - Xref)`，**H、约束矩阵时不变 → 开机预构造**，每周期只重算 `f`。
- **约束**：速度上下限、控制量上下限、**全向舵轮摩擦力锥（转成线性不等式）**。
- **求解器**：qpOASES，40 步预测，**平均 < 1ms**。
- **双次期望轨迹**（这条是窄缝通过率的来源）：
  1. 一次：按时间步 Δt 在 MINCO 上取样，`x,y,v` 取自轨迹，`yaw` 由速度切向给出。
  2. 二次：发现"起点状态与机器人实际状态差异大时，纵向误差会淹没横向误差"。不引入 Frenet（大曲率线性化误差 + 奇异点），而是**按一次轨迹与期望轨迹对应时间点速度法向夹角余弦缩小时间步长重采样** —— 主动牺牲纵向速度换横向精度。
- **输出**：取 `t_start + 10ms` 处的规划速度作为 `cmd_vel`（补偿求解+通信+下位机 PID 延迟）。
- **底盘朝向策略**：两种状态（小陀螺前进 / 底盘跟随轨迹切向），按环境狭窄程度自动切换；**狭窄程度由 MINCO 控制点到 ESDF 的最小距离判断**；进洞前把雷达方向对齐切向正向，进洞后就近取切向正/反向。

---

## 2. 与本项目的差距映射

| 报告模块 | 本项目现状 | 差距 | 结论 |
|---|---|---|---|
| 全局规划 | Nav2 `nav2_theta_star_planner`（`nav2_params_srm.yaml:751`） | 报告不用全局规划器，每次都由 JPS 在局部窗口重搜 | 保留 Theta* 作为**参考路径/兜底**，不急着换 |
| 局部轨迹规划 | Nav2 Controller 跟踪 Theta* 全局路径（20Hz） | 无独立局部轨迹重规划、无显式动力学约束 | 被 Phase D/E 的 MINCO 重规划器替代 |
| 控制 | `OmniPidPursuitController`（纯跟踪+PID） | 无延迟补偿、无显式动力学约束 | **Phase A 先换成 MPC** |
| 障碍梯度 | 2D costmap（`InflationLayer` 只有几何膨胀，**没有距离梯度**）+ `srm27_nav_costmap::IntensityVoxelLayer` | 无 ESDF、无连续梯度 | 需要新建 ESDF（§3.2） |
| 动态障碍 | `IntensityVoxelLayer` 在局部代价地图中标记点云障碍 | 有障碍占用信息，缺少聚类速度估计 | 后续增加聚类和速度估计 |
| 前端搜索 | Nav2 Theta*（全局） | 无局部窗口 JPS | Phase E |
| 里程计 | Point-LIO（`camera_init`→`body`）+ `loam_interface` 转 `odom` 系 + small_gicp 先验 PCD 重定位 | **频率完全对齐报告（100/200/1000Hz）** | 直接用，无需改 |
| TF / 自旋 | `fake_vel_transform`：`base_link_fake` 虚拟不转底盘，把 `spin_speed_` 加到输出 yaw（`fake_vel_transform.cpp:153`），输入 `cmd_vel_nav2_result`，输出 `cmd_vel_chassis` | 本车 `init_spin_speed: 0.0`（注释说明本车没有 `cmd_spin` 发布者） | **这层是我们的额外优势，见 §3.4** |
| 底盘执行 | 串口 `cmd_vel_chassis`（Twist 速度级） | **无加速度/力前馈通道** | 见 §0.3，需要决策 |
| 感知 | MID360 点云 + `terrain_analysis`（局部可通行性）+ `pointcloud_to_laserscan` | 无 3D Occupancy Grid 结构化（报告 §5.3 那套） | ESDF 初期用 costmap，后续用 terrain_analysis+点云 |
| 决策 | `srm27_behavior`（BehaviorTree.CPP）+ Nav2 BT navigator + `BackUpFreeSpace` 等恢复行为 | 报告是自己一套重规划策略 | **Phase A~E 全部保留 BT**，别动 |

---

## 3. 关键技术决策

### 3.1 D1：插件化替换 vs 独立规划栈（最重要的决策）

| 方案 | 做法 | 优点 | 缺点 |
|---|---|---|---|
| **① MPC 作为 Nav2 Controller 插件** | 新建 `srm27_mpc_controller`，实现 `nav2_core::Controller`，替换 `FollowPath`。输入=`nav_msgs/Path`（全局路径），内部化为时间参数化轨迹 | 改动最小，可与现有 Omni PID 对比；BT/进度检查/恢复/仿真全部不动，**可随时回退** | 拿不到连续重规划能力（Nav2 给的 Path 是无时间的），MINCO 只能作为插件内部实现 |
| ② 完全替换 Nav2 planner+controller | 自建 `sentry_planner` 节点，直接发 `cmd_vel_nav2_result` | 完全对齐报告架构、能力上限最高 | 要自己实现目标管理/进度监控/失败恢复/仿真链路，工作量翻倍，回退困难 |

**建议：先 ① 走完 Phase A/B/D/E，把 MINCO 做成一个**库**（不绑定节点），到 Phase F 再把库挂到同一个插件里（插件内部自持局部 JPS+MINCO+MPC），只在**确认收益足够**时才考虑 ②。** 报告的 1ms MPC + 10ms 优化本来就是单节点内的实现，插件化完全可以承载。

### 3.2 D2：ESDF 怎么建

- 报告：静态场地图预构造 + 每帧全量重建动态层，合计 ~2ms（0.1m 分辨率，RMUC 尺度）。
- 本项目：
  - 静态层：从 `maps/srm_site_01.pgm`（或先验 PCD）离线构建一次，落盘成 `.esdf` 缓存（含距离 + 梯度），启动时 mmap 加载。**分辨率建议 0.05~0.10m**，赛场 ~30m×15m 在 0.1m 下是 450×225=10 万格，float 距离+2×float 梯度 ≈ 5MB，完全可接受。
  - 动态层：**不要**直接拿 costmap 当输入，而是订阅 `terrain_analysis` 的地面点云 + `IntensityVoxelLayer` 的点云，做 2D 投影 + 形态学闭运算，再跑 **Felzenszwalb 精确 EDT（O(n)，单线程 2ms 量级）**。第一版全量重建，够用；增量（FIESTA）留到以后。
  - 梯度：**必须用报告的二次函数插值**，不要双线性。双线性在"两格距离相同"的峡谷中梯度恒为 0，这正是报告优化失败的主因。
- 依赖：自己写 ~300 行 EDT 比引 `grid_map` + `grid_map_esdf` 更省事，也避免拉进整条 grid_map 依赖链。

### 3.3 D3：QP 求解器选型

- 现状：**系统里没有 qpOASES，也没有 OSQP**（已确认 `/usr/include`、`/opt/ros/humble/include` 均为空）。
- 选项：
  1. **vendored qpOASES**（推荐）：报告的选择，30 变量级稠密 QP 亚毫秒级；源码 3000 行、CMake 直接编，无 ROS 依赖。加进 `dependencies.repos` 或 `src/dependencies/`。
  2. OSQP（稀疏 ADMM）：更适合大而稀疏的问题，我们 40 步 × 3 控制的规模用不上，还多一个 `osqp_vendor` 依赖。
  3. Eigen + 自己写 active-set：不划算。
- 结论：**vendor qpOASES**，并写一个 `MpcQpTest` 节点，把 H/约束预构造后的单次求解时间打点出来（目标 TP99 < 2ms）。

### 3.4 D4：底盘 yaw 策略与 `fake_vel_transform` 的关系

这是本项目**独有的、报告没有的一层**，必须想清楚：

- `fake_vel_transform` 把 `cmd_vel_nav2_result` 的 `angular.z` **加上** `spin_speed_` 后发 `cmd_vel_chassis`，同时维护 `base_link_fake`（导航用的虚拟不转底盘）与 `base_link`（真实底盘）之间的 TF。
- 报告的控制器是**直接控制真实底盘 yaw**（狭窄区就近对齐切向），因为它没有"导航系可以自旋解耦"这一层。
- 我们有两个选择：
  - **(a) 保持分工**：MPC 在虚拟系里输出纯平移速度（`ω≈0`），底盘 yaw 由**独立的策略层**决定并发布 `cmd_spin` 给 `fake_vel_transform`。好处：导航位姿永不旋转，全局路径/代价地图/BT 全部简单，与现状（`init_spin_speed: 0.0`）一致。
  - **(b) 复刻报告**：MPC 直接输出真实底盘 `ω`，绕过或旁路 `fake_vel_transform`。好处：能实现"进洞前把雷达对准切向"，因为近距离避障与底盘实际朝向强相关。
- **建议 (a) 为主 + (b) 的狭窄策略**：MPC 决策用**真实底盘 yaw**（从 `odometry` + tf 取，用于窄缝判断），输出仍走虚拟系平移；新增 `ChassisYawPolicy` 小节点，按 ESDF 狭窄度在 `spin / align_tangent / nearest_tangent` 之间切换，发 `cmd_spin`。
- ⚠️ 当前 `init_spin_speed: 0.0`，整车自旋链路尚未打通。迁移前应先确认比赛时哨兵是否需要自旋；如果需要，应重新打通这条链路并明确 MPC 输出角速度的所有权。

### 3.5 D5：底盘有没有力前馈通道

- 报告认为正确的做法是 MPC 的 `[Fx, Fy, Mz]` 直接做力控前馈，他们没做，代价是低速起步没劲（§5.5.5.6）。
- 本项目串口协议目前只有 `cmd_vel_chassis`（Twist）。要么：
  1. **改协议加一路加速度/力前馈**（`std_robot_pp` 自定义包，工作量在嵌入式侧，收益是低速+窄缝起步性能）；
  2. 保持速度级，接受低速精度损失，用报告的速度下限 trick 兜底。
- **建议先按 (2) 落地，把 (1) 作为独立任务并行推**，因为 (1) 依赖下位机配合，不能卡住软件主线。

---

## 4. 接口定义（迁移的"合同"）

内部通信**必须带时间**，否则 MPC 的延迟补偿无从谈起。建议新增 `sentry_planning_msgs`：

```text
# TrajectoryPoint.msg
float64 t          # 相对轨迹起点的秒数（或绝对 ros 时间，二选一，全栈统一）
float64 x, y, yaw
float64 vx, vy, omega
float64 ax, ay        # 可选：供力前馈用
```

```text
# Trajectory.msg
std_msgs/Header header
TrajectoryPoint[] points     # 严格等时间间隔或单调递增
float64 start_time
geometry_msgs/PoseStamped start_pose
uint8 mode                   # 0=正常 1=仅优化 2=部分重规划 3=纯跟踪(无规划)
bool in_narrow_passage
```

- **给 MPC 的**：`Trajectory`（一次即可，MPC 内部做二次重采样）
- **给下位机的**：仍然只有 `cmd_vel_chassis`（受 §3.5 限制）
- **给调试的**：`/planner/esdf_slice`（OccupancyGrid 或 PointCloud2 带 intensity=距离）、`/planner/control_points`（MarkerArray）、`/planner/replan_level`

x 与 y 关系：一次轨迹取 `yaw = atan2(vy, vx)`（报告 §5.5.5.4.2）。**注意**：如果采用 §3.4(a)，这个 yaw 属于"切向参考"，不是给底盘的指令。

---

## 5. 分期迁移路线

> 每期都是"可独立验收、可独立回退"的。建议**严格按 A→B→C→D→E→F**，不要跳。

### Phase A：MPC 控制器（1~2 周）—— 收益最大、风险最低

**目标**：用 MPC 跟踪 Nav2 现有的全局路径，替换 Omni PID 控制器，其余全部不动。

- 工作项：
  1. vendor qpOASES 进 `src/dependencies/`（或 `dependencies.repos`）。
  2. 建 `srm27_mpc_controller` 包：`qp_solver` + `mpc_core`（H/约束预构造、`f` 每周期重算）+ `nav2_core::Controller` 插件壳。
  3. 一次性轨迹化：把 `nav_msgs/Path` 转成等时间间隔 `Trajectory`（速度用后续 `velocity_smoother` 的限幅值或分段常速假设）；`yaw` 取路径切向。
  4. 状态同步：控制器内用 IMU/轮速做局部积分，把里程计位姿外推到"当前"。**注意本项目 LIO 是 `camera_init→body`，经 `loam_interface` 转 `odom`；务必确认用的是哪条 odom 以及时间戳基准。**
  5. 输出取 `t_start + 10ms` 的前瞻速度，发 `cmd_vel_controller`（Nav2 链路）→ 由现有 `velocity_smoother` / `fake_vel_transform` 接走。
- **验收**：
  - 仿真 + 实车同一批路点，横向跟踪 RMSE 优于 Omni PID 基线 ≥ 20%；
  - TP99 求解时间 < 2ms，控制频率保持 20Hz；
  - 现有哨兵行为树用例全部通过（点位、巡逻、回防）；
  - 回退开关：`FollowPath.plugin` 一键切回 `OmniPidPursuitController`。

### Phase B：底盘 yaw 策略 + 窄缝通过（1 周）

- 新增 `ChassisYawPolicy`（spin / align_tangent / nearest_tangent），狭窄度先用 costmap 距离场近似，Phase C 换成 ESDF。
- 与 `fake_vel_transform` 的 `cmd_spin` 打通（§3.4）。
- **验收**：按真实隧道尺寸（我们比报告的通道更极限：车径 0.66m/洞宽 ~0.8m，单边余量 ~7cm）搭仿真窄缝，连续 20 次通过 0 卡死；入口不减速到停车。

### Phase C：ESDF 服务（1~2 周）

- `esdf_server` 节点：静态层离线构建 + 启动加载；动态层从 `terrain_analysis`/点云 2D 投影 + Felzenszwalb EDT 全量重建。
- **二次插值梯度**（§3.2）+ 可视化话题 + `查询延迟/构建耗时` 统计。
- **验收**：0.1m 分辨率下单帧构建 < 3ms（目标对齐报告的 2ms）；梯度在峡谷地形中不再整片为 0（写单测，构造对称障碍场景断言梯度非零）。

### Phase D：MINCO 局部重规划器（2~3 周，最重）

- 从 GCOPTER 移植 `minco` + `lbfgs`（**不要**整包引入），改 2D 全向微分平坦模型，接 ESDF 梯度。
- 实现 PRE/FINELY 两步优化、时间约束 `[0.9, 1.1]`、切向梯度剔除、`gradPos` 归一化、阈值 0.5 的 `violaPos`、以及 DDR-OPT 的 L-BFGS 提前退出条件。
- 起点：先用**全局路径重采样**当初值（还没 JPS）；重规划策略先只做"全局重规划"。
- **验收**：30m 轨迹单次优化 TP99 < 15ms（NUC Ultra7 上量）；不出现"扭麻花"；优化失败率 < 1% 且失败时保留旧轨迹（不输出空轨迹）。

### Phase E：JPS 前端 + 时间重采样（1~2 周）

- JPS 在 2D 栅格上搜（可用 ESDF 阈值生成的障碍栅格）；`s = k1·Σlen + k2·Σturn` → 梯形加减速 → 均匀 Δt 重采样；必要时加报告的"思路二"（放松约束的 MINCO 预优化）。
- 隧道等狭窄场景的**转角时间分配**在这里定型 —— 报告明确指出折角处时间分配不合理会让后端直接崩。
- **验收**：折角处速度曲线无尖峰；进洞前 1m 开始出现"提前对齐"减速段。

### Phase F：重规划策略 + 收尾（1~2 周）

- 三策略状态机（全局 / 仅优化 / 部分重规划）、起点目标点"排出障碍"、部分重规划的**回溯拼接**（最近投影点往回取一段旧轨迹）。
- ESDF 狭窄度喂给 Phase B 的 yaw 策略（闭环）。
- **验收**：动态障碍插入时横向误差 < 5cm（报告二次期望轨迹要解决的就是这个）；重规划前后位置/速度跳变 < 阈值；整场跑通一次完整比赛流程。

---

## 6. 风险与坑

1. **梯度无效化**（双线性插值）→ 优化器 line search 失败。**必须**二次插值。这是报告花了最长时间定位的问题。
2. **L-BFGS 不收敛** → 抄 DDR-OPT 的终止条件，**并且**保证"优化失败也输出当前最优解"，否则规划器会往外发空轨迹，控制器直接停车。报告 §5.5.4.1 第一步就是这么救回来的。
3. **折角时间分配** → 前端不做运动学处理，后端必然扭麻花。别指望调参绕过。
4. **控制点被推出狭窄区** → PRE 阶段的时间约束是必需的，不是优化项。
5. **两步优化的第一次会"缺少提前对齐"**（报告自己承认的现象）→ 控制器要能容忍，或按 Phase B 的 yaw 策略补。
6. **延迟补偿做反了会更糟**：`t_start + 10ms` 的前瞻量要按我们实际的"求解+通信+下位机 PID"链路重测，不能照抄 10ms。
7. **`yaw` 归属混乱**：MPC 想控真实底盘 yaw，`fake_vel_transform` 想加 `spin_speed_`，两个都想写 `angular.z` 就会打架。迁移前先定所有权（§3.4）。
8. **无 esdf 时的降级路径**：ESDF 服务挂了要有 fallback（退到 costmap 距离变换或纯跟踪），否则等于整车失能。
9. **算力**：报告在 NUC 上跑通了，本项目也是 NUC Ultra7-155h 级别（国赛配置），但 `small_gicp_relocalization` + `terrain_analysis` + 视觉都在抢 CPU，**必须先把 CPU 占用打点**，规划器不能假设独占一个核。
10. **不要让规划器和行为树互相打架**：Phase A~E 期间 BT 的恢复行为（`BackUpFreeSpace` 等）仍在，规划器的重规划策略可能与 BT 的 re-plan 请求重复触发，需要加节流。

---

## 7. 不迁移的部分（明确写下，避免范围膨胀）

- 报告的机械/嵌入式（§3~§4：两舵两全向、力控、主动力矩分配、功率控制）—— 与本项目硬件不同，只作为长期参考。
- 报告的感知 §5.3（3D Occupancy Grid 点云结构化）与 §5.4（可通行区域分割、动态障碍提取）—— 本项目有 `terrain_analysis` + `IntensityVoxelLayer` 等价链路，**先复用**，等 ESDF 跑起来后再评估是否替换。
- 报告的自瞄系统（§6 RW Vision）—— 不是本次范围。
- 报告 §5.5.4.4 里的 Minco "跨模块完全重现"（只传路标点+时间戳+初速）—— 好设计，但要求控制器和规划器共享 MINCO 实现，放在 Phase F 之后再考虑。

---

## 8. 参考

- 报告 PDF：`/home/srm/下载/UTF-8__中科大哨兵2025技术报告.pdf` §5.5（PDF 第 12~24 页）
- 本项目现状：`局部规划器相关(ai).md`
- 关键配置：`src/srm27_navigation/srm27_nav_bringup/config/real/nav2_params_srm.yaml`
- GCOPTER（MINCO + L-BFGS 参考实现）：ZJU-FAST-Lab/GCOPTER
- DDR-OPT（JPS + 梯形加减速重采样 + L-BFGS 终止条件参考）：FAST-Lab 2025
- qpOASES（稠密 QP 求解器，需 vendor）
- FIESTA（增量 ESDF，报告提到 RMUC2026 才实装，我们作为后续优化项）
