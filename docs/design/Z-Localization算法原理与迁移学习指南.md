# Z-Localization 算法原理与迁移学习指南

> 状态：算法调研与学习指南。本文说明 Z-Localization 的核心算法、代码入口，以及迁入本项目时必须保留的算法语义；不是已经验收的整体迁移方案。
>
> 上游源码基线：本机 `/home/srm/z-localization`，Git 提交 `76e1263`。本项目目前已包含 ZLIO `frontend` 和 `localization_interfaces`；因子图 `backend` 尚未并入。上游仓库不可用时，可先按本文的上游相对路径在对应版本中查阅。

## 1. 先建立整体认识

Z-Localization 将状态估计拆成两层：

```text
LiDAR + IMU
    │
    ▼
frontend：高频局部里程计、IMU 传播、去畸变、局部平面地图
    │  同步观测帧：位姿 + 去畸变局部点云；另有高频里程计和会话状态
    ▼
backend：关键帧、先验地图匹配 / 回环 / UWB、增量因子图
    │
    └── map → odom 全局校正、定位状态、可信度和恢复
```

前端回答“短时间内传感器如何运动”，输出连续的 `odom` 轨迹；后端回答“这条局部轨迹在全局地图哪里”，通过全局观测估计 `map → odom`。前端不直接依赖大先验地图，后端也不替代前端的高频惯性传播。

这个边界是理解迁移的第一要点：只迁 `frontend` 得到的是 LIO 里程计能力，并不等于迁入完整的全局定位、重定位和回环能力。本项目目前正是这种状态：导航链使用 ZLIO 前端及 `zlio_bridge`，全局校正由静态 `map → odom` 或 `small_gicp_relocalization` 等本项目组件负责；上游 `backend` 未在运行链中。

## 2. 前端：ZLIO 如何估计局部运动

### 2.1 状态与滤波框架

前端维护 IMU 导航状态，核心量包括姿态 `R`、位置 `p`、速度 `v`、陀螺仪零偏 `b_g`、加速度计零偏 `b_a`，并维护它们的协方差。上游采用 `SE_2(3) ⋉ se(3)` 半直积误差状态表示，而不是把旋转、位置、速度和零偏都当普通欧氏向量相加。姿态在旋转群上更新，其余量按误差定义注入；这减少姿态与速度误差传播中的线性化误差。

忽略噪声项时，导航状态的连续时间运动模型可概括为：

```text
R_dot = R * (omega_m - b_g)^          # 陀螺仪积分姿态
p_dot = v
v_dot = R * (a_m - b_a) + g           # 比力转到世界系后加重力
b_g_dot = random_walk
b_a_dot = random_walk
```

其中 `omega_m`、`a_m` 是 IMU 测量。实现中的误差状态和协方差传播比这组名义状态方程复杂，特别是半直积误差、重力和零偏耦合不能用简单的 15 维欧氏加法替代。

运行时由 IMU 做状态预测，再用 LiDAR 的概率点到面观测更新状态。这属于迭代滤波式 LIO：给定当前位姿估计，先建立点与局部地图平面的关联，计算残差及其不确定度，线性化后修正导航状态；必要时重新关联并迭代。源码重点：`src/frontend/include/zlio/state.hpp`、`prediction.hpp`、`filter.hpp`、`observation.hpp`，实现主要在 `src/frontend/src/prediction.cpp`、`filter.cpp`、`observation.cpp`、`odometry.cpp`。

理解和调试时要区分三个概念：IMU 预测负责高频连续性；LiDAR 更新约束长期漂移；状态协方差表达当前估计及观测的不确定程度。点云匹配成功不代表六个自由度都可观，走廊等退化场景尤其如此。

### 2.2 不要求静止的初始化

常见静止初始化通过加速度均值估计重力，但机器人重启时若仍在运动或云台旋转，这会把运动加速度误认作重力，造成初始姿态错误。ZLIO 的初始化用一段同步 LiDAR/IMU 数据建立短窗口：先用 IMU 给出初值，通过 IMU 传播和点云粗配准形成初始轨迹，再联合优化窗口内多帧的点面几何约束和 IMU 预积分，估计位姿、速度、IMU 零偏及重力方向。优化后重做去畸变并继续迭代；通过几何与数值检查后，坐标系按重力方向对齐，并用窗口末状态和累积点云初始化在线滤波器与局部地图。

迁入时需重点理解初始化的拒绝条件，而不只是看“几秒后有没有 odom”：有效平面数量、窗口连通性、最弱几何方向、联合信息矩阵秩和优化进展都会影响是否交接。源码入口：`src/frontend/include/zlio/initialization.hpp`、`src/frontend/src/initialization.cpp`、`src/frontend/src/bootstrap.hpp`。

### 2.3 IMU 轨迹与逐点去畸变

一帧扫描不是同一时刻采集的。ZLIO 根据 IMU 数据构造扫描期间的连续运动轨迹，并按每个 LiDAR 点的采样时间将点变换到本帧结束时刻的 IMU 坐标系。轨迹段中姿态由角速度积分得到，位置使用加速度及其变化构造连续插值；处理时还要应用 LiDAR 到 IMU 的外参。

因此输入点云的逐点时间字段、单位、时间基准，IMU 时间戳、加速度单位及传感器外参都是算法输入，不是普通 ROS 配置。缺时间字段或时间单位错误时，即使节点仍发布点云和 odom，运动中的扫描也可能无法正确补偿。需要核对 `sensor.*`、`initialization.*` 参数以及 `src/frontend/src/input.cpp`、`trajectory.cpp`、`inertial.cpp`。

### 2.4 概率平面局部地图与点面更新

前端局部地图用稀疏体素索引，体素内部可递归细分：几何简单处保留较大平面，几何复杂处继续分裂。平面由点的累计统计量拟合，使用散布矩阵最小特征值对应的特征向量作为法向；达到稳定条件后可冻结平面统计，控制内存。

点到面残差为 `r = nᵀ(p-c)`。与只按残差大小加权的普通点面 ICP 不同，ZLIO 还考虑：点测量噪声、平面拟合不确定度、表面本身的非平整度，以及同一平面上多个点共享的法向/中心误差。共享误差意味着同一平面上的残差相关；若把它们错误地当独立样本，点数越多滤波器就会越过度自信。实现把同一平面的共享误差在低维平面参数空间中联合处理。

单点残差方差可用下式理解：

```text
sigma_r^2 = nᵀ Sigma_point n + tau^2 + H_plane Sigma_plane H_planeᵀ
```

三项依次表示测量沿法向的噪声、真实表面偏离理想平面的散布，以及拟合平面参数的不确定性。迁移或优化关联代码时，若删掉后两项或把同一平面上的点当成完全独立观测，会改变滤波器对几何约束的置信度。

对应源码：`src/frontend/include/zlio/map.hpp`、`plane.hpp`、`voxel.hpp`，以及 `src/frontend/src/map.cpp`、`plane.cpp`、`storage.hpp`、`observation.cpp`。这是前端最需要在迁移和性能改动中保护的统计语义之一。

### 2.5 时空自适应

前端不是固定“每帧更新一次”或“每点更新一次”。它将点按采样时间分段，每段去畸变并做一次滤波更新；下一段的时间间隔根据近期线/角加速度，以及当前点面观测的信息矩阵自适应。运动变化大且会影响几何残差时缩短间隔，运动平稳或观测不敏感时延长间隔。

空间采样也会自适应：用局部场景尺度与代表点数估计目标采样密度，再调节采样体素边长。它的目的在于不同场景下维持合适计算量和观测信息，而非简单固定下采样比例。参数入口为 `sampling.*`、`window.*`，实现见 `src/frontend/src/sampling.cpp`、`window.cpp`。

上游没有在实时前端启用动态点滤除。其判断是比赛场景中的主要定位几何来自墙面和顶部结构，在线过滤的算力代价不一定换来明显收益；动态残影处理主要放在离线地图流程。迁移时不要假设“ZLIO 自带动态目标剔除”。

## 3. 后端：从 odom 轨迹到全局定位

### 3.1 关键帧因子图

后端以关键帧为变量，状态为关键帧在 `map` 中的位姿。图中主要约束包括：

| 因子 | 来源 | 作用 |
|---|---|---|
| 里程计相对位姿 | 前端相邻关键帧 odom 位姿及协方差 | 保持局部运动连续 |
| 先验地图配准 | 当前扫描与先验地图子图的匹配 | 抑制全局漂移并确定 map 系 |
| 回环相对位姿 | 当前帧与历史帧的重访匹配 | 无先验地图时闭合轨迹 |
| UWB 位置 | UWB 天线的位置测量及杆臂模型 | 提供绝对位置约束 |
| 重力方向 | 前端估计的重力方向 | UWB-only 时约束横滚/俯仰 |

图使用 GTSAM `iSAM2` 增量更新，并可按时间固定窗口边缘化旧位姿、保留其对活动变量的约束。全局观测校正的是图中关键帧及 `map → odom`，而不是把后端当作第二个高频 LIO。源码重点：`src/backend/include/backend/pose_graph.hpp`、`factors.hpp`，`src/backend/src/pose_graph.cpp`、`graph_pose.cpp`、`graph_uwb.cpp`。

从优化角度看，后端求解的是多个观测的加权 MAP 问题：

```text
X* = argmin_X  Σ_k  rho_k( r_k(X)ᵀ W_k r_k(X) )
```

`X` 是活动关键帧位姿及可选的共享标定变量；`r_k` 是里程计、地图匹配、回环或 UWB 因子的残差；`W_k` 来自测量协方差/信息；`rho_k` 表示鲁棒核等处理。地图配准的观测信息若弱或秩亏，就必须如实反映在 `W_k` 中，而不能仅因优化器返回了一个位姿就当作完整可信的 6 自由度定位。

### 3.2 先验地图跟踪与质量判断

正常跟踪阶段，后端按预测位姿选取附近先验地图关键帧/子图，累计近期源扫描，以预测位姿作初值做多层点到面配准。若细层支撑不足或残差较大，可先跑粗层再细化。配准不只返回一个变换，还会从 Hessian/信息矩阵分析观测质量和可观性：强约束方向得到较高信息，弱方向降低权重，不可观方向不应伪造为强约束。

接受过程分层进行：先判断配准模型和几何支撑是否有效，再限制不合理的物理校正幅度，最后将全局观测与图预测做创新检验（NIS/卡方门限）。跟踪中可以接收部分自由度约束；启动锚定/重定位通常要求足够完整、可观的位姿约束。源码：`tracking_matcher.cpp`、`registration.cpp`、`registration_quality.cpp`、`measurement.cpp`、`graph_innovation.cpp`。

### 3.3 描述子、启动定位与重定位

描述子只负责从地图中召回候选位置，不直接证明定位正确；候选必须再通过几何配准和质量验收。

- **Scan Context (SC)**：按距离环和方位扇区栅格化点云，每格记录高度统计，旋转比较用于候选排序和 yaw 初值。对重力调平和高度基准敏感。上游认为它较适合 Mid-360 近全向视场。
- **G-PROBE (GP)**：将有限视场点云建成带占据概率/不确定性的二维网格，并准备不同方向视场分支；候选比较双方共同可见区域。针对有限视场输入，但上游文档明确称测试不足，召回率尚未达到 100%，迁移后必须单独验证。

启动或 LOST 恢复时，后端可能以描述子候选、初值提示或 UWB 位置为线索生成多个假设，再逐个做更严格的几何验证。一个候选“分数高”不是恢复成功，必须满足配准、可观性和创新条件后才能锚定全局位姿。源码：`scan_context.cpp`、`gprobe/`、`relocalization*.cpp`、`recovery_matcher.cpp`。

### 3.4 无先验地图的 UWB 对齐

单个 UWB 位置只能约束位置，无法确定前端 odom 与 UWB/map 之间的水平 yaw。后端收集时间对齐的前端天线轨迹与 UWB 轨迹，估计水平平移和 yaw；横滚/俯仰来自前端重力方向。机器人原地不动时，yaw 不可观；轨迹在水平面展开后才逐渐可观。因此 UWB 初始化依赖运动跨度、时间同步、UWB 坐标系标定和天线杆臂，而不是启动时收到一个点就能定全局姿态。

运行时 UWB 位置可作为图因子；还可选在线估计 UWB 到 map 的对齐与时间偏置。迁移时应先决定是否真的需要 UWB 这条能力，再处理时钟偏差、天线外参和运动条件。源码：`uwb_alignment.cpp`、`uwb_fusion.cpp`、`uwb_timing.cpp`、`graph_uwb.cpp`。

### 3.5 回环与异常状态机

无先验地图时，回环先通过 SC/GP 找候选，再对源/目标点云做相对配准，并检查几何重叠、位姿变化和候选间一致性（PCM）。通过验收的回环才进入位姿图。描述子检索的召回率和配准的误接受率是两类不同风险。

后端状态包括 `INITIALIZING`、`NORMAL`、`OBS_DEGRADED`、`SUSPECT`、`LOST`。弱点云可能意味着观测退化，此时可保留前端里程计；持续的全局观测冲突、速度异常或局部跟踪失效则提高故障等级，进入重定位；前端自身发散时需要 reset 后重新建立局部 odom，再由地图/UWB 找回 map 系。进入 `SUSPECT`/`LOST` 后冻结可信输出，避免把错误跳变继续发给导航。状态转移源码：`localization_state.cpp`、`localization_recovery.cpp`；总流程：`localization.cpp`。

Reset 是前后端协议的一部分：会话 `instance/epoch` 用于识别进程和重置代次，事务 ID 与目标 epoch 用于重试幂等和确认重置结果。若前端重置不能提供新旧坐标系之间的变换，后端只能在明确的“机器人静止”假设下接续。不要将其简化成单纯清空缓存而忽略轨迹连续性和输出冻结边界。

## 4. 迁入本项目时必须掌握的接口语义

### 4.1 前后端的同步帧

`FrontendFrame` 将同一观测时刻的里程计后验、协方差和去畸变点云绑定在一起；后端依赖这些数据在时间、坐标系和观测身份上对应。`FrontendOdometry` 是较高频的惯性传播/预测；`FrontendSession` 通知实例身份、epoch、恢复请求或处理失败；`ResetFrontend` 是带事务身份和目标 epoch 的重置服务。

接口定义与本项目说明见 [localization_interfaces](../../src/interfaces/localization_interfaces/README.md)。迁移后端时需要逐字段核对消息语义、frame、时间戳、协方差定义和 QoS。单纯 remap 话题名不足以解决坐标系或生命周期语义不一致。

### 4.2 坐标系、外参与运动模型

后端默认把传感器到车体的关系作为固定外参；本项目雷达安装在可动 yaw 云台上，`base_link ← lidar` 随时间变化。因此上游后端直接输出的 `map → body`、车体位姿及基于固定杆臂传播的协方差不能未经适配就替代本项目的实时 TF 变换链。后端 `map → odom` 也必须与 `zlio_bridge` 重新定义后的 odom 完全一致；同名 frame 并不能保证原点和轴向相同。

本项目已有 ZLIO 前端集成说明：[仿真 LIO 前端替换 Z-LIO 实施方案](仿真LIO前端替换Z-LIO实施方案%28ai%29.md)。完整后端并入分析另见[剩余部分整体迁移问题分析](Z-Localization剩余部分整体迁移问题分析%28ai%29.md)。本文关注算法语义；这两篇分别记录当前前端适配方式和后端迁移风险。

## 5. 建议的阅读顺序与迁移判断

1. 先读上游 `docs/design.md` 的前后端架构、初始化、去畸变和因子图章节，建立坐标系与估计边界。
2. 再读 `src/frontend/include/zlio/state.hpp`、`initialization.hpp`、`map.hpp`，接着跟 `initialization.cpp`、`observation.cpp`、`odometry.cpp`，理解一次同步观测如何进入滤波器。
3. 只有需要全局地图定位、恢复、UWB 或回环时，再读 backend 的 `localization.cpp`、`tracking_matcher.cpp`、`registration.cpp`、`pose_graph.cpp`，配合上游 backend 参数文档。
4. 将算法需求映射到本项目：只需局部连续里程计时保留 frontend；需要先验地图全局修正时，评估 backend 跟 `small_gicp_relocalization` 的职责是否重叠；需要无先验图定位时，再评估 UWB 对齐和回环，并明确图优化如何拥有 `map → odom`。
5. 在改代码前先固定数据集与坐标契约，分别验证静止初始化、运动初始化、重力方向、扫描畸变、退化几何、先验图跟踪、错误匹配拒绝、reset 连续性。正常运行能输出 pose 不是充分验收条件。

不要只按包名决定迁移范围。算法层面最重要的决策是：后端是否要取代或协同现有 GICP/静态 TF；若两者都发布 `map → odom` 会产生所有权冲突，若两者对 `odom` 的原点约定不同则会出现系统性位姿偏差。

## 6. 上游参考资料

- `/home/srm/z-localization/docs/design.md`：算法动机与主要公式。
- `/home/srm/z-localization/src/frontend/README.md`、`docs/parameters.md`：前端输入输出、参数与传感器要求。
- `/home/srm/z-localization/src/backend/README.md`、`docs/parameters.md`：后端接口、状态机、地图工具和参数。
- `/home/srm/z-localization/src/localization_interfaces/README.md`：前后端消息及 reset 协议。
- 本项目已迁入实现：[frontend](../../src/localization/frontend/frontend/)；[接口包](../../src/interfaces/localization_interfaces/)。
