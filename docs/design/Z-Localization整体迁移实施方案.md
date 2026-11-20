# Z-Localization 整体迁移实施方案

> 状态：**代码迁入已在 2026-11-16 执行完毕**（见
> [迁入实施记录](../records/2026-11-16-zlocalization-backend-migration.md) 与
> [backend 接入契约](../architecture/localization_backend.md)）：验收 A、B 已通过，
> C–F 未做。新链路在验收通过前保持关闭（默认关闭），不代表已经决定替换当前定位链。
> 本文件保留为实施方案原文，实施结果与偏差以实施记录为准。
>
> 依据：本项目当前源码与 [算法原理与迁移学习指南](Z-Localization算法原理与迁移学习指南.md)、[剩余部分整体迁移问题分析](Z-Localization剩余部分整体迁移问题分析%28ai%29.md)。上游基线为本机 `/home/srm/z-localization` 提交 `76e1263`。

## 1. 目标与范围

本项目已经迁入 ZLIO `frontend` 和 `localization_interfaces`，导航侧通过 `zlio_bridge` 将 ZLIO 观测改写为本项目的 odom 点云/里程计契约。尚未迁入的是上游 `backend`、其离线地图/标定工具和统一 bringup。

本方案要回答并验证：上游 backend 的先验地图跟踪、自动重定位、因子图、回环和 UWB 能否在本项目场景中工作，以及它们是否值得替换或扩展现有的 `small_gicp_relocalization` 与静态 `map → odom` 链。

目标架构候选：

```text
ZLIO Frontend
    │ FrontendFrame / FrontendOdometry / FrontendSession
    ▼
srm27_zlocalization_adapter
    │ 统一 frame、时间戳、odom 原点和点云坐标
    ▼
srm27_zlocalization_backend
    │ 唯一发布 map → odom
    ▼
sensor_scan_generation（仍负责实时 TF 下的雷达到云台/底盘运动学换算）
    ▼
Nav2 / 地形分析 / MINCO
```

后端不直接替代 `sensor_scan_generation`，不让上游固定外参模型输出的车体位姿进入导航。迁移中一次只启用一个 `map → odom` 所有者。当前可选所有者为静态 TF、GICP、SLAM 或 Z-Localization backend，模式必须互斥。

## 2. 范围决策

| 上游内容 | 本方案处理 | 原因 |
|---|---|---|
| `src/backend` | 选择性迁入，ROS 包改名为 `srm27_zlocalization_backend` | 核心定位与离线工具；避免通用包名 `backend` |
| `src/bringup` | 不迁 | 本项目已有脚本与分层 launch，新增通用 `bringup` 入口会造成入口分叉 |
| `src/livox_ros_driver2` | 不迁 | 本项目已有同名包；上游 bringup 的 `launch_ROS2` 路径也不兼容 |
| `localization_interfaces` | 复用本项目现有包 | 前后端消息和 reset 协议已存在；逐字段核对版本兼容 |
| 上游 `frontend` | 不重复迁移 | 本项目已有 ZLIO 前端及适配链 |

建议迁移 backend 源码与地图工具时尽量保留上游目录结构和文件历史；只做包名、构建、安装路径和适配层所需的改动，避免把算法重写和工程接入混为一轮。

## 3. 一次完成代码迁入

代码迁移作为一个完整任务实施，不按验收结果分批导入上游代码。迁入范围确定后，一次完成 backend 算法、离线工具、项目适配、launch/config、测试和文档；新 backend 模式默认关闭，当前正式入口继续使用现有链路。代码具备完整构建与验收入口后，再开始下一节的逐项验收。

### 3.1 迁入前锁定的工程约束

在代码迁移任务开始时一次确认以下约束；这是迁移输入，不是分阶段的代码验收门槛：

- 本次迁入目标包含先验地图跟踪、自动重定位、回环、UWB 中哪些能力；未计划使用的输入/输出通过配置关闭。
- backend 模式下由它唯一发布 `map → odom`；GICP、静态 TF、SLAM 与 backend 必须互斥。
- GPL-2.0-only 代码的仓库发布、产品分发和维护责任。
- 固定 rosbag、场地地图、仿真场景，以及与 GICP 对照的指标门槛。精度、初始化成功率、错误匹配拒绝率、恢复时间、CPU 和内存门槛由项目方在跑测前确定。

保留当前入口的实测基线：TF 发布者、地图来源、初值要求、恢复行为、定位误差、延迟、CPU/内存。迁入代码默认不开启，不修改现有模式的默认行为。

### 3.2 一次迁入的代码清单

1. 将上游 `src/backend` 全量迁入 `src/localization/backend/srm27_zlocalization_backend`，包含 `include/`、`src/`、`apps/`、`tools/`、`launch/`、配置、文档和测试。ROS 包名、CMake project、安装资源和 executable 前缀统一使用 `srm27_zlocalization_backend`；内部 C++ namespace 可暂时保留 `backend`。
2. 一并迁入上游 backend 所需的 6 个地图/标定工具和完整测试。工具包括 `build_pcd_map_bundle`、`clean_map_bundle`、`transform_map_bundle`、`filter_map_bundle`、`rebuild_scan_context_descriptors`、`body_calibration_node`。
3. 不迁上游 `src/bringup` 和 `src/livox_ros_driver2`。本项目复用现有 `livox_ros_driver2`、ZLIO frontend、`localization_interfaces`、脚本入口和分层 launch。
4. 在 `srm27_localization_adapter` 中实现 backend 专用输入适配：同步处理 `FrontendFrame` 的位姿/点云/协方差、`FrontendOdometry` 预测、`FrontendSession` 与 reset boundary；映射到 backend 期望的坐标系、话题、QoS 和时间语义。不能只 remap 名称。
5. 完成 reset/epoch 唯一所有权设计和实现。backend 协调重置时，关闭 frontend 自主换代路径；若保留 bridge 锚定，则向 backend 提供一致的新旧坐标边界，避免 backend 与 bridge 双重补偿。具体方案固定在接口文档与测试中。
6. 将 backend 实验模式一次接入 `localization_launch.py` 及正式脚本可选参数，加入冲突检查：backend 模式不能同时启动静态 `map → odom`、`small_gicp_relocalization` 或 `slam_toolbox`。默认关闭 backend 模式。
7. 仍由 `sensor_scan_generation` 基于实时 TF 产生云台/底盘运动学输出。禁止 backend 固定外参模型的 `map → body`、车体 pose/twist 或其杆臂协方差直接进入导航。
8. 配置 backend 只发布本项目实际需要的输出，关闭无消费者的 legacy 输出和高频 `world_cloud`；地图/日志/轨迹路径使用明确的绝对路径或 package share 解析，不依赖进程 CWD。
9. 解决构建依赖：包内化 `Warnings.cmake`、默认配置、许可证和安装文档；声明 PCL、TBB、Eigen、OpenMP 等依赖；锁定 GTSAM 4.2.x 并检查构建及运行时 ABI。保留 GPL 与第三方许可文本。
10. 加入项目侧测试：包/launch 接线、唯一 TF owner 冲突、frame/time/QoS、适配数学、reset/epoch 故障注入。迁入后整体构建通过之前，不执行验收链路。

迁移时遵守以下实现边界：

- 算法文件尽量贴近上游，项目改动集中在包名/安装、adapter、参数、launch 和测试。
- 当前 GICP、静态 TF、SLAM 的既有默认链路不变；backend 通过单独显式参数启用。
- 地图 bundle 和当前 PGM/PCD 并行保留，迁入代码本身不触发地图格式切换。
- 不把上游固定 `body_from_sensor` 外参解释成支持本车运动云台；在线导航仍使用当前实时 TF 运动学链。

**迁移完成定义：**上述源码、工具、adapter、launch/config、测试和文档均已进入本项目；backend 模式可被显式启动且默认关闭；新旧定位模式冲突会在启动前失败；不存在对上游工作区绝对路径的运行依赖。

## 4. 迁入后的逐项验收

按以下次序验收。发现问题后修复迁入代码并重跑受影响项；不需要因此重新拆分迁移范围或等待某阶段才迁后续代码。

### 验收 A：构建、安装与上游测试

从干净构建目录执行 workspace build，确认所有新增 ROS 包、工具和 launch 资源能安装；检查 GTSAM 4.2.x configure/runtime 版本一致。运行 backend 全部 unit、integration、tool、ROS 测试及项目 adapter/launch 测试，确保没有忽略或跳过的失败项。

**通过标准：**干净构建、安装、测试全通过；运行不依赖 `/home/srm/z-localization`；部署依赖和 GTSAM ABI 有记录。

### 验收 B：地图工具与地图坐标

从正式的 `maps/pcd/<site>_mapframe.pcd` 生成 bundle，不使用未经坐标变换的 `camera_init` 系 `scans.pcd`。验证 bundle 可重复生成、加载、重建描述子和执行自查询/配准；记录描述子类型和参数、地图原点、重力/高度基准、采样高度、体素尺寸及文件校验信息。

`clean_map_bundle` 只接受建图阶段保留的 runtime bundle，不是普通全局 PCD。需要该清理能力时，另按其数据格式留存建图产物并保留 CloudCompare 人工收尾步骤。

**通过标准：**至少一个正式场地地图的 bundle 可重复生成、backend 正常加载，坐标与现有 PGM/PCD 一致，流程可复现。

### 验收 C：输入契约与适配数学

使用固定 rosbag 回放，逐帧核验 `FrontendFrame`、`FrontendOdometry`、点云、协方差、session 的时间戳、frame、QoS 和同步关系。测试首帧 `T_off`、刚体变换对位姿和点云的一致性、协方差坐标变换、仿真/实车 sensor frame，以及云台角变化时 odom 轴保持约定不变。

若 legacy/Point-LIO 输入也保留用于对照，单独验证点云确在声明的 odom 系、里程计是对应扫描末端，且时间同步无一帧偏差；该输入不能绕过对坐标和时间契约的证明。

**通过标准：**数值测试和录包逐帧对照一致，无话题丢失/错误 QoS，无系统性姿态或位置偏差。

### 验收 D：定位算法回放与 reset 故障注入

在 backend 的 map-to-odom 输出尚未接入导航前，对固定 rosbag 和正式地图执行：已知初值跟踪、不同初值误差、错误地图/错配拒绝、走廊等退化几何、重定位和（若启用）回环/UWB。检查 backend 状态、配准质量、创新量、轨迹和资源数据。

注入 LiDAR/IMU 中断、前端失败、backend 超时重试、重复 transaction、进程重启、新 epoch 首帧及移动中 reset。检查输出冻结、事务幂等、图约束和 `odom → base` / `map → odom` 连续性，确保一次故障只有一个连续性补偿所有者。

**通过标准：**正确定位能达到预设基线；错误结果被拒绝；退化状态可识别；reset 不重复补偿、不发布不可信跳变。UWB/回环若不在本次目标内则保持关闭，不宣称通过。

### 验收 E：仿真导航链

只通过显式 backend 参数启动仿真，测试静止、直行、原地旋转、云台旋转、初始位姿误差、输入中断、重定位、退化几何和错误地图。检查 Nav2、terrain、MINCO 所见的坐标/速度，检查平面投影行为，并监控 `/tf` 确认没有第二个 `map → odom` 发布者。

**通过标准：**仿真各场景达到预设指标；TF owner 唯一；无坐标跳变、地面判据异常或控制输入异常。失败时保留 backend 代码但关闭模式，修复后重跑相关验收。

### 验收 F：实车低速与基线比较

先在安全、静态场地低速运行，保留急停和切回原定位模式的操作。使用迁入前冻结的同一地图、bag/init 条件与现有 GICP 做对照，比较定位误差、初始化成功率、错误匹配拒绝率、恢复时间、延迟、CPU、内存和点云带宽。

**通过标准：**所有预先约定的实车指标通过；现场操作和恢复流程可复现；单一 TF owner 持续成立。该项通过后再决定是否纳入正式入口，不以代码迁入完成替代产品采用决策。

## 5. 验收记录矩阵

| 范围 | 必要检查 | 通过证据 |
|---|---|---|
| 构建与部署 | 干净工作区构建、GTSAM 版本/ABI、运行库加载 | 构建日志、依赖清单、部署复现记录 |
| 地图 | 地图原点、重力/高度基准、bundle 加载与描述子 | 地图元数据、重建结果、回放自检 |
| 坐标 | `T_off`、odom 轴向、点云/位姿一致变换 | 单元测试、TF/点云数值对照 |
| 时间 | 同步帧时戳、扫描末端语义、UWB 时间偏差（启用时） | 录包时间分析、对齐误差报告 |
| 估计质量 | 正常、退化、错配、初始化和恢复 | backend 状态、创新/质量日志、真值误差 |
| reset | epoch、重试、失败冻结、连续性所有权 | 状态机测试与故障注入记录 |
| 导航集成 | 唯一 TF owner、2D 投影、下游输入 | TF 检查、terrain/Nav2/MPC 回归 |
| 资源 | CPU、内存、点云带宽、恢复耗时 | 与迁入前基线同环境对照 |

## 6. 回退与采用决策

- 新 backend 模式默认关闭；现有实车/仿真入口的默认定位行为不变。
- 代码可以作为一个迁移提交或同一迁移分支整体评审；验收修复按问题提交。地图格式切换与在线 TF 接管仍须分别开关控制，便于定位问题和回退。
- 保留现有 GICP、静态 TF 和 SLAM 启动路径；切换前用 launch 冲突检查防止双发布。
- 地图 bundle 与 PGM/PCD 并行维护，直到正式采用决策完成；正式地图变更保留原始 PCD、生成参数和 bundle 版本。
- 遇到坐标误差、TF 双发布、reset 不连续、错误匹配被接受、资源超限任一情况，立即关闭 backend 模式并回到迁移前记录的基线。

代码迁入完成只表示功能具备验收条件。实车验收通过且相对 GICP 达到预定收益后，才将 backend 纳入正式入口矩阵、地图流程、部署依赖和现场指南；否则代码可保留为默认关闭的实验模式。

## 7. 执行原则

先一次性完成代码迁入和项目接线，再按验收 A → F 逐项推进。验收阶段允许修复实现并重跑测试，但不再把代码迁移拆成多个阶段。整体验收的核心是证明 backend 能在云台运动、本项目 odom/map 约定和 reset 生命周期下，安全地产生唯一且可验证的 `map → odom`。

## 8. 相关文档

- [Z-Localization 算法原理与迁移学习指南](Z-Localization算法原理与迁移学习指南.md)
- [Z-Localization 剩余部分整体迁移问题分析](Z-Localization剩余部分整体迁移问题分析%28ai%29.md)
- [仿真 LIO 前端替换 Z-LIO 实施方案](仿真LIO前端替换Z-LIO实施方案%28ai%29.md)
- [启动入口矩阵](../architecture/launch_matrix.md)
- [Localization 接口说明](../../src/interfaces/localization_interfaces/README.md)
