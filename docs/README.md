# SRM27 文档索引

文档按**用途与状态**分目录，不再把方案、实施记录、现场日志混在一起。

| 目录 | 放什么 | 状态 |
|---|---|---|
| [`architecture/`](architecture/) | 当前架构、TF、话题、速度链、唯一入口表 | 当前有效 |
| [`operations/`](operations/) | 启动、建图、定位、调试、合并操作指南 | 当前有效 |
| [`design/`](design/) | 尚未完成或仍在演进的设计方案 | 设计/提案 |
| [`records/`](records/) | 已完成的实施记录、验收记录、现场日志 | 历史记录（结论仍可引用） |
| [`archive/`](archive/) | 过时方案、旧链路、AI 生成的历史材料 | 历史资料，不再适用 |

> 入口与参数的唯一权威表是 [`architecture/launch_matrix.md`](architecture/launch_matrix.md)。
> 其他文档里的入口说明若与它冲突，以入口表为准。

## architecture

- [启动入口矩阵（唯一入口表）](architecture/launch_matrix.md) —— 四个正式入口、16 个 launch 的 current/compat/deprecated 分类、参数文件分类、fake_vel_transform 当前职责。
- [ROS 包清点](architecture/package_inventory.md) —— 48 个包的分类、路径与职责。
- [仿真数据流](architecture/仿真数据流%28ai%29.md) —— 仿真链的话题、frame、控制路径（AI 生成，已按 SRM 链更新）。
- [哨兵导航 TF 与下位机数据接口](architecture/哨兵导航TF与下位机数据接口%28ai%29.md) —— MID-360 安装、传感器分工、上下位机约定。
- [导航通信协议：下位机接口说明](architecture/导航通信协议-下位机接口说明.md) —— 串口帧格式、速度反馈、状态包。

## operations

- [MID360 使用指南](operations/mid360使用指南.md) —— 网络配置与驱动启动。
- [导航速度调试指南](operations/导航速度调试指南.md) —— 限速、加速度、前瞻、PID。
- [导航避障调试指南](operations/导航避障调试指南.md) —— 代价地图、膨胀、净空、恢复行为。
- [分支合并说明](operations/分支合并说明.md) —— `--no-ff` 合并与 CLion 操作。

## design

- [MINCO 与 MPC 局部规划控制器从零详解](design/MINCO与MPC局部规划控制器从零详解.md) —— 面向初学者的原理与源码讲解。
- [MINCO 迁移实施方案](design/迁移minco实施方案%28ai%29.md) —— 目标架构、阶段与验收（AI 生成）。
- [局部规划器迁移 MINCO/MPC 方案](design/局部规划器迁移MINCO_MPC方案%28ai%29.md) —— 早期分阶段方案。
- [局部规划器相关](design/局部规划器相关%28ai%29.md) —— 当前控制器与候选方案对比。
- [SRM 仿真与导航自转控制实施方案](design/SRM仿真与导航自转控制实施方案.md) —— SRM-only 仿真、平面速度、独立自转。
- [wz 控制迁移导航与 fakeyaw 仿真实施方案](design/wz控制迁移导航与fakeyaw仿真实施方案.md) —— 目标链路、接口改动、回退开关。
- [导航改云台系与决策层小陀螺实施方案](design/导航改云台系与决策层小陀螺实施方案.md) —— 云台系导航与 `rotation_cmd` 的职责边界。
- [GICP 初始重定位仿真验证方案](design/GICP初始重定位仿真验证方案.md) —— 地图对齐、误差矩阵、验收指标。
- [仿真 LIO 前端替换 Z-LIO 实施方案](design/仿真LIO前端替换Z-LIO实施方案%28ai%29.md) —— ZLIO 接入与三级回退。
- [Z-Localization 剩余部分整体迁移问题分析](design/Z-Localization剩余部分整体迁移问题分析%28ai%29.md) —— 迁移风险与三档路线。

## records

- [2026-11-14 工作区清理记录](records/2026-11-14-workspace-cleanup.md) —— 本轮数据迁移、脚本/launch/docs 重整与 Git 清理。
- [MINCO 迁移实施记录](records/minco迁移实施记录%28ai%29.md)
- [MINCO 实车迁移实施记录与上电验收清单](records/minco实车迁移实施记录%28ai%29.md)
- [MINCO 脱困机制迁移实施记录](records/minco脱困机制迁移实施记录%28ai%29.md)
- [MINCO 连续导航修复记录](records/minco连续导航修复记录%28ai%29.md)
- [wz 控制迁移与 fakeyaw 实施记录](records/wz控制迁移与fakeyaw实施记录%28ai%29.md)
- [SRM 仿真与自转控制实现](records/SRM仿真与自转控制实现%28ai%29.md)
- [仿真链迁移实车实施记录](records/仿真链迁移实车实施记录%28ai%29.md)
- [仿真云台系改造实施记录](records/仿真云台系改造实施记录%28ai%29.md)
- [云台系改造实施记录](records/云台系改造实施记录%28ai%29.md)
- [仿真 LIO 前端替换 Z-LIO 实施记录](records/仿真LIO前端替换Z-LIO实施记录%28ai%29.md)
- [调试日志——by maori](records/调试日志——by%20maori.md)
- [旧版完整 README（2026-11-14 归档）](records/README_full_20261114.md) —— 重构前的完整项目说明，仅供追溯。

## archive

- [TF 树与 fake 虚拟底盘详解](archive/TF与fake底盘详解%28ai%29.md) —— **历史资料**：旧步兵 fake 链路，当前 SRM 链路不适用。
- [TDT 导航算法移植记录](archive/tdt开源.md) —— **历史资料**：上游算法调研，旧链路图已过时。
- [`legacy_viz/`](archive/legacy_viz/) —— 旧可视化 launch/rviz 配置。

## 新增文档的规则

1. 先说清状态：当前架构 / 操作指南 / 设计提案 / 已完成记录 / 历史资料。
2. 已完成的实施记录放 `records/`，不要修改成“当前架构”口吻；需要更新结论时另写一篇。
3. 历史资料放 `archive/`，文件开头写：状态、当前是否适用、替代入口。
4. 不把运行日志、CSV、PNG、rosbag 放进 `docs/`；实验产物放 `~/srm_nav_27_data/`。
