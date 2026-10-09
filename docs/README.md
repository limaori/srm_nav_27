# Documentation

- [分支合并说明](分支合并说明.md): use `--no-ff` to preserve development commits and create a merge commit, with CLion instructions.
- [MID360 setup](mid360使用指南.md): network configuration and LiDAR bringup.
- [哨兵导航 TF 与下位机数据接口](哨兵导航TF与下位机数据接口(ai).md): MID-360 安装与底盘自旋的四种组合、传感器分工及上下位机通信约定。(AI 生成)
- [Simulation data flow](仿真数据流(ai).md): topics, frames and control paths in simulation (updated for the SRM chain). (AI 生成)
- [Controller comparison](局部规划器相关(ai).md): current controller and alternatives.
- [MINCO + MPC 从零详解](MINCO与MPC局部规划控制器从零详解.md): 面向初学者，结合当前源码讲解 ESDF、MINCO 轨迹优化、MPC 跟踪、线程与速度链，附参数说明、仿真练习、排错方法和实现边界。
- [MINCO migration implementation plan](迁移minco实施方案%28ai%29.md): 基于中科大 2025 技术报告、当前 SRM 代码和北理 navi_minco_bit 的迁移方案，含源码复用清单、接口修正、MINCO/MPC 设计、阶段验收与回退；测试从 0.5 m/s 起步，上限 1.0 m/s。(AI 生成)
- [MINCO/MPC migration notes](局部规划器迁移MINCO_MPC方案(ai).md): staged design proposal.
- [MINCO migration implementation record](minco迁移实施记录(ai).md): P0/P1/P2 落地记录 —— 新增 vendor/core/controller 三个包、odometry 语义修复、MPC 与 MINCO 关键实现选择、构建/测试/启动命令、未完成项与已知缺口。(AI 生成)
- [TDT navigation notes](tdt开源.md): upstream navigation implementation review.
- [SRM simulation and independent rotation plan](SRM仿真与导航自转控制实施方案.md): SRM-only simulation, planar Nav2 velocity, and separate chassis rotation control.
- [SRM simulation and rotation implementation notes](SRM仿真与自转控制实现(ai).md): 落地说明 —— 三个新包的职责、速度/自转接口表、合成与安全优先级、启动方式、测试矩阵与实测结论。(AI 生成)
- [TF and fake chassis deep dive](TF与fake底盘详解(ai).md): 旧步兵 fake 虚拟底盘（`gimbal_yaw_fake`）与 TF 详解，历史资料，SRM 仿真已不再使用。
- [调试日志——by maori](调试日志——by%20maori.md): 现场调试记录。

Build output, runtime logs, rosbag files, and generated diagnostic plots are kept outside this documentation tree.
