# archive

这里只放**不再适用于当前工作区**的材料。每篇文档开头应标注：

- 状态：历史资料
- 当前是否适用：否
- 替代入口：当前架构文档或正式入口

当前内容：

| 文件 | 原用途 | 为什么归档 |
|---|---|---|
| `TF与fake底盘详解(ai).md` | 旧步兵 fake 虚拟底盘/TF | 旧 `gimbal_yaw_fake`/`MecanumDrive2` 执行链已删除；`fake_vel_transform` 组件本身仍用于当前链路，见入口表 |
| `tdt开源.md` | TDT 上游算法调研 | 调研时的链路图/入口已过时，算法结论可参考 |
| `legacy_viz/` | 旧 `viz/` 可视化 launch 与 RViz 配置 | 不在四个正式入口内 |

不要在这里新增“当前架构/操作指南”；这类内容放 `docs/architecture/` 或 `docs/operations/`。
