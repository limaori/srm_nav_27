# 导航维护约定

- `srm27_minco_core` 是无 ROS 的算法层；控制器与 `BackUpFreeSpace` 共用脱困方向与线段检查。
- 速度必须服从当前命令系。fakeyaw 链由 `fake_vel_transform` 完成唯一一次旋转；不得在恢复分支固定输出车体系速度。
- 配置值以 YAML 为准，恢复设计、验证和边界集中在根目录 `docs/recovery.md`；不要在 XML/YAML 堆积调参历史。
- `script/validate_recovery.sh` 在隔离容器内构建并验证恢复功能；运行产物位于 `.cache/recovery-validation`。禁止连接实车或复用其他任务的运行目录。
- 修改后台规划状态时，同时覆盖过期的成功、失败和到点结果；清理操作必须使正在计算的旧结果失效。
