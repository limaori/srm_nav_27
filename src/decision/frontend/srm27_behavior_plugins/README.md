# srm27_behavior_plugins

决策**前端**：BehaviorTree.CPP 4.x 插件。服务端 `srm27_behavior` 在启动时枚举
`share/srm27_behavior_plugins/bt_plugins/*.so` 并注册到行为树工厂。

| 类别 | 库 | 节点 |
|---|---|---|
| 动作 | `srm27_behavior_action_plugins` | `SendNav2Goal`、`NavigateThroughPoses`、`PubNav2Goal`、`PubRotationCmd`、`PubSpinSpeed`、`PubTwist`、`PubGimbalAbsolute`、`PubGimbalVelocity`、`SetSentryPosture`、`MuxStopControl`、`CalculateAttackPose` |
| 条件 | `srm27_behavior_condition_plugins` | `IsGameStatus`、`IsStatusOK`、`IsRfidDetected`、`IsAttacked`、`IsHpDropping`、`HysteresisCheck`、`IsInSemanticZone` |
| 控制 | `srm27_behavior_control_plugins` | `RecoveryNode` |
| 装饰器 | `srm27_behavior_decorator_plugins` | `NavigationTransitionGate`（含 `RecoveryWindow`）、`RateController`、`TickAfterTimeout` |

所有节点注册名保持不变（决策树 XML 不需要改）。C++ 命名空间仍是 `srm27_behavior`。

## 约束

- 插件只依赖共享底座 `srm27_behavior_core`，不依赖服务端包；
- 需要真实 ROS 话题/Action 的节点在 `tick()` 里用法与原来一致；
- 插件头文件路径为 `srm27_behavior_plugins/<action|condition|control|decorator>/...`，
  共享状态/算法头文件在 `srm27_behavior_core/...`。

## 构建与测试

```bash
colcon build --packages-select srm27_behavior_plugins --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON --parallel-workers 2
colcon test --packages-select srm27_behavior_plugins --event-handlers console_direct+
```

`test_set_sentry_posture` 用真实 `SetSentryPosture` 节点验证姿态 4 秒冷却、
累计时间预算与回退姿态；其余无节点依赖的决策核心用例在
`srm27_behavior_core/test/test_decision_core.cpp`。
