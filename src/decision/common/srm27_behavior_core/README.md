# srm27_behavior_core

决策模块的共享底座（header-only）：服务端 `srm27_behavior` 与 BT 插件包
`srm27_behavior_plugins` 都要用的状态/算法，放在这里避免两份拷贝。

| 头文件 | 内容 |
|---|---|
| `srm27_behavior_core/waypoint_utils.hpp` | YAML/CSV 航点、命名点/路线、内联航点解析、`makePoseStamped` |
| `srm27_behavior_core/semantic_map.hpp` | 语义区多边形加载与点所属判断 |
| `srm27_behavior_core/navigation_spin_tracker.hpp` | 导航到点/离点滞回、Action 完成、自转状态同步到黑板 |
| `srm27_behavior_core/spin_reference.hpp` | 掉血反制的随机变速参考曲线（五次多项式、限加速度/加加速度） |
| `srm27_behavior_core/sentry_posture_state.hpp` | 姿态请求预算、5 秒冷却与累计时间 |

包内没有 `.cpp`、没有 ROS 节点、没有 `BT_REGISTER_NODES`；C++ 命名空间仍为
`srm27_behavior`（与另外两个决策包共用）。

## 测试

```bash
colcon build --packages-select srm27_behavior_core --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON --parallel-workers 2
colcon test --packages-select srm27_behavior_core --event-handlers console_direct+
```

`test_decision_core` 覆盖：到点/离点滞回与坐标系切换、内联/YAML/CSV 航点拒绝与接受、
导航完成后按停车点判到点、`SpinReference` 的平滑/限幅/可复现/无固定周期。
需要真实 BT 节点的姿态用例在 `srm27_behavior_plugins`（`test_set_sentry_posture`）。

## 分层

```text
srm27_behavior（服务端/裁判输入/决策树） ──► srm27_behavior_core
srm27_behavior_plugins（BT 动作/条件/控制/装饰器） ──► srm27_behavior_core
```

前端（插件）与后端（服务端）之间没有编译依赖；服务端在运行时从
`srm27_behavior_plugins/bt_plugins` 目录加载 `.so`。
