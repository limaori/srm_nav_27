# srm27_map_adapter

规划地图**适配层**：把 Nav2 代价图复制成与 ROS 解耦的
[`srm27_map_core::GridSnapshot`](../srm27_map_core/README.md)。

2026-11-24 合并了两份重复实现：

- `srm27_nav_common` 里规划器/平滑器用的 `CostmapSnapshot` 转换（阈值 + 未知策略）；
- `srm27_minco_controller` 里的 `CostmapAdapter`（加锁复制 + 内容版本号）。

现在两边共用同一套加锁复制、代价值映射与 FNV-1a 内容哈希。

## 语义

| 代价值 | `CellState` |
|---|---|
| `NO_INFORMATION` (255) | `kUnknown`（是否不可通行由下游决定） |
| `> occupied_cost_threshold` | `kOccupied` |
| 其余 | `kFree` |

`occupied_cost_threshold` 的两个常用档：

- `253`（MINCO / controller 默认）：只有 `LETHAL_OBSTACLE` (254) 是原始障碍，
  内切膨胀 253 与 1..252 都是自由（半径在碰撞校验里单独计入）；
- `252`（YAstar / SFC 默认）：253/254 都算障碍，用于「中心不可进入」口径。

适配层只做「代价值 → 原始占用」的映射，**不**写膨胀代价；未知格不在这里决定
可通行性，`GridSnapshot::buildBinaryMask()` / `buildByteMask()` 由调用方显式传
`unknown_is_obstacle`。

## 接口

- `makeGridSnapshot(Costmap2D*, config, stamp, snapshot, hash?, reason?)`：
  `CostmapSubscriber` / 裸 `Costmap2D*` 场景使用，可选输出内容哈希。
- `CostmapAdapter`：持有 `Costmap2DROS`，`snapshot()` 同时维护内容版本号
  （只在哈希变化时递增，避免滚动地图让下游版本检查永远失败）。

## 构建与测试

```bash
colcon build --packages-select srm27_map_adapter --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON --parallel-workers 2
colcon test --packages-select srm27_map_adapter --event-handlers console_direct+
```

`test_costmap_adapter` 直接构造 `Costmap2D`，覆盖 253/252 两档阈值、
未知格与内容哈希稳定性。
