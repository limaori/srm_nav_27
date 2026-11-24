# srm27_nav_smoother

轨迹规划**后端**：Nav2 `Smoother` 插件，把 `srm27_nav_common` 里 vendored 的
T-DT MinimumSnap（OSQP 求解）与 SfcSquare 方形安全走廊包装成路径平滑器。

| 项 | 值 |
|---|---|
| 插件 ID | `srm27_nav_smoother::MinimumSnapSmoother` |
| 基类 | `nav2_core::Smoother`（Humble：`CostmapSubscriber` + `FootprintSubscriber`） |
| 输入/输出 | `nav_msgs/msg/Path` 就地写回；另发 `visualization_msgs/msg/MarkerArray` 走廊 |
| 地图输入 | `srm27_map_adapter::makeGridSnapshot`（Nav2 costmap → `srm27_map_core::GridSnapshot`），SFC 栅格由 `srm27_map_core::makeSfcGrid` 生成 |
| 上游算法 | MinimumSnap + SfcSquare @ `tdt-nav-kit` `814d5ce0`（MIT，源码内置在本包 `vendor/src/MinimumSnapOsqp/`） |

2026-11-24 起，参数与调用流程直接对齐上游 `main.cpp:145-164`：
`setOrder(6)`、`setDt(0.1)`、`setTL(10)`、`backend=OSQPCorridor`、起点速度 `{0,0}`；
输入路径不切角、不加密，失败时也不回退。只保留 Nav2 包装层（costmap→SFC 栅格的
语义转换、origin、生命周期、话题和 Marker 发布）。

行为树必须在 `ComputePathToPose` 与 `FollowPath` 之间插 `SmoothPath`
（`srm27_nav_bringup/nav2_bt/navigate_to_pose_w_smoothing.xml` 已内置）。

## 包内结构

```text
srm27_nav_smoother/
├── vendor/src/MinimumSnapOsqp/ # 上游 MIT 源码（minimumSnap.* + sfcSquare.*）
├── vendor/LICENCE              # MIT 原文
├── include/srm27_nav_smoother/ # 插件头与 SFC Marker 头（Apache-2.0）
├── src/                        # minimum_snap_smoother / sfc_visualization
├── test/                       # pluginlib、Marker、SFC 原点/边界补丁
├── PATCHES.md                  # 上游最小补丁 P3/P6/P7/P10 与已知缺陷
└── THIRD_PARTY_NOTICES.md      # 来源、commit、sha256、许可边界
```

`vendor/` 只在本包内使用（CMake 用 `PRIVATE` include），安装到
`share/srm27_nav_smoother/` 与 `include/srm27_nav_smoother/vendor/` 仅为溯源，不导出给下游。

## 启用

实车当前参数（`config/real/current/nav2_params_srm_minco_fakeyaw.yaml`）已经默认使用；
仿真是按需开关：

```bash
ros2 launch srm27_nav_bringup nav_srm_simulation_launch.py use_minisnap_smoother:=True
ros2 param get /smoother_server smoother_plugins   # ["minimum_snap_smoother"]
```

实例名在 YAML 里是 `minimum_snap_smoother`，参数前缀是
`smoother_server.minimum_snap_smoother.*`。

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `occupancy_threshold` | `252` | SFC 判障阈值；与规划器保持一致 |
| `treat_unknown_as_obstacle` | `true` | 动态层留下的未观测格是否按障碍（实车用 `false`） |
| `order` / `max_dx` | `6` / `3` | 多项式**系数个数**（6 = 五次）/ 最大导数阶数 |
| `strict_collision` | `false` | 严格碰撞检查（更慢、更易失败） |
| `backend` | `corridor` | `corridor`（上游 main.cpp）\| `path` \| `close` |
| `collision_iter` | `6` | 碰撞迭代轮数（上游 6） |
| `norm_time` | `true` | 段内归一化时间 |
| `max_corridor_range` | `2.5` | 走廊最大扩张范围（米），内部硬夹紧到 `1e6` |
| `corridor_shrink` | `0.0` | 走廊初始收缩（米）；迭代中自动减半 |
| `max_speed` / `max_acc` | `3.0` / `1.0` | 只用于时间分配，不是硬约束 |
| `orientation_mode` | `tangent` | `identity` 或 `tangent` |
| `publish_sfc` | `true` | 是否发布走廊 MarkerArray |
| `sfc_topic` | `~/sfc_corridor` | 解析为 `/<smoother_server>/sfc_corridor` |
| `sfc_line_width` | `0.02` | 方块线宽（米） |

## 失败语义

- QP 不收敛或超时（上游 `setTL(10)`）：**不改动传入路径**，返回 `false`；
  上游没有闭式兜底，也没有本仓库此前的加密折线 fallback；
- 代价地图不可用：原样返回路径，节流 WARN；
- `backend=corridor` 以方形走廊为硬约束；上游方块扩张会跳过未检查的环（方块内可能
  含障碍，见本包 `PATCHES.md` 的「方块扩张缺陷」）。

## SFC 走廊可视化（RViz）

每次平滑后向 `~/sfc_corridor` 发一个 `MarkerArray`，同一个话题里有两组方块和两行文字：

- **黄色细线**：`getCorridor()` 的初始自由方块，未经过碰撞迭代 `shrink`。看“路径两侧
  实际还剩多少空间”用这一组；它不随求解失败消失。
- **橙色粗线**：求解迭代后真正当约束用的 `result.corridor`（可能插入过控制点、被
  `shrink` 成小方块）。橙色明显小于黄色，说明求解在反复收紧；OSQP 失败时橙色为空。
- **白色文字**：两行，`SFC free (yellow): N box(es) W×H m` 与
  `SFC final (orange): ...`，放在路径中点上方。`final: no result` 表示 OSQP 没解出来。

`srm27_nav_bringup/rviz/nav2_default_view.rviz` 的 Global Planner 组已预置
`SFC (yellow=free, orange=final)` 显示（相对名 `smoother_server/sfc_corridor`）。

```bash
ros2 topic hz /smoother_server/sfc_corridor
```

## 构建与测试

```bash
colcon build --packages-select srm27_nav_smoother --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON --parallel-workers 2
colcon test --packages-select srm27_nav_smoother --event-handlers console_direct+
```

测试覆盖：MarkerArray 结构（`test_sfc_markers`）、pluginlib 注册
（`test_pluginlib_load`）、SFC 原点补丁 P7（`test_sfc_origin`）与 P10 走廊落帧
（`test_sfc_bound_frame`）；SFC 栅格语义/边界补圈在 `srm27_map_core`（`test_sfc_grid`）。
