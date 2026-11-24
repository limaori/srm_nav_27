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
| `order` / `max_dx` | `5` / `3` | 多项式阶数 / 最大导数阶数 |
| `strict_collision` | `false` | 严格碰撞检查（更慢、更易失败） |
| `backend` | `path` | `path`（默认）\| `close` \| `corridor`（仅验证用，见下） |
| `collision_iter` | `2` | 碰撞迭代轮数（实车 6） |
| `norm_time` | `true` | 段内归一化时间 |
| `max_corridor_range` | `10.0` | 走廊最大扩张半径（米），内部硬夹紧到 `1e6` |
| `corridor_shrink` | `0.0` | 走廊收缩（米） |
| `max_speed` / `max_acc` | `2.0` / `2.0` | 只用于时间分配，不是硬约束 |
| `min_points` | `2` | 少于此点数不平滑，直接返回成功 |
| `path_resample_step` | `0.3` | 平滑前加密步长（米）；`0` 会让 `path` 后端不收敛 |
| `path_resample_max_points` | `2000` | 加密点数上限 |
| `fallback_to_resampled` | `true` | 求解失败/超时时回退到加密折线 |
| `corner_round_cut` | `0.4` | 折角切角长度（米） |
| `corner_round_min_turn_deg` | `25.0` | 小于该转角不切 |
| `corner_round_sample_step` | `0.2` | 贝塞尔采样步长（米） |
| `init_vel_from_odom` | `false` | 起点速度取里程计 |
| `odom_topic` / `init_vel_max_age` | `odom` / `0.3` | 起点速度来源与有效窗口 |
| `orientation_mode` | `tangent` | `identity` 或 `tangent` |
| `publish_sfc` | `true` | 是否发布走廊 MarkerArray |
| `sfc_topic` | `~/sfc_corridor` | 解析为 `/<smoother_server>/sfc_corridor` |
| `sfc_line_width` | `0.02` | 方块线宽（米） |

## 失败语义

- QP 不收敛或超时：**不改动传入路径**，返回 `false`；
  `fallback_to_resampled=true` 时改写为已加密折线并返回 `true`（`was_completed=false` 的
  语义由 smoother_server 记 WARN，行为树不会因此重规划）；
- 代价地图不可用：原样返回路径，节流 WARN；
- `backend=corridor` 以方形走廊为硬约束，上游方块扩张会跳过未检查的环（方块内可能
  含障碍，见本包 `PATCHES.md` 的「方块扩张缺陷」），因此只建议用于验证。默认 `path`
  不受影响。

## SFC 走廊可视化（RViz）

每次平滑后向 `~/sfc_corridor` 发一个 `LINE_LIST` Marker（固定 ns/id，数量变化不留残影）：

- `backend=corridor`：画求解迭代里真正当约束用的方块；
- `backend=path/close`：结果里只有退化点，改为按当前 costmap 现场生成一次
  `SfcSquare::getCorridor()`，用来看「路径两侧还剩多少余量」。

`srm27_nav_bringup/rviz/nav2_default_view.rviz` 的 Global Planner 组已预置
`SFC Corridor` 显示（相对名 `smoother_server/sfc_corridor`）。

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
