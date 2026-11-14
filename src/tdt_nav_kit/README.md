# tdt_nav_kit

把东北大学 T-DT 开源组件 [`tdt-nav-kit`](https://github.com/T-DT-Algorithm-2026/tdt-nav-kit)
包装成两个 Nav2 插件：

| 插件 | 基类 | 上游算法 |
| --- | --- | --- |
| `tdt_nav_kit::TdtGlobalPlanner` | `nav2_core::GlobalPlanner` | YAstar（分层 Mask 的 A*） |
| `tdt_nav_kit::TdtSmoother` | `nav2_core::Smoother` | MinimumSnap（OSQP 求解）+ SfcSquare 走廊 |

- 上游版本：commit `814d5ce0237acc7faa8c070e5c5e61e2ddc81996`（tree `3274cebb...`，2026-10-09）
- 许可：`vendor/**` 为 MIT（见 `vendor/LICENCE`）；本包其余文件为 Apache-2.0
- 来源与补丁：`THIRD_PARTY_NOTICES.md`、`PATCHES.md`

## 范围

| 做 | 不做 |
| --- | --- |
| 两个插件、Costmap 快照转换层、5 个 gtest | KinodynamicAstar（约 2 s）、动态障碍 / 多层 Mask 调优 |
| 输出 `nav_msgs/msg/Path`（`map` 帧），不改 `cmd_vel` | 动 Costmap / 行为树 / 控制器 / 底盘 |
| 新增独立参数文件与启动封装 | 修改任何既有文件 |

上游 8 个源码全部入库；CMake 编译 `yastar.cpp`、`minimumSnap.cpp`、`sfcSquare.cpp`，
**不编译 `kinodynamicAstar.cpp`**（上游自述全局搜索约 2 s，主链路不用）。

## 初版移植范围

1. 改动只在分支 `hj_dev`（基于 `b2d6fc2`）上，`main` 不提交。
2. 初版只新增 `src/tdt_nav_kit/`。后续仿真修复同步更新了 SRM 仿真入口和 RMUC 2025 地图。

## 启动前置（重要）

`src/srm27_navigation/srm27_nav_bringup/package.xml` **没有**把本包声明为
`exec_depend`（约束 2 的代价）。pluginlib 靠 ament resource index 发现插件，
只要本包已 build 且 overlay 已 source 就能加载，但：

- `rosdep install --from-paths src` 不会自动装本包的系统依赖；
- `colcon build --packages-up-to srm27_nav_bringup` **不会**带上本包；
- 二进制打包（bloom / .deb）会漏装本包 → 生产部署需显式补上。

因此**每次都要显式单独构建并 source**：

```bash
cd <workspace>
colcon build --packages-select tdt_nav_kit --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON --parallel-workers 2
source install/setup.bash            # Windows: install\setup.bat
ros2 pkg prefix tdt_nav_kit          # 应输出 install 路径
```

## 阶段二依赖（OSQP / OsqpEigen）

`TdtSmoother` 需要 OSQP 与 OsqpEigen，装法与阶段一无依赖的情况不同：

```bash
# 1) OSQP：ROS 包直接装（头文件在 /opt/ros/humble/include/osqp，库在 /opt/ros/humble/lib）
sudo apt install ros-humble-osqp-vendor

# 2) OsqpEigen：没有 rosdep key，源码装到 /usr/local
git clone --depth 1 -b v0.8.0 https://github.com/robotology/osqp-eigen.git
cmake -S osqp-eigen -B osqp-eigen/build -DCMAKE_BUILD_TYPE=Release \
      -DOSQP_EIGEN_BUILD_TESTS=OFF -DOSQP_EIGEN_BUILD_EXAMPLES=OFF
sudo cmake --build osqp-eigen/build -j2 && sudo cmake --install osqp-eigen/build

# 3) 关键一步：把 /usr/local/lib 加进动态库搜索路径
#    否则运行期报 libOsqpEigen.so.0.8.0: cannot open shared object file
#    （表现为 colcon test 全部 "1 test, 1 error"，pluginlib 实例化也失败）
echo /usr/local/lib | sudo tee /etc/ld.so.conf.d/usr-local.conf && sudo ldconfig
```

`package.xml` 里只声明了 `<depend>osqp_vendor</depend>`；OsqpEigen 没有 rosdep key，
无法在其中声明，CMake 用 `find_package(OsqpEigen REQUIRED)` 直接找。

## 启动

```bash
# 默认启动 RMUC 2025、YAstar 和 MinimumSnap
ros2 launch tdt_nav_kit tdt_nav_simulation.launch.py use_rviz:=False

# 用 SimpleSmoother 做对照
ros2 launch tdt_nav_kit tdt_nav_simulation.launch.py \
  use_rviz:=False \
  params_file:=$(ros2 pkg prefix tdt_nav_kit)/share/tdt_nav_kit/config/nav2_params_srm_tdt.yaml

# 接入已有 Gazebo 时加 start_simulation:=False
# 切换回 SRM 原有导航链路
ros2 launch srm27_nav_bringup nav_srm_simulation_launch.py
```

TDT 入口默认加载与 Gazebo 世界同名的地图。从 `sim_config_file` 读取出生位姿，
作为 `map → odom` 的默认平移和航向。使用其他坐标原点的地图时，可显式传入
`map_to_odom_x`、`map_to_odom_y` 和 `map_to_odom_yaw`。

`rmuc_2025.pgm` 用 `python3 script/build_rmuc_2025_map.py` 从 Gazebo 碰撞网格生成，
保留平地并标记底盘高度范围内的障碍。YAML 原点为 `[-0.10, -0.10, 0]`，分辨率 0.05 m。
旧测试目标 `(12, 3)` 位于障碍区域，可用默认出生点 `(3.4, 9.5)` 到 `(10, 5)` 验证导航。

**不要**传 `speed_limits_file`。原 launch 的默认值已是显式绝对路径
`config/simulation/speed_limits.yaml`，与本包的参数文件所在目录无关；
若改成 `auto`，`ReplaceString` 会让参数文件落到临时目录，
`srm_speed_limits.py` 会静默不合并限幅，退回插件默认高值（Omni 3.0 m/s）。

单目标行为树由 Nav2 选用各配置中唯一注册的平滑器。SimpleSmoother 版本在 YAstar
输出时加密路径，MinimumSnap 版本在平滑阶段加密，并处理只有两个输入点的直线路径。
两份文件都使用 `srm27_omni_pid_controller::OmniPidPursuitController`，速度限制由
`speed_limits.yaml` 的 `@omni` 档位提供。

### 构建机接口已核对（2026-11-04）

按 `ros-navigation/navigation2` 的 `humble` 分支头文件核对并据此实现：

- `nav2_core::GlobalPlanner::createPlan(start, goal)` 是**两参**，没有较新版本的 `cancel_checker` 第三参数；
- `configure` 的 `name` 为按值 `std::string`，`parent` 为 `rclcpp_lifecycle::LifecycleNode::WeakPtr`；
- `Costmap2D::getMutex()` 返回 `std::recursive_mutex*`；`getCharMap()` 返回 `unsigned char*`；
- `cost_values.hpp` 的 `NO_INFORMATION / LETHAL_OBSTACLE / INSCRIBED_INFLATED_OBSTACLE / MAX_NON_OBSTACLE / FREE_SPACE` 名称一致；
- `PlannerException` 位于 `nav2_core/exceptions.hpp`。

### 真实构建验证（2026-11-04，ROS 2 Humble 容器）

在 Ubuntu 22.04 + ROS 2 Humble 容器中（脚本 `D:/ros2/run_tdt_build_in_docker.sh`）：

- `colcon build --packages-select tdt_nav_kit` **通过**（阶段一约 26 s，阶段二约 18 s）；
- `colcon test` **5 个测试可执行文件、32 个 gtest 用例全部通过**，`colcon test-result` 汇总 32 项、0 errors / 0 failures；
- pluginlib 加载测试证明 `TdtGlobalPlanner` 与 `TdtSmoother` 均已注册到对应基类、可被实例化，且来自同一个共享库。

构建期修正：`ament_auto_add_library()` 内部通过 `ament_target_dependencies()` 使用
`target_link_libraries` 的 plain 签名，因此本包链接 Eigen3 / OsqpEigen 时也**不能写 `PUBLIC`**，
否则 CMake 报 all-plain / all-keyword 混用。`SolveInput::setTimeAllocated()` 取的是非 const
引用，必须传具名 `std::vector<double>` 左值。另有一个非致命警告：`ament_auto_package`
建议使用 `USE_SCOPED_HEADER_INSTALL_DIR`；本包头已是 `include/tdt_nav_kit/*.hpp`
布局，保持默认非 scoped 才能让下游 `#include "tdt_nav_kit/xxx.hpp"` 找到，故保留。

> 本机 WSL 发行版是 Ubuntu 24.04，而 ROS 2 Humble 只支持 22.04，因此验证放在
> Ubuntu 22.04 容器内进行，未改动 WSL 宿主机系统，也未往本仓库写构建产物。

## 验证

```bash
colcon test --packages-select tdt_nav_kit --event-handlers console_direct+
colcon test-result --all --verbose
ros2 param get /planner_server GridBased.plugin        # tdt_nav_kit::TdtGlobalPlanner
ros2 param get /smoother_server tdt_smoother.plugin    # 仅 smooth 变体：tdt_nav_kit::TdtSmoother
ros2 param get /controller_server FollowPath.plugin    # 仍为 OmniPidPursuitController
```

启动日志不应出现 `Failed to create planner` / `Failed to create smoother`
或 `[srm_speed_limits] 警告`。5 个 gtest 分别覆盖：Costmap 语义/索引/快照、
SFC 反向占据语义与边界补圈、P7 原点符号与两套坐标换算一致性、
YAstar 链路与 SDF 一致性、pluginlib 加载。

## 仿真实测（2026-11-05，Ignition Fortress 无头容器）

在容器里跑起完整链路：Ignition Fortress 6.18（`rmuc_2025` 世界 + `rmuc_2025.yaml` 地图）
+ Nav2（`nav2_params_srm_tdt_smooth.yaml`），机器人出生点改到 (3.4, 5.0)
（默认 (3.4, 9.5) 在地图 y 范围 [-9.44, 6.76] 之外，Nav2 无法规划）。

两个插件都在真实 Nav2 栈里加载成功：

```
ros2 param get /red_standard_robot1/planner_server GridBased.plugin    -> tdt_nav_kit::TdtGlobalPlanner
ros2 param get /red_standard_robot1/smoother_server tdt_smoother.plugin -> tdt_nav_kit::TdtSmoother
```

同一起终点 (3.4, 5.0) → (12, 3)，两者路径都经逐 2 cm 采样验证**未穿障、未出图**：

| 链路 | 点数 | 路径长度 | 规划耗时 | 平滑 |
| --- | --- | --- | --- | --- |
| ThetaStar + SimpleSmoother（基线） | 598（间距 0.074 m） | 33.16 m | ~102 ms | SimpleSmoother 未改变路径 |
| YAstar + TdtSmoother | 11 → **101** | 34.87 → 37.10 m | ~45 ms | **1 ms**，最大步长 6.06 → **0.78 m** |

规划拓扑一致（两条都绕过同一片墙，说明不是我们的转换层把地图读歪），
YAstar 的路径点少 54 倍、规划快约 2 倍；TDT 平滑器把 6 m 的稀疏折线插成 0.78 m
的连续轨迹，代价是路径长度 +6%。

### 仿真暴露并已修掉的三个问题

1. **配置写 `1.0e9` 被 YAML 1.1 当成字符串** → `smoother_server` 配置期直接报
   `parameter ... is invalid type ... setting it to {string} is not allowed`，整个
   导航栈起不来。改成 `10.0`。
2. **`max_corridor_range` 转 int 溢出**：上游 `_getBound()` 里
   `static_cast<int>(maxRange / mapping)`，1e9 / 0.05 = 2e10 超出 int，
   走廊扩张越界读，**直接把 component_container 打崩**（无 OOM 记录、无异常输出）。
   代码里加了硬性夹紧 `kMaxCorridorRangeLimit = 1e6`，默认值降到 10 m。
3. **`corridor` 后端在真实地图上不可行**：OSQP 返回 -4（dual infeasible）/ -6（超时），
   稀疏与稠密路径都不收敛；同一路径 `close` 后端 0.2~19 ms 解出且无碰撞。
   当时把默认后端改成了 `close`。

   > 2026-11-05 补充：加入路径加密（见下文 `path_resample_step`）后重测三种后端
   > （11 点 / 34.46 m → 加密到 1191 点）：
   >
   > | backend | 结果 | 耗时 |
   > | --- | --- | --- |
   > | `path` | 成功 | **2 ~ 17 ms** |
   > | `close` | 成功 | 83 ms |
   > | `corridor` | **失败**（OSQP 不收敛）| 408 ms |
   >
   > 三者输出几何一致（34.56 m、最大步长 0.04 m、无碰撞），默认后端据此改为
   > **`path`**。corridor 仍然不可用，且换后端前必须先确认加密是开的。

`max_speed / max_acc` 一度从 2.0/2.0 改到 **50/20**，依据是上游代码注释
「建议 speed 大于 40，acc 大于 10」。后续对照实验**否掉了这个判断**：

| 参数 | 结果 | 最大曲率 | 平滑耗时 |
| --- | --- | --- | --- |
| 3.0 / 1.0（上游 README） | 1441 点 / 36.56 m / 无碰撞 | **5.64 rad/m** | 45 ms |
| 50 / 20 | 1431 点 / 36.58 m / 无碰撞 | 5.96 rad/m | 26 ms |

两者都能解，上游值轨迹更平顺。现默认值已回退到 **3.0 / 1.0**，代价是平滑
35 ms（原 26 ms），仍在 3 Hz（333 ms）预算内；要更快可改回 50/20。

### 平滑耗时 vs 控制点间距（2026-11-07 实测）

上游的 3.90 ms（RMUC2026 图）是在**十几个控制点**上测出来的；我们为了让它按输入点
判碰撞的逻辑可用，先把路径加密到 0.30 m，控制点因此多一个数量级。间距是唯一的
调节旋钮，实测（rmuc_2025，12 点 / 36.6 m，`path` 后端）：

| `path_resample_step` | 平滑耗时 | 平均曲率 | 最大曲率 | 2 cm 采样穿障 |
| --- | --- | --- | --- | --- |
| **0.30 m（默认）** | 35 ms | **0.438 rad/m** | 5.64 rad/m | 0 |
| 0.80 m | 28 ms | 0.722 | 5.81 | 0 |
| 1.20 m | **10 ms** | 0.862 | 5.81 | 0 |
| 2.00 m | 11 ms | 0.999 | 5.85 | 0 |

**耗时能降 3.5 倍，代价是平均曲率翻倍**：控制点变稀后，多项式在相邻点之间会"飘"，
不再紧贴折线。上游敢用稀疏控制点，是因为 `OSQPCorridor` 的方形走廊把轨迹摁在路径
附近；而我们这条路径 corridor 后端不收敛（见上），没有这层约束兜底。

结论：**默认保持 0.30 m**。35 ms 相对 3 Hz 的 333 ms 预算还有 10 倍余量，没有理由
拿轨迹质量去换。若将来平滑成为瓶颈，1.20 m 是性价比最高的档（10 ms / 曲率 0.86）。

### 长路径「几乎不动」的成因与修复（2026-11-08）

**现象**：短目标（2~3 m）能正常跑完；跨图长目标（30 m 以上）表现为车原地微动、
最终 `ABORTED`。

**成因链**（逐段实测确认）：

1. 平滑器 `MinimumSnap` 在起点附近有动态障碍标记时不肯收敛——
   `OSQP solve failed (iter=6)`，6 轮碰撞迭代都没找到无碰撞轨迹；
2. 失败分支当时返回**原始稀疏路径**（上游规划器段长最大 10.4 m）；
3. Nav2 控制器按局部代价地图半径（5 m 图的 2.5 m）裁剪全局路径，
   稀疏路径一裁只剩 **1 个点**，前视点落在车身上；
4. 控制器据此只给 **0.19 m/s** 的爬行速度，行为树判「无进展」后中止。

**触发源**：`obstacle_min_range: 0.2` 小于车体半径 0.274 m，雷达扫到自身，
出生点周围被标成代价 75 的「障碍」；任何从该点出发的平滑都会在第 1 轮就判碰撞。

**修复**：

| 改动 | 内容 |
| --- | --- |
| `obstacle_min_range` | 0.2 → **0.55 m**（局部/全局两处）。须大于车体半径，否则恒有自击点 |
| `fallback_to_resampled` | 新增参数（默认 `true`）：求解失败/超时时返回**已加密的折线**（0.30 m 间距），而不是原始稀疏点，保证控制器始终有可跟踪的路径 |

**验证**：单测 40 项全绿；同一路径平滑输出 1401 点 / 间距 0.029 m，
首末点与输入偏移 **0.000 m**；出生点代价由 75 降到 0。

**遗留**：控制器在该场的推进速度约 0.19~0.25 m/s（原工程的 PID 与前视参数），
36 m 路径需 2~3 分钟，超出行为树默认的进度检查窗口。这是控制器侧的调参问题，
不属于本包范围。

### 上游接口的接入取舍（2026-11-07 复核）

上游提供的接口并非都要接，逐个说明理由，避免"看起来全移植了"的误读：

| 上游接口 | 状态 | 理由 |
| --- | --- | --- |
| `YAstar::search` / `setCostField` / `findSavePoint` | ✅ 已接 | 主规划链路 |
| `MinimumSnap::solve` + `setMap/setOrder/setMaxDx/setTL` | ✅ 已接 | 主平滑链路 |
| `setInitVel` | ✅ 已接（开关 `init_vel_from_odom`）| 实车换目标时避免横向加速度突变；默认关闭以免改变既有行为 |
| `setItems`（途经点速度/走廊） | ❌ 不接 | **Nav2 的 SmoothPath 只传位姿，没有速度信息**；没有速度源时逐点约束只能全填默认值，等于没接。真正有意义的只有起点速度，已由 `setInitVel` 覆盖 |
| `lineInObsticle` | ❌ 不接 | 仅用于测试断言，规划链路不调用 |
| `setCostWeight` | ❌ 不接 | 用库内默认权重；调它属于调参，不是移植缺口 |
| `KinodynamicAstar` | ❌ 不编译 | 上游自述全局搜索约 2 s，`planner_server` 的 1 s 超时下必然失败；等有"离线/低频重规划"需求时再评估 |

### 闭环验证：行为树必须补 SmoothPath（2026-11-05 追加）

原 `navigate_to_pose_w_replanning_and_recovery.xml` 是「ComputePathToPose → FollowPath」
直连，**没有 SmoothPath 节点**——也就是说 `smoother_server` 只在手动调用动作时用过，
真实导航链路里从来没生效过。

后果在 TDT 规划器上尤其明显：YAstar 输出的是稀疏路径（短目标只有 2~11 个点），
而 `srm27_omni_pid_controller::transformGlobalPlan()` 会按局部代价地图范围裁剪，
把超过 `max_costmap_extent`（本配置 5×5 m → 2.5 m）的点全部丢掉。
实测裁剪后 `local_plan` **只剩 1 个点、距机器人 0.03 m**，
`getLookAheadPoint()` 拿不到前视点，`theta_dist` 退化成噪声，
速度趋零 → "车不动 + Failed to make progress"。

本包新增 `behavior_trees/navigate_to_pose_w_smoothing.xml`：在规划与跟随之间插入
`SmoothPath`（Humble 端口名：`unsmoothed_path` / `smoothed_path`），
两份参数文件的 `default_nav_to_pose_bt_xml` 已指向它。

效果：同一目标下单次导航的位移从 **0.31 m / 150 s** 提升到 **5.43 m / 120 s**，
`local_plan` 从 1 点恢复到 14 点，前视点方位角稳定。

#### 平滑前必须先加密路径（`path_resample_step`）

把 `SmoothPath` 的 `check_for_collisions` 打开后，平滑会**必然失败**：
上游 `MinimumSnap` 的碰撞检查按输入点连线/样条采样判定，而 YAstar 的段长可达
5.7 m，样条在拐角处切进膨胀层，于是 `was_completed=false`、原样返回。

本包因此在 `TdtSmoother` 内加了 `path_resample_step`（默认 0.30 m）：
求解前先把相邻点按该步长线性插值，段长降到 0.3 m 量级，碰撞检查才能通过。
`path_resample_max_points`（默认 2000）防止病态输入撑爆内存。

实测（同一条 11 点 / 34.46 m 路径，`rmuc_2025` 地图）：

| 配置 | 结果 |
| --- | --- |
| 不加密 + 开碰撞检查 | `was_completed=false`，原样返回 11 点 |
| 0.30 m 加密 + 开碰撞检查 | **`was_completed=true`，1191 点，83 ms** |
| 0.15 m 加密 + 开碰撞检查 | `was_completed=true`，2341 点，**669 ms**（超出 3 Hz 预算） |

取 0.30 m：83 ms 在 3 Hz（333 ms）预算内。0.15 m 虽然更细，但耗时涨 8 倍，
且最大曲率反而更差——因为曲率是按相邻三点估的，采样越密越能分辨出
**原路径本身就有的折角**（原折线总转角 545°），不是平滑引入的。

> 注意：这一条不是 TDT 特有的缺陷。用仓库原始链路（ThetaStar + SimpleSmoother）
> 跑同一目标，行为与 TDT 完全一致——说明稀疏路径 + 无平滑是**全栈**的问题，
> 只是 TDT 的路径更稀而先暴露。

#### 折角圆弧化（`corner_round_cut`）

MinimumSnap 对输入点是**插值**的，折线本身的尖角会被原样保留：一条 575° 总转角的
YAstar 路径上，平滑后仍出现 24 rad/m 的瞬时曲率（转弯半径 4 cm），底盘无法跟随，
控制器只能切内弯。

`roundCorners()` 在拐点两侧各退 `cut` 米，用二次贝塞尔（控制点=原拐点）连接。
曲线始终落在两条相邻线段构成的三角形内，只要原折线无碰撞，切角一般也不会引入碰撞。
顺序必须是**先切角再加密**，反过来会把贝塞尔重新拉回折线。

同一条输入路径（12 点 / 37.27 m）的 A/B：

| 配置 | 输出点数 | 长度 | 最大曲率 | 99% 分位 | 中位 | 平滑耗时 |
| --- | --- | --- | --- | --- | --- | --- |
| `corner_round_cut: 0.0`（关） | 1301 | 37.37 m | **24.04 rad/m** | 7.42 | 0.11 | 14 ms |
| `corner_round_cut: 0.40`（开） | 1431 | 36.58 m | **5.64 rad/m** | 4.11 | 0.02 | 22 ms |

最大曲率降到 1/4.3，路径还短了 0.8 m，代价是平滑耗时 14 → 22 ms（3 Hz 预算内）。
开启后闭环回归通过：出生点 → (6.00, 4.50)，2.27 m / 7.2 s，`SUCCEEDED`。

`corner_round_min_turn_deg`（默认 25°）用于跳过小转折，避免把近似直行的路径切出波动。

### 闭环验证：已跑通（2026-11-05 追加）

闭环最终跑通，两次实测均为 action `SUCCEEDED`：

| 起点 | 目标 | 位移 | 耗时 | 结果 |
| --- | --- | --- | --- | --- |
| (3.40, 5.00) 出生点 | (6.00, 4.50) | 2.24 m | **6.8 s** | SUCCEEDED |
| (2.67, 5.72) 途中 | (6.00, 4.50) | 3.18 m | 43.9 s | SUCCEEDED |

跑通过程中修掉两个**与插件无关**的问题，但缺了任何一个闭环都不可能通：

**1. `map -> odom` 必须是出生位姿，不能是单位变换**

Gazebo 速度插件的 `odom` 帧以「出生点」为原点（`odom -> base_link` 从 0 开始），
而地图是世界坐标。仓库里 `map_to_odom_x/y` 默认 0.0，于是：

```
TF  odom->base_link : (+0.260, +0.972)
chassis_odometry_gt : (+3.660, +5.972)      ← 世界坐标
偏差 6.05 m（正好等于出生位姿）
```

Nav2 按 TF 认为车在地图原点，规划出以 (0,0) 为起点的路径，现场表现是「车朝固定方向
跑、最后冲出地图」。把 `map_to_odom_x/y` 设成出生位姿后，
`bt_navigator` 的日志从 `Begin navigating from current location (0.00, 0.00)`
变成 `(3.40, 5.00)`，规划随之正确。

**2. 目标点必须落在空闲栅格上**

之前所有「不收敛」的测试，目标都设成了 `(6.0, 3.5)`——查地图才发现该处灰度 0，
**本身就是障碍**。目标在墙里时，规划器只能规划到最近的可达点，平滑器再判定碰撞、
行为树进恢复，最后表现为「走一米就中止」。换成空闲的 `(6.0, 4.5)` 后立即成功。

> 教训：验证闭环前先查目标点的栅格值，否则会把「用例错」误判成「系统错」。
> 两个问题都会伪装成"车不动/跑偏"，与插件行为难以区分。

高台式结论：**TDT 插件在仿真里可以完成端到端导航**，规划 26~53 ms、平滑 2~17 ms
（1191 点），全程开碰撞检查。

## 阶段二进展与遗留

已完成：OSQP（`osqp_vendor`）+ OsqpEigen v0.8.0 接入；`minimumSnap.cpp` /
`sfcSquare.cpp` 纳入编译；`TdtSmoother` 插件；SFC 反向语义转换与边界补圈；
P7 原点补丁经单测验证；OSQP 失败 / 超时一律保留原路径并返回 `false`。

仍遗留（需真机或仿真数据才能定论）：

- **126.7 ms 长尾**：2026 地图上 Minimum Snap 的最大求解耗时，需用真实路径采样确认是否超平滑预算；
- **corridor 不裁剪**：上游 `_getBound()` 显式不判边界，纯自由图上走廊会外扩到 `max_corridor_range`，
  建议在参数里限制该值（默认 1e9 等价于不限）；
- **`obstacleTolerance = 2` 硬编码**：非严格模式下检测线可"擦过" 2 个障碍栅格，穿薄墙不算碰撞；
- **`setDt()` 实际不生效**：`lineDecoder` 每段固定 11 点；
- **与 `MincoMpcController`（`odom` 帧 / 50 Hz）叠加**：需统一帧与朝向语义后再联调；
- **未接动态 Mask 与多层 Mask**：`mask_num_layers` 已暴露但只用于静态图。
