# 仿真 `--lio` 前端替换 ZLIO 实施记录

> 对应方案：[仿真LIO前端替换Z-LIO实施方案(ai).md](../design/仿真LIO前端替换Z-LIO实施方案%28ai%29.md)
> 对应入口：`./script/start_sim_nav.sh --lio --lio-frontend zlio`
> 记录日期：2026-11-05

---

## 0. 一句话结论

**方案里"只换第 ① 层（谁产生 odom←雷达）"的分层落地成功**：新增 `zlio_bridge` 一个脚本 +
两个上游包，改的是 4 个 launch 的条件判断，下游（`sensor_scan_generation` / `terrain_analysis*` /
`fakeyaw` / 速度链 / `map→odom`）**一行未动**，且默认行为（`point_lio`）与迁移前完全一致。

| 阶段 | 内容 | 状态 |
|---|---|---|
| P0 | 拷 `frontend` + `localization_interfaces`、编译、zlio 单独起得来 | ✅ 完成（含配置严格校验的正/反例） |
| P1 | `zlio_bridge` 单独验（位姿/点云/换代锚定） | ✅ 完成（离线端到端 16 项全过） |
| P2 | 全链路 `--lio` | 🟡 **已起过一次**（2026-11-05 实跑，暴露并修复了 `intensity` 字段告警刷屏，见 §5.3）；静置/直行/自转三段量化仍未做 |
| P3 | A/B 量化对比（位移倍数/航向/点数/CPU） | ⬜ **未跑** |
| P4 | 云台自转下的稳定性 | ⬜ **未跑** |
| 坐标数学 | 🔴→✅ 曾因「T_off 少乘 `R_0⁻¹`」把 odom 系整体转了约 30°（§5.5），已修并加**纯数学回归测试** | 完成（2026-11-05） |
| 默认值 | **已切换**：仿真链（`start_sim_nav.sh`、`nav_srm_simulation_launch.py`）缺省 `zlio`；导航栈/实车入口仍 `point_lio` | 完成（按需求，2026-11-05；见 §6.3） |

> P2~P4 是"要不要换"的唯一证据链，本记录不代跑、也不代替结论。

---

## 1. 实测数据（本机，2026-11-05）

| 项 | 命令/来源 | 结果 |
|---|---|---|
| 编译两个新包 | `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select localization_interfaces frontend` | 2 包成功，frontend 59.7 s；**0 warning**（上游 `-Werror` 是开着的） |
| 产物 | `install/frontend/lib/frontend/` | `zlio_node`、`zlio_visualizer` |
| 算法配置严格校验（正例） | `ros2 run frontend zlio_node -p config_file:=zlio_mid360.yaml` | 静默启动（该节点启动时只发 session，不打日志），6 s 无报错 |
| 算法配置严格校验（反例） | 同上，但 YAML 里插一个 `sensor.bogus_key` | `ROS node failed: unknown configuration key: bogus_key`，进程 exit 1 → **证明配置文件确实被解析**，不是被静默忽略 |
| launch 布线 | `.sim_run/check_launch_wiring.py`（一次性脚本，见 §8） | 17/17 通过（4 种前端×组合组合矩阵） |
| `zlio_bridge` 端到端 | `.sim_run/test_zlio_bridge.py`（一次性脚本，见 §8） | 16/16 通过，关键数值见 §3.3 |
| 包回归测试 | `colcon test --packages-select srm27_nav_bringup` | 3/3 通过（含新增 `test_lio_frontend_wiring.py`） |
| 负向验证（测试真的能抓错） | 把组合路径那条 `loam_interface` 条件改回去 | 新测试立即失败并指出"zlio 前端下 loam_interface 必须关闭 (use_composition=True)" |
| launch 烟测（zlio） | `ros2 launch ... use_lio_odometry:=True odometry_frontend:=zlio`（不起 Gazebo） | `zlio_node` + `zlio_bridge.py` 起来；**`loam_interface` 出现 0 次**；bridge 从参数文件读到 `planarize_pose=False / anchor_on_epoch_change=True` |
| launch 烟测（point_lio） | 同上但 `odometry_frontend:=point_lio` | `pointlio_mapping` + `loam_interface_node` 起来，无 zlio → **旧行为无回归** |

烟测里 `local_costmap` 反复报 `Timed out waiting for transform from base_link_fake to odom` 是**预期**的：
该烟测故意不起 Gazebo 与 `robot_state_publisher`，没有任何 TF 源。

---

## 2. 代码改动清单

### 2.1 新增（方案 §6.1）

| 路径 | 内容 |
|---|---|
| `src/frontend/` | z-loc `src/frontend` 原样拷入（含 `apps/ src/ include/ test/ launch/ rviz/ docs/`）；**仅构建路径做了包内自持改造**，见 §4.1 |
| `src/localization_interfaces/` | 同上（`FrontendFrame` / `FrontendSession` / `FrontendOdometry` / `ResetFrontend`） |
| `srm27_nav_bringup/config/simulation/zlio_mid360.yaml` | ZLIO 算法配置，72 项逐项复制自 z-loc `config/lio.yaml`，只改 `sensor.body_from_lidar.translation: [0,0,0]`（对齐 point_lio 的 `extrinsic_T`），其余值保持上游默认，文件头写清"为什么是这些数" |
| `srm27_nav_bringup/scripts/zlio_bridge.py` | §42 行的桥接节点：`T_off` 缓存、位姿复合、点云按当时位姿变换、换代锚定、契约自检 |
| `srm27_nav_bringup/test/test_lio_frontend_wiring.py` | 前端接线的 launch 回归测试（方案 §6.2 的"漏改一条不会报错"就靠它兜住） |
| `srm27_nav_bringup/scripts/imu_saturation_guard.py` | 给 ZLIO 补上 Point-LIO 的 `satu_acc` 那一层：`|a|` 限 30 m/s²、超限 WARN+计数（见 §5.4） |

没有拷的东西（按方案要求）：`livox_ros_driver2`（本仓库 v1.1.0 已满足，`msg/` 两边 `diff -r` 为空，
同名包无法共存）、`backend/`、`bringup/`。

### 2.2 修改（方案 §6.2 的 4 个 launch + 3 个构建/参数文件 + 1 个脚本）

| 文件 | 改动 |
|---|---|
| `launch/localization_launch.py` | ① 新增 `odometry_frontend`（默认 `point_lio`）与 `zlio_config`（默认指向包内 `zlio_mid360.yaml`）<br>② `start_point_lio_node` 条件改为 `(use_pcd_localization or use_lio_odometry) and frontend == point_lio`<br>③ 新增 `zlio_node`（条件 `use_lio_odometry and frontend == zlio`），仿真参数与输出 remap 全写在 launch 里并逐条注明依据<br>④ 新增 `zlio_bridge` Node（同一条件）<br>静态 `map→odom` 节点**未动** |
| `launch/nav2_stack_launch.py` | ① 透传 `odometry_frontend` 给两个 include<br>② 新增声明<br>③ 新增取值校验 `OpaqueFunction`：拼错 / `zlio` 缺 `--lio` / `zlio` + `slam` 三种直接让 launch 失败 |
| `launch/navigation_launch.py` | ① 新增 `odometry_frontend` 声明<br>② 派生 `use_loam_interface = (有里程计来源) and (前端 == point_lio)`<br>③ **两处** `loam_interface` 启动路径都换成新条件：`:170` 的 `Node(...)` 与 `:432` 的 `LoadComposableNodes(...)`<br>其余（terrain remap、`sensor_scan_generation`、`fake_vel_transform`、平滑器、速度出口）**一行未动** |
| `launch/nav_srm_simulation_launch.py` | 声明 `odometry_frontend` 并透传给 `nav2_stack_launch` |
| `srm27_nav_bringup/CMakeLists.txt` | `install(PROGRAMS ...)` 加 `scripts/zlio_bridge.py`、`scripts/imu_saturation_guard.py`；`BUILD_TESTING` 的 `foreach` 加 `test_lio_frontend_wiring` |
| `srm27_nav_bringup/package.xml` | 新增 `exec_depend`: `frontend`、`localization_interfaces`（`tf2_sensor_msgs` 本来就有） |
| `config/simulation/nav2_params_srm_fakeyaw.yaml` | 新增 `zlio_bridge:` 段（方案 §7.2）与 `imu_saturation_guard:` 段，每个键都带注释；`point_lio` / `loam_interface` 两段**保留不动** |
| `script/start_sim_nav.sh` | ① 新增 `--lio-frontend {point_lio\|zlio}`（环境变量 `LIO_FRONTEND`）<br>② 校验：非法值直接退出；`zlio` 但没有 `--lio`（或走 `--gicp`）直接退出并打印正确用法<br>③ `--lio` 分支把 `odometry_frontend:=${LIO_FRONTEND}` 拼进 `NAV_EXTRA_ARGS`<br>④ 启动检查：选 `zlio` 时要求 `frontend` 已构建，否则给出 `colcon build` 命令<br>⑤ 启动横幅区分两种前端<br>⑥ 文件头用法/参数说明补 `--lio-frontend` |

### 2.3 明确**没有**动的东西（与方案 §6.3 对齐）

`sensor_scan_generation`（一行未改）、`map→odom` 静态发布、`terrain_analysis(_ext)` 的 remap、
`fake_vel_transform`、`srm_cmd_mux`、`gimbal_spin_gate`、`ign_sim_pointcloud_tool`、
`point_lio` / `loam_interface` 的参数段、地图与出生点。

---

## 3. 关键实现细节（与方案一致的部分，附落地后的确认）

### 3.1 `zlio_bridge` 的数学

```text
T_off   = TF(base_link ← front_mid360) @ 首帧 stamp，只查一次并缓存
T_odom←lidar(k) = T_off · R_k
cloud_odom      = do_transform_cloud(frame.cloud, T_odom←lidar(k))
lidar_odometry  = { frame_id: odom, child_frame_id: front_mid360, stamp = frame.odometry 的 stamp }
```

`do_transform_cloud` 之后**显式**重设 `frame_id` 与 `stamp`——实测该库会把输出 `stamp` 清成 0
（`rclpy` 版本如此），而 `sensor_scan_generation` 的 `ApproximateTime` 同步器正是按 `stamp`
配对 `lidar_odometry` 与 `registered_scan`。

### 3.2 换代（epoch）锚定

```text
epoch 变化时： T_off ← T_off · R_last_old · R_first_new⁻¹
```

以 `frame.epoch` 为触发源（每帧都带），`session` 只做诊断（`failed` / `reset_required` 显式报错）。
`anchor_on_epoch_change: false` 时明确 WARN"odom 会跳一次、地形图会错位"——不静默降级。

### 3.3 端到端验证通过的关键数值

用一个已知外参 `T(base_link←front_mid360) = t(0.1, 0.2, 0.3) + yaw 1.0 rad` 做离线验证：

| 用例 | 期望 | 实测 |
|---|---|---|
| 原始位姿 = 单位阵 | 输出位姿 = `T_off` | `(0.1000, 0.2000, 0.3000)`，yaw = 1.0 ✅ |
| 雷达系点 `(3,0,0)` | `R_off·p + t_off` | `(1.7209, 2.7244, 0.3000)` ✅（**不是**只旋转不平移） |
| 前进 1 m（雷达系 x+1） | `T_off · R_k` | `(0.6403, 1.0415, 0.3000)` ✅ |
| 同帧点云 | 随位姿一起动 | `(2.2612, 3.5659)` ✅ |
| epoch 0→1（原始位姿跳回单位阵） | 位姿连续、不跳 1 m | 换代前 `(0.640302, 1.041471)` → 换代后 `(0.640302, 1.041471)` ✅ |
| 点云坐标系 ≠ `child_frame_id` | 整帧丢弃 | 输出 0 条 ✅（日志：`丢弃该帧: 点云坐标系 'livox_frame' != odometry.child_frame_id 'front_mid360'`） |
| `session.failed = true` | 显式 ERROR | `ZLIO 会话失败 (instance=1, epoch=2): 前端已停止且禁止自动重试` ✅ |

---

## 4. 与方案不一致的地方（逐条给出理由）

### 4.1 两个上游包改成了"包内自持"路径（唯一动到上游文件的地方）

z-loc 的 `src/frontend` 与 `src/localization_interfaces` 里有 4 处 `../../` 相对路径，指向
z-loc **仓库根**的 `cmake/Warnings.cmake`、`config/lio.yaml`、`LICENSE`。拷进本工作空间后这些
路径会指向本仓库根目录。处理方式是把依赖收进各包内（**不改任何算法/CMake 逻辑**）：

| 原路径 | 新路径 |
|---|---|
| `src/frontend/CMakeLists.txt` 的 `../../cmake/Warnings.cmake` | `cmake/Warnings.cmake`（已从 z-loc `cmake/` 拷入） |
| `src/frontend/CMakeLists.txt` 的 `../../LICENSE` / `../../config/lio.yaml` | `LICENSE` / `config/lio.yaml`（均已拷入，`config/lio.yaml` 也顺便让上游 `lio.launch.py` 仍可用） |
| `src/frontend/test/CMakeLists.txt` 的 `${PROJECT_SOURCE_DIR}/../../config/lio.yaml`（4 处） | `${PROJECT_SOURCE_DIR}/config/lio.yaml` |
| `src/localization_interfaces/test/CMakeLists.txt` 的 `../../cmake/Warnings.cmake` | `cmake/Warnings.cmake`（已拷入） |

**为什么不在仓库根造 `cmake/` + `config/` + `LICENSE`**：那样会在仓库根多出一个 GPL-2.0 的
`LICENSE` 文件，等于对整仓库的授权做出错误声明（本仓库其它包是 Apache-2.0）。

> ⚠ **授权边界**：`frontend` 与 `localization_interfaces` 上游是 **GPL-2.0-only**，与本仓库其它
> 包不同。两个包的 `package.xml` 与包内 `LICENSE` 都保持上游原样，没有改写。

### 4.2 输出 QoS：用 reliable，而不是方案写的 `SensorDataQoS`

方案 §4.1 建议两个输出都用 `SensorDataQoS`（best-effort）。落地时**不能**这么做：

```cpp
// terrain_analysis/src/terrainAnalysis.cpp:255
nh->create_subscription<nav_msgs::msg::Odometry>("lidar_odometry", 5, odometryHandler);
//                                     ↑ 默认 QoS = RELIABLE, KEEP_LAST(5)
```

best-effort 发布者与 reliable 订阅者**建不上连接**（QoS 不兼容），表现是 `terrain_analysis`
收不到位姿、地形图完全不出——又是一种静默失败。所以两个输出都改成
`QoSProfile(depth=5, reliability=RELIABLE)`（与 `loam_interface` 的原实现一致）。
反方向是安全的：`sensor_scan_generation` 的订阅是 best-effort，能收 reliable 发布者的数据。

### 4.3 `zlio_bridge.py` 的可执行位

`install(PROGRAMS ...)` + `--symlink-install` 只做软链，**不保证目标文件的可执行位**。
新写的脚本初始权限是 `600`，装完之后 `ros2 run srm27_nav_bringup zlio_bridge.py` 报
`No executable found`（launch 里则会让整个启动失败）。已 `chmod 755`。
> 以后往 `scripts/` 加脚本时，写完就 `chmod +x`，否则这个错会在 `ros2 run` 才暴露。

### 4.4 `T_off` 的"精确时刻 → 最新可用"显式降级

方案 §4.5 要求"`T_off` 没查到之前丢帧"。实现按此执行，但额外加了一条**显式**降级：
精确时刻查询失败时，退化为"最新可用变换"，并打一次性 WARN + 在 INFO 里标注
`（退化: 用最新可用变换，云台此时应为零位）`。

理由与本仓库 `sensor_scan_generation.cpp::getTransform()` 的原则一致：**降级要可见，不要伪装成
单位变换**。启动阶段云台被 `gimbal_spin_gate` 锁在零位，此刻的最新变换与精确时刻等价；
如果将来 gate 被删掉，这条 WARN 会立刻出现。

### 4.5 方案外新增的两个参数与两个 launch arg

| 新增 | 用途 |
|---|---|
| `zlio_bridge.warn_throttle_sec`（默认 5.0） | 自实现的日志节流：rclpy 的 `throttle_duration_sec` 在 sim time 下行为不稳，而本仓库其它脚本已有"自己算节流"的先例 |
| `zlio_bridge.epoch_topic`（默认 `zlio_bridge/epoch`） | 方案 §4.1 列的"诊断（可选）"话题，换代时发一次 `UInt64`，便于 P2/P3 用 `ros2 topic echo` 判断有没有悄悄换代 |
| `localization_launch` 的 `zlio_config` | 算法配置文件的绝对路径。不写死，是为了将来 `--gicp`/实车复用同一份 launch 时只换配置 |
| `nav2_stack_launch` 的前端取值校验 | 方案没要求，但"拼错一个字 → 两条条件同时不成立 → `odom→base_link` 静默消失"是这条链最容易踩的坑，值得在 launch 里直接失败 |

---

## 5. 落地时发现、方案里没有写的两件事

### 5.1 ★ `registered_scan` 的点数会显著变少（P3 第一条要量的）

ZLIO 的 `FrontendFrame.cloud` 不是原始扫描，而是**自适应采样后**的点：

```cpp
// frontend/src/odometry.cpp:209
result.cloud = std::move(sampled.output);   // sampling: target minimum_points 1000 / maximum_points 4000
// apps/node.cpp: setOutputResolution(output_resolution) 默认 0.10 m 体素
```

对比 `point_lio` 的 `cloud_registered`：本仓库参数注释里的实测值是 **16,890 点/帧**
（`nav2_params_srm_fakeyaw.yaml` 的 `point_filter_num` 那段）。也就是说 ZLIO 每帧给
`terrain_analysis` 的点可能只有 Point-LIO 的 **1/4 ~ 1/10**，而
`terrain_analysis.scanVoxelSize: 0.05` 并不为此调整。

→ 这可能是"避障看起来没坏但明显变稀"的根因，属于方案 §8.4 的"`registered_scan` 点数/帧"指标。
P3 必须先量这一条，再决定是否要动 `sampling.maximum_points` 或 `output_resolution`。
**本次实施没有改这些值**（方案要求先全部保持默认，改动面越小越不会和上游调参结论打架）。

### 5.2 时间戳语义：`stamp` 是后验时刻，`begin` 才是观测开始

```cpp
message.begin  = result.begin;                    // 观测开始
message.odometry = makeOdometryMessage(result.estimate, ...);  // header.stamp = 后验时刻
message.cloud    = makePointCloudMessage(result.cloud, message.odometry.header, frames_.body); // 同 stamp
```

所以 bridge 直接用 `frame.odometry.header.stamp` 给两个输出打时间戳即可，不需要（也不应该）
拿 `begin` 去对齐——`sensor_scan_generation` 需要的是"这一刻的位姿 + 这一刻的点云"这一对。

### 5.3 ★ `registered_scan` 缺 `intensity` 会把终端刷满（2026-11-05 实跑踩到）

**现象**（用户实跑贴回来的日志）：

```text
[terrainAnalysis-8]    Failed to find match for field 'intensity'.
[terrainAnalysisExt-9] Failed to find match for field 'intensity'.
...（20 Hz × 2 个节点 = 40 行/秒，一直刷）
```

**根因**：ZLIO 的点云只有 `x/y/z` —— `apps/ros.cpp` 的 `makePointCloudMessage()` 用的是
`PointCloud2Modifier::setPointCloud2FieldsByString(1, "xyz")`。而两个地形节点都用
`pcl::PointCloud<pcl::PointXYZI>` 接：

```cpp
pcl::fromROSMsg(*msg, *laserCloud);   // laserCloud 是 PointXYZI
```

PCL 找不到字段映射就**每条消息**打一行告警。

**影响面：只影响日志，不影响算法**。`terrainAnalysis.cpp:170` 拿到点之后立刻把
`point.intensity` 覆盖成扫描时间戳（用它做 `decayTime` 衰减），输入的 intensity 从头到尾没被读过。
但 40 行/秒会把真正的告警（TF 失败、跳变守卫、换代）全淹掉 —— 而这条链上"静默失败"本来就多，
所以必须修。

**修法**：在 `zlio_bridge` 里补一个常量字段（改 ZLIO 上游更贵，而 bridge 本来就是适配层）：

```python
# zlio_bridge.py::_with_intensity()
cloud = point_cloud2.create_cloud(header, [x, y, z, intensity], points)   # 全 float32
```

输出布局因此与 Point-LIO 时代的 `registered_scan` 一致：`x, y, z, intensity`。
`zlio_bridge` 的 `_publish()` 在发点云前统一调用它；包依赖新增 `sensor_msgs_py`。

**验证（A/B 对照，用真实 `terrain_analysis` 节点，不依赖 Gazebo）**：

| 用例 | 喂的是什么 | 结果 |
|---|---|---|
| 负例 | xyz-only 点云（= ZLIO 原始格式），直连 `terrainAnalysis` | **8 帧 → 8 条告警**（每帧一条，复现用户的刷屏） |
| 正例 | 同一份点云经 `zlio_bridge` 转发 | **8 帧全部到达（旁路 best-effort 订阅证实）→ 0 条告警** |

> 也就是说这条告警是"每收到一帧就打印一次"，用行数就能数出收到了几帧 —— 这反倒成了一个
> 很好用的"地形节点到底有没有收到点云"的探针（现在被修掉了，别再用它判连通性）。

**代价**（本机实测，`create_cloud` + `read_points_numpy` 各 50 次取平均）：

| 点数/帧 | 每帧耗时 | 20 Hz 下的单核占用 |
|---|---|---|
| 1200 | 0.067 ms | 0.13% |
| 4000 | 0.091 ms | 0.18% |
| 8000 | 0.096 ms | 0.19% |

顺带确认用户那次运行里另外两个数是对的：
`已转发 200 帧 (丢弃 1)`（丢 1 帧是首帧等 TF，符合方案 §4.5）与
`odom←lidar t=(-0.032, 0.187, 0.456)`（云台安装外参：雷达在 base_link 上方 45.6 cm，量级合理）。

### 5.4 ★ 撞墙之后位姿飞掉：为什么会飞、为什么会一直飞

**现象**（2026-11-05 第一轮实跑，`zlio_scope` 图）：撞击后 `odom→base_link` 的位置一路涨到
几千米、累计位移 ≈ 真值的 **136×**；而**航向一直贴着真值**（误差 ±2° 带内，撞击后才有 ±20° 抖动）。

这个"航向好、位置崩"的组合本身就是诊断信息：它不是"整体发散"，而是**平移不可观**。

#### 为什么会飞（四层叠加）

| # | 环节 | 证据 | 为什么这一层没有兜住 |
|---|---|---|---|
| ① | 撞击把巨大冲量灌进 IMU | ⑤ 面板 roll/pitch 在 ~50 s 从 0 跳到 ±2~4° | **ZLIO 运行时没有任何加速度饱和检查**（全文搜 `satur`/`clip` 为空）；Point-LIO 有 `check_satu: True` + `satu_acc: 30.0`。Gazebo 里 0.2 m/s 撞停在 1~2 个物理步内完成，等效加速度比 3 g 高一个量级，直接积分成几十 m/s 的速度 |
| ② | 撞击后的点云匹配救不回来 | ⑦ 面板 matches/点数在这一段塌下去；`sensor.minimum_range: 0.5` 会把贴墙的回波整段丢掉 | 正对一面大墙时，**法向可观、切向不可观**：沿墙滑几米，点面残差几乎不变。ZLIO 唯一的门是 `filter.cpp:112` 的 `observation.count == 0`（一个面都没匹配到才跳过），**没有卡方/新息门限、没有离群点剔除**，所以"高残差解"会被照单全收 |
| ③ | 坏帧本身还带畸变 | `window.maximum_interval: 0.12 s` 的观测区间可能横跨撞击瞬间 | 去畸变用的是"本来就错"的轨迹 → 一帧点云内部互相矛盾 → 关联/拟面更差（自激） |
| ④ | 云台在转（1 rad/s）会放大这一切 | ① 面板绿色"原始"曲线稳定爬升到 ~2500° | 那是**云台带着雷达转**（1 rad/s ≈ 57°/s），不是发散。它本身被 `sensor_scan_generation` 的实时 TF 正确补偿（所以红线车体 yaw 贴着真值），但撞击会让云台关节受外部力矩、偏离指令，这一小段时间 `lidar→base` 与点云时刻更难对齐 |

#### 为什么会**一直**飞（没有任何恢复路径）

* `sensor_scan_generation` 的 `max_position_jump: 0.75` / `max_yaw_jump: 1.20` **只把差分速度清零**，
  位姿原样发出去（`sensor_scan_generation.cpp:283-320`）→ TF、`terrain_map`、代价地图全都吃这个错位。
* ZLIO 不会因为发散而重新初始化（`initialized_` 恒为 true）；唯一的重置触发是**输入中断换代**。
* `zlio_bridge` 的换代锚定是"保连续"设计：如果换代发生在**已经飞掉**的位姿上，它会把错误一起锚住
  —— 这是方案 §4.3 的固有取舍（`T_off ← T_off · R_last_old · R_first_new⁻¹`）。
* `reset_mode: warm` 换代时保留速度/零偏 → 被撞击污染的速度会带进新代次。

#### 这一轮顺带修掉的两个"看不见"

1. **示波 ⑧ 面板原来是错的（我的 bug）**：它拿"车体位姿 `/odometry`"与"雷达原始位姿
   `zlio/lidar_odometry`"配对算 `T_off`，两者之间还差一个**随云台转动**的 `T_base←lidar`，
   于是每转一圈画一个锯齿 —— 看着像 `T_off` 在崩，其实是配错了对象。
   已改成用 bridge 的 `lidar_odometry`（修正后的**雷达**位姿）与原始位姿按 `stamp` 配对。
   合成数据验证（原始位姿 1 rad/s 自转时）：`T_off` 量到 `yaw +63.00° ± 0.000`,
   `|t| 60.0 ± 0.0 mm` —— 完全恒定，说明配对对了。
2. **发散现在是可观测的**（`zlio_bridge` 新增，参数见 `nav2_params_srm_fakeyaw.yaml`）：

   | 观测 | 手段 |
   |---|---|
   | `zlio_bridge/matches`（`std_msgs/UInt32`，每帧匹配面观测点数） | 掉到 `min_matches_warn`（默认 30）以下打 WARN；示波 ⑦ 面板画成曲线 |
   | 同一代内**单帧跳变** > `max_frame_jump_pos`（1.0 m，20 Hz = 20 m/s）/ `max_frame_jump_yaw`（0.5 rad = 10 rad/s） | 打 **ERROR** 并计数；`reject_implausible_jump: true` 时直接丢帧（默认关，只报不拦） |
   | ZLIO 本帧没接受点云更新（`lidar_updated=false`） | WARN：位姿在纯 IMU 外推 |
| 以上全部 + 换代/丢帧 | 同时发 `zlio_bridge/events`（`DiagnosticArray`），示波画竖线 |
   | 周期日志 | `已转发 N 帧 (丢弃 M), epoch=…, 匹配点过少 X 帧, 单帧跳变 Y 次` |

   > 设计取向与全仓库一致：**先让失败可见**，再谈要不要拦。默认不拦，是因为"姿态卡住"同样会
   > 让地形图停在错误的位置上，只是换了一种错法。

#### IMU 饱和保护的运行时验证（`imu_saturation_guard` 节点，不需要 Gazebo）

| 输入 | 期望 | 实测 |
|---|---|---|
| `(0.5, 0.2, 9.81)`（正常行驶含重力） | 原样通过 | `|a|=9.825` ✅ |
| `(150, 0, 0)`（撞墙量级） | 削到 30，方向不变 | `(30.00, 0.00, 0.00)` ✅ |
| `(60, 80, 0)`（斜向超限） | 削到 30，比例不变 | `|a|=30.000`，`x/y=0.750`（= 3:4）✅ |
| 时间戳 / `frame_id` | 原样透传且严格递增 | 13/13 ✅ |

#### 其余的缓解手段（按代价排序，尚未实施）

| 手段 | 做法 | 代价/风险 |
|---|---|---|
| **IMU 饱和保护** ✅ **已实施**（2026-11-05，2026-11-06 改策略） | `scripts/imu_saturation_guard.py`：`livox/imu` → `livox/imu_saturated`。`|a|` 超 30 m/s²（= Point-LIO 的 `satu_acc`）时，**该帧加速度替换成上一次未饱和的值**（不是按比例缩到 30，理由见下）；`zlio` 的 `imu_topic` 已改订过滤后的话题，两者**同条件起停**。参数在 `imu_saturation_guard:` 段 | 注入误差只剩真实碰撞的速度变化（0.2 m/s 撞停 / 实车 1~2 m/s），激光能修回来；正常行驶（< 5 m/s²）完全不受影响。对照实验：`max_acceleration` 调大（如 1e6）= 关掉保护 |
| 换代改为冷启动 | `zlio_bridge` 的 launch 参数 `reset_mode: cold` | 换代时丢掉被污染的速度/零偏；但位置错位仍由锚定保留 |
| 撞击后就地重启定位 | `kill_nav.sh` 后只重起导航标签页（Gazebo 不用重起） | 人工介入 |
| 临时换回 Point-LIO | `--lio-frontend point_lio` | 它有 `check_satu`/`satu_acc`，抗撞击好一些；但就不是在验 ZLIO 了 |
| 隔离变量 | 用更慢的速度跑同一条路线、或降低接触刚度 | 只是为了确认"是撞击导致的"，不能当结论 |

### 5.5 ★ `zlio_bridge` 少乘了一个逆：整个 odom 系被转了约 30°（迁移引入的 bug）

**现象**：`registered_scan` 与静态地图的平面歪开约 30°（雷达安装 roll 的度数）；开跑头 10 s
航向有 2~3.5° 的**恒定**偏差；odom 的 xy 被投影压缩，表现出方向相关的尺度误差。

**根因**（本仓库方案 §4.2 初版的公式就是错的，实施时照抄了）：

```text
初版： T_off = TF(base←lidar)(t0)            # 隐含假设 ZLIO 首帧位姿 R_0 ≈ 单位阵
正确： T_off = TF(base←lidar)(t0) · R_0⁻¹
```

推导只用到契约 C3：**首帧的 odom→base_link 必须是单位阵**（odom 原点/轴向 = 出生时的
`base_link`），而首帧 `T_odom←lidar(0) = T_off · R_0` 必须等于安装外参 `M`，故
`T_off = M · R_0⁻¹`。

* **Point-LIO 满足** `R_0 ≈ I`（它的世界系按定义就是首帧机体系），所以 `loam_interface`
  那样写没问题 —— 方案抄的就是这个前提；
* **ZLIO 不满足**：`src/frontend/src/initialization.cpp:177` 的 `alignGravity()` 会把世界系
  旋转到重力朝 -Z（`state.rotation = rotation * state.rotation` / `alignGravity(states_)`
  在 350、410 行调用），于是 `R_0` **就是那次扶正旋转**。仿真雷达装在 30° roll 的云台上，
  `R_0` 因此带 30°。

**后果**：`T_off · R_0` 中安装 roll 被算了两次 → odom 系相对真实底盘姿态被绕一根**水平轴**转了
`E = M·R_0·M⁻¹`（共轭，角度 = |R_0|）。纯 roll 30° 的首帧位姿对应**正好 30°、水平轴**；
一般情形（roll 30° + 出生航向）对应 |R_0| ≈ 38.9°（实测 38.87°，与理论一致）。

**为什么 ⑤ 面板看不出来**：`⑤` 画的是 `/odometry`，而 `sensor_scan_generation.cpp:171` 的
`planarize()` 已经把 roll/pitch 清零了 —— 倾斜被藏起来了。这也解释了为什么这个 bug 能活过
第一轮验收。

**修法**（与 `_anchor()` 的换代锚定同一形式，那里本来就是相对量、一直是对的）：

```python
# zlio_bridge.py::_on_frame 首帧
self.mount = mount                      # T_base←lidar(t0)，纯安装外参
self.t_off  = mount * raw.inverse()     # ★ 抵消 ZLIO 世界系的 R_0
```

顺带两处必须跟着改：
* `planarize_pose` 分支不能用 `t_off` 当安装外参了（它已含 `R_0⁻¹`），改用 `self.mount`；
* 首帧日志把 `R_0`（位置/相对重力倾角）与安装外参分开打印，以后一眼能看出抵消对不对。

**验证**（都是不依赖 Gazebo 的）：

| 用例 | 结果 |
|---|---|
| 纯数学回归 `test/test_zlio_bridge_math.py`（5 项，已进 `colcon test`） | 首帧 base 位姿 = 单位阵；旧写法残留**正好 30.0°** 且轴水平；一般情形残留 = \|R_0\| = 38.9°；相对运动不受影响；`R_0 ≠ I` 时换代锚定仍连续 |
| 端到端（起真 bridge，首帧喂 `R_0` = roll 30° + yaw 25°，安装外参 roll 30° + yaw 63°） | 首帧输出姿态与 `M` **偏差 0.000°**、位置偏差 0；对照：旧写法偏 **38.87°** |
| 日志 | 打印 `首帧 ZLIO 位姿 R_0: 位置 (...), 相对重力的倾角 xx.xx° …已抵消` |

**留给下一轮的判定指标**（示波已加，见 §6.4）：`⑤` 面板新增 `ZLIO 原始 roll/pitch` 与
`bridge roll/pitch` 两条线，统计里也打印 `姿态(raw/bridge)`。
**正确时两者都应 ≈ 雷达安装倾角（仿真 30° roll）**；若 bridge 比 raw 多出约 30°，
就是又回到这个 bug。

**这次没解释的部分**：倾斜最多解释 ±4° 量级的航向误差与方向相关的尺度误差；2100/2102
后半段 -10°~-30° 的航向误差、-6 m/s 的速度、matches 下降，是另一个问题（疑似高角速度下
ZLIO 发散 / 陀螺饱和 / 去畸变），必须等倾斜修掉、基线干净之后再单独看。

### 5.5b IMU 饱和处理改成"用上一帧未饱和值替换"（2026-11-06）

最初的实现是"按比例缩到 30 m/s²"。**改掉**，理由是它把真实碰撞的冲量**摊平成一个持续的高
加速度**：滤波器照样把这段积分进速度里（只是慢一点），而且缩过的值在物理上根本不存在。

现在的做法：饱和帧的 `linear_acceleration` **替换成上一次未饱和的采样值**
（stamp/角速度/朝向/协方差全部原样透传）。语义是"这一帧加速度计不可信，当它没发生" ——
滤波器按"没有冲击"继续外推，注入的误差只剩**真实碰撞的速度变化**（0.2 m/s 撞停；
实车 1~2 m/s），这个量级激光匹配完全能拉回来。这也是 Point-LIO `check_satu` 的思路：
饱和的那一帧**不信任**，而不是给它编一个折中值。

两个必须显式处理的边界：

| 边界 | 行为 |
|---|---|
| **首帧就饱和**（还没有未饱和参考值） | 原样透传 + 计数（不猜）；日志 `还没有可替换的参考值` |
| **连续饱和 > `max_hold_sec`**（默认 0.5 s，远超任何碰撞） | 放弃替换、恢复原样透传 + 告警：这么久说明是传感器/链路异常，不能靠"假装没加速度"掩盖 |

被替换过的帧**不会**污染参考值；一帧未饱和就更新参考并清掉"超时"状态。
告警按事件类型分别节流（`replaced` / `no_reference` / `hold_expired`）—— 否则"替换"的告警
会占满节流窗口，把最该被看见的"放弃替换"吞掉（实测踩到）。

**验证**（`.sim_run/test_imu_guard.py`，起真实节点，11 项全过）：

| 用例 | 结果 |
|---|---|
| 正常帧原样通过并成为参考 | `(0.5,0.2,9.81)` ✅ |
| 饱和帧（150 m/s²）→ 参考值 | `(0.40,0.30,9.80)` ✅（且明确不是 30.0）|
| 连续 3 帧饱和 → 同一参考值 | ✅（被替换的帧不污染参考）|
| 恢复帧更新参考 → 后续饱和帧用新参考 | ✅ |
| 持续饱和 0.63 s > `max_hold_sec` | 恢复原样透传 `x=300.0` + 告警 ✅ |
| 首帧饱和（无参考） | 原样透传 `180.0` ✅ |
| stamp / frame_id / 角速度 | 原样透传且严格递增 ✅ |

---

### 5.6 `scan_period` 改回 0（与仓库其它三份参数文件对齐）

**它是什么**：`ign_sim_pointcloud_tool` 用它给每个点**编一个逐点时间戳**
（`point_cloud_converter.cpp:94`）：

```cpp
time = (point_id % horizon_scan) * scan_period / horizon_scan;   // 秒，相对 header.stamp
```

含义是"这一帧是从 `scan_period` 秒里扫出来的"，按**列号**线性分摊时间（0 → 0.05 s）。
下游只有去畸变用它（ZLIO：`point_time_field: time` / `point_time_scale: 1.0` /
`point_time_absolute: false` → 采集时刻 = `stamp + time`）。

**为什么改成 0**：

1. Gazebo 的 `gpu_lidar` 是**瞬时快照**（一次 update 把所有 600×32 条射线打完），物理上一致的
   模型就是"所有点同一时刻"，即 0；
2. 仓库里 `nav2_params.yaml` / `nav2_params_srm.yaml` / `nav2_params_srm_minco.yaml` **本来就是
   0.0**（注释：*GPU雷达整帧同时采样，逐点时间偏移为0*），只有 fakeyaw 这份是 0.05 —— 那是
   2026-11-05 为 **Point-LIO** 的航向误差标出来的经验值，对 ZLIO 未必成立；
3. ZLIO 本来就有自己的去畸变（逐点时间 + 轨迹），先给它一条"没有人工扫掠"的干净基线，
   再决定要不要把 0.05 加回来。

**改完会发生什么**（都不报错，靠观察）：

| 项 | 变化 |
|---|---|
| `velodyne_points.time` | 全为 **0** → ZLIO 的去畸变退化为 no-op |
| ZLIO 的观测结束时刻 | `Input::addScan` 用 `max(point.time)`，从 `stamp+0.05` 变成 `stamp` → 每帧**提前 50 ms** 收口 |
| `registered_scan` / `lidar_odometry` 的 `stamp` | 因此变成云的时间戳本身（bridge 原样用它），**与改之前相差 50 ms** |
| 输入校验 | 无影响：`acquisitionTime()` 只拒绝非有限/越界值，全零合法 |

**怎么确认生效**（不用等出图）：录一帧 `velodyne_points`，看 `time` 字段的 min/max ——
期望 `0.000000 / 0.000000`（改回 0.05 则是 `0 / 0.04998`）。

**A/B 怎么读**：同一段自转各跑一遍 0 与 0.05，看示波 ①（航向误差）与 ⑥（位移夸大倍数）。
判据还是那句："补少了平移崩、补多了航向崩"；0 与 0.05 谁更好要数据说话。

**回退**：把 `scan_period` 改回 `0.05`（必须 = `1/lidar.update_rate`，几何 YAML 里是 20 Hz）。

### 5.7 `body_from_lidar.translation` 改为 `[0, 0, 0.03]`（回归真实几何）

方案 §5.3 本来就把这一步排在"验证通过后再试"；2026-11-06 按需求提前做了。

**依据**（`srm27_robot_description/model_builder.py` 生成的 SDF，
`<link name="front_mid360">` 内）：

| 传感器 | 位姿（相对 link） | 说明 |
|---|---|---|
| `<sensor name="front_mid360_imu" type="imu">` | **无 `<pose>`** → 在 link 原点 | 这条链用的就是它（`ros_gz_bridge.yaml` 把 `livox/imu` 映到 `link/front_mid360/sensor/front_mid360_imu/imu`） |
| `<sensor name="front_mid360_lidar" type="gpu_lidar">` | `<pose>0 0 0.03 0 0 0</pose>` | 雷达传感器在 link 系里 **+3 cm** |

`body_from_lidar` 的语义是"把雷达坐标转换到 IMU/机体系"，`translation` = **雷达原点在 IMU 系中的
坐标** → 正确的是 `(0, 0, +0.03)`。gz 的点云数据产生在**传感器系**，而 `<ignition_frame_id>` 两个
传感器都写成了 link 名（`front_mid360`），所以这 3 cm 必须在配置里补。

**改完影响什么**：

| 项 | 影响 |
|---|---|
| 点云 | ZLIO 把点从传感器系搬到机体系（+3 cm），`registered_scan` 随之整体平移 3 cm |
| 位姿语义 | ZLIO 的 body 现在**确实等于** link 原点（IMU），与 bridge 的 `lidar_frame: front_mid360`、与 TF 完全对齐 |
| 旧值（0）的隐性错误 | ZLIO 那时把"雷达传感器系"当成 body，而 bridge 仍把它标成 `front_mid360`（link 原点）→ 整条位姿链里混进一个**恒定 3 cm 偏置**（方向由 30° roll + 63° yaw 的安装角决定，换算到车体系主要是横向/竖直） |
| `T_off` / 桥接 / TF | **不受影响**：`T_off` 只来自 TF(base←front_mid360) |

**⚠ A/B 注意**：`point_lio.extrinsic_T` 写的是 `[0, 0, 0]`（当年忽略这 3 cm），所以这一项改了之后
**两个前端不再严格等价** —— ZLIO 这边更接近真实几何。要严格 A/B 就临时把这里改回 `[0, 0, 0]`。

**怎么判断值不值得**：看 `terrain_map` 的"地面厚度/平整度"与静态地图的贴合度，以及示波 ② 的航向
误差是否有系统性改善（3 cm 的偏置主要是平移项，对 yaw 影响很小，不要指望它解决大漂移）。

### 5.8 `fake_vel_transform` 的 yaw 外推：先修墙钟，再改成积分

这一条**不属于 ZLIO 迁移**（`fake_vel_transform` 与前端是谁无关），但它属于同一条
"定位 → fakeyaw" 链路，且会直接污染示波上的航向读数，所以记在这里。

#### 问题1：外推不能用墙钟

仿真比实时慢（实测 RTF ≈ **0.77**：指令 114.6 °/s 的底盘自转，用墙钟量到 86 °/s；
云台 -57.3 °/s 量到 -44.7 °/s）。而 `wz` 的单位是"弧度/**仿真**秒"：

* 拿墙钟年龄 × 仿真单位的 wz → 外推量**多算 1/0.77 ≈ 30%**；
* 拿墙钟年龄去对 `odom.header.stamp`（仿真时间）→ 两个时间轴比大小，没有意义。

修法（`fake_vel_transform.cpp`）：

| 用途 | 时钟 | 理由 |
|---|---|---|
| `odom_timeout` 判死 | **墙钟**（`steadyNow()`，保持不变） | 仿真暂停时墙钟照走、仿真时间停住；用仿真时间判超时会永远判不出来 |
| yaw 外推 | **节点时钟 `this->now()` + `odom.header.stamp`** | 两者同源（`use_sim_time: True`） |

诊断里同时给出 `odom_age`（墙钟）与 `odom_age_sim`（仿真），两者之比就是 RTF。

#### 问题2：外推要从"当前 wz × 年龄"改成积分

底盘收到指令要过 `d` 才执行，这条 vx/vy 指令真正起作用是 `now + d`：

```text
yaw(now + d) = yaw_odom(stamp) + ∫[stamp − d → now] wz_cmd(s) ds
```

* `wz` 取 **mux 的输出 `cmd_vel_sim.angular.z`**（新参数 `chassis_cmd_vel_topic`）：
  mux 是底盘前最后一道关口，会钳位（`wz_max`）、切输入源、超时清零；
* 缓存最近约 1 s 的 `(仿真时刻, wz)`（`wz_history_window`），按**分段常值**积分；
  这样自转起停的加减速段（MINCO `max_angular_accel: 15`，0→2 rad/s 约 0.13 s）
  已经如实地包含在指令序列里；
* `d` = 新参数 `actuation_delay`，仿真里先给 **0.02 s**，待标定。

**踩到的一个反直觉坑**：缓存修剪如果只按 `now − window`，一旦 odom 停止刷新
（积分左端 `stamp−d` 冻结、`now` 继续前进），窗口会把早先那段 wz 挤掉，积分**倒着走**。
现在修剪会额外保证覆盖 `last_odom_stamp − d`。

**实车默认关闭**：`chassis_cmd_vel_topic: ""` = 不订阅、不外推（等价旧行为），
因为串口下发延迟还没标定；实车两份参数文件已经按这个写。

#### 示波怎么用（新增）

| 出口 | 内容 |
|---|---|
| `/fake_vel_transform/yaw_debug`（`geometry_msgs/Vector3`，每条指令一次） | x = 预测 yaw，y = odom yaw，z = 积分量（单位 rad） |
| `/diagnostics`（0.5 s 一次） | 新增 `predicted_yaw_deg` / `predicted_yaw` / `wz_integral_deg` / `extrapolation_span` / `actuation_delay` / `wz_history_size` / `chassis_cmd_vel_topic` / `odom_age_sim` |
| 示波工具 | `①` 面板新增"fake_vel_transform 预测 yaw"曲线；统计新增 `外推 yaw: \|pred−GT\| vs \|odom−GT\|` 与 `外推量`，并直接判断"有/没有改善" |

**示波工具本身也改成仿真时间**（`--sim-time auto\|on\|off`，默认检测到 `/clock` 就用仿真时间），
否则延迟类数字全部差 1/RTF 倍。

#### 验证（起真实节点，不需要 Gazebo）

| 用例 | 结果 |
|---|---|
| 稳态（odom 延迟 0.1 s、wz=1 rad/s、d=0.02） | 积分 **0.1201** rad，期望 0.1200 ✅ |
| `predicted = odom + 积分` | 恒等 ✅ |
| **odom 停止刷新 0.4 s** 时 predicted 的前进速率 | **0.988 rad/s** ≈ wz ✅（这就是"补锯齿"） |
| wz 归零 0.3 s 后积分 | 增长 **-0.01°**（不再涨）✅ |
| 输出速度的旋转角 | `vx=0.992802` vs `cos(pred)=0.992803` ✅（不是 odom 的 1.000） |
| 诊断键 | 8 个新键全部就位 ✅ |
| 示波 `--sim-time auto/on/off` | 三档均按预期选时钟 ✅ |

最后一步（**没做**）：用仿真时间同时录 `cmd_vel_sim.angular.z` 与真值 yaw 角速度做互相关，
标定 `d` 与增益 `k = 实际 wz / 指令 wz`。残差大时再考虑在线估计 `k`（限 0.8~1.2）。

---

## 6. 怎么用 / 怎么回退

### 6.1 使用

```bash
# 先构建两个新包（只需一次；没装这两个包时 --lio-frontend zlio 会明确报错）
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-select localization_interfaces frontend

# 全链路：ZLIO 出 odom->base_link
./script/start_sim_nav.sh --lio --lio-frontend zlio -w rmuc_2025 -m rmuc_2025

# 排障用的原始输出（未修正、仅观察）
ros2 topic echo /red_standard_robot1/zlio/lidar_odometry --once
ros2 topic echo /red_standard_robot1/zlio_bridge/epoch      # 换代时才会出现新样本
```

启动横幅会打印前端、`loam_interface` 是否启动、算法配置文件路径，一眼可核对。

### 6.2 回退（三级，与方案 §9 一致）

1. 运行期：`--lio --lio-frontend point_lio`（或干脆不给 `--lio-frontend`）
2. 构建期：不装 `frontend` / `localization_interfaces`，其余照旧（zlio 的启动条件不成立）
3. 代码期：把 `script/start_sim_nav.sh` 的 `LIO_FRONTEND="${LIO_FRONTEND:-point_lio}"` 与
   三个 launch 文件里 `odometry_frontend` 的默认值改回 `point_lio` 即回到迁移前

### 6.3 默认值已经切成 `zlio`（2026-11-05，按需求）

**只切仿真链，不切导航栈/实车入口**，理由是 ZLIO 的算法参数是仿真专用的
（`imu_acceleration_scale: 1.0`、`body_frame: front_mid360`、`lidar_topic: velodyne_points`），
实车不同（方案 §10）。实车入口 `nav_real_launch.py` 目前也不传 `odometry_frontend`，
保持 `point_lio` 缺省就不会被仿真默认值带跑。

| 位置 | 现在的值 | 说明 |
|---|---|---|
| `script/start_sim_nav.sh` | `LIO_FRONTEND="${LIO_FRONTEND:-zlio}"` | 用户实际入口 |
| `launch/nav_srm_simulation_launch.py` | `default_value="zlio"` | 仿真入口（直接 `ros2 launch` 也走 ZLIO） |
| `launch/nav_simulation_launch.py` | 透传 + `default_value="zlio"` | 薄封装入口保持一致 |
| `launch/sim_slam_launch.py` | **显式** `odometry_frontend: "point_lio"` | 建图链路固定 Point-LIO（方案 §10 边界），否则会撞上 `nav2_stack_launch` 的 slam+zlio 校验 |
| `launch/localization_launch.py` / `navigation_launch.py` / `nav2_stack_launch.py` | 仍 `point_lio` | 实车入口共用这三个文件，缺省不动 |

脚本侧的回退是"分级"的，都没有告警噪音：

* 显式 `--lio-frontend zlio` 但没 `--lio`／没构建 `frontend` → **报错退出**（不让人以为在跑 ZLIO）；
* 只是默认值、又没 `--lio`（真值里程计或 `--gicp`）→ 静默退回 `point_lio`；
* 只是默认值、`frontend` 没构建 → **告警 + 自动退回 `point_lio`**（保住方案 §9 的第 2 级回退：
  不构建那两个包，其余照旧还能跑）。

回归测试 `test_lio_frontend_wiring.py::test_frontend_defaults` 把这套默认值契约钉住了
（仿真入口 `zlio`、其余 `point_lio`、`sim_slam_launch` 必须显式钉 `point_lio`）。

---

## 6.4 示波工具：`debug_tools/plot_lio_gt_yaw.py --zlio`

换前端之后，"原来的 5 个面板"看不出差别（它们只比较车体位姿），所以给这个工具加了
`--zlio`：多 3 个面板 + 一组专属统计，正好覆盖方案 §8.3/§8.4 的判据。

| 面板 | 对应方案判据 |
|---|---|
| ⑥ 累计位移（真值 / 导航 / ZLIO 原始）与 **★位移夸大倍数** | §8.4 "位移夸大倍数"（基线 6.5×/8.0×/10.7×）。3~10 倍 = `point_time_scale` 可疑 |
| ⑦ `registered_scan` 点数/帧 + 扫描/桥接频率 | §8.4 "点数/帧" + §8.3 "registered_scan 有输出" |
| ⑧ `T_off(k) = T_导航·T_原始⁻¹` 的 yaw 与 \|平移\| 是否恒定 | §4.2/§4.5 的实现正确性（漂了 = 有人每帧重查 TF） |
| ②⑥⑦⑧ 上的紫色点线 = 换代 | §4.3 换代锚定；epoch 变了但位姿跳 = 锚定失效 |

2026-11-05 实跑之后又加了两样，专治"第几秒开始坏的"和"哪一层放大的"：

| 新增 | 作用 |
|---|---|
| `zlio_bridge/events`（`diagnostic_msgs/DiagnosticArray`） | bridge 把 `few_matches` / `update_rejected` / `frame_jump` / `epoch` / `drop` 的**确切时刻**发出来；示波画成竖线（红=ERROR、橙=WARN、紫=换代），统计里直接打印"第一条 ERROR 在 X s" |
| **分层累计路程** | ⑥ 与统计同时给出 `真值 / 导航 / ZLIO 原始 / bridge` 四条累计路程，并打印 `bridge/GT`（= ZLIO 自己的问题）与 `导航/bridge`（= 云台补偿/帧链路的问题）两个倍数；③ 里也画出 bridge 的雷达轨迹 |

> 为什么需要分层：2026-11-05 第二轮实跑里 `导航 45 m / 真值 10 m / ZLIO 原始 15 m`，
> 放大发生在 raw→nav 之间 —— 但按几何（0.5 m 杠杆臂 × 1 rad/s）最多只能解释几米，
> 所以必须靠下一轮的分层数据把它钉死，而不是靠推理。

实现要点（踩过的坑，改这个工具时别退回去）：

* 原始位姿按 `header.stamp` 与 bridge 输出**配对**再算 `T_off`。取"最新一帧"会把两帧之间的
  运动算进 `T_off`，看起来像"T_off 在漂"，其实是采样对齐问题。
* `--zlio` 时原始话题默认 `zlio/lidar_odometry`；非 zlio 时仍是 `lidar_odometry`。
* `DiagnosticStatus.level` 是 `byte` → Python 侧是单字节 `bytes`，必须转 int（否则工具崩）；
* 录制循环按 `--rate` 节流（原来等于 ~500 Hz 采样，5 s 就 2493 个点，CSV 与 CPU 都浪费）；
* 事件记的是**绝对时钟**、波形记的是**相对秒**，画竖线前必须减去 `t0_abs`；
* `--demo` 分模式：不加 `--zlio` 时合成数据与老版本**完全一致**（GT 静止 + 13 cm 小圆），
  加了才造"位移夸大 3× / 1500 点/帧 / 第 12 s 换代"的数据，用来自检 ⑥⑦⑧。

```bash
./debug_tools/plot_lio_gt_yaw.py --zlio --demo --headless        # 自检工具（不需要 ROS）
./debug_tools/plot_lio_gt_yaw.py --zlio --headless               # 真实录制，Ctrl-C 出图
./debug_tools/plot_lio_gt_yaw.py --zlio --duration 60 --stats-only
```

产物默认写进 `user_test_log/zlio_scope_<时间>.png` + 同名 `.csv`（列里已经带上
`off_yaw/off_t/scan_points/scan_hz/nav_hz/epoch/raw_x/raw_y`，事后可以重新算任何指标）。

---

## 7. 下一步：P2~P4 的具体做法（本次未跑）

未跑的原因：`start_sim_nav.sh` 启动前会 `kill_nav.sh` / `kill_gzb.sh`（清理现有仿真），
且 P2~P4 需要有人盯着 RViz 与真值对照、跑满 60 s 静置与多段直行，属于**有人值守的验收**，
不适合代替用户执行。

| 阶段 | 命令 | 判据（方案 §8.3~§8.5） |
|---|---|---|
| P2 | `./script/start_sim_nav.sh --lio --lio-frontend zlio -w rmuc_2025 -m rmuc_2025` | 起步 `map→base_link` yaw 与真值差 < 3°（`./script/check_yaw_sources.py --duration 15`）；静置 60 s 位移 < 0.1 m；直行 5 m 位移倍数 0.9~1.1；`terrain_map_ext` 有输出且 `obstacle_scan` 正常；无 "multiple parents" / `TF_OLD_DATA`；**`ros2 node list` 里没有 `loam_interface`** |
| P3 | 同一世界/出生点/目标点，分别 `--lio-frontend point_lio` 与 `zlio` | 位移夸大倍数（`user_test_log/lio_*.csv` 累积比；Point-LIO 基线 6.5×/8.0×/10.7×）、`check_yaw_sources.py --spin 2.0` 航向误差、**`registered_scan` 点数/帧**（见 §5.1）、CPU |
| P4 | `./script/start_sim_nav.sh --lio --lio-frontend zlio --yaw-mode spin` | `T_off` 捕获时云台仍锁定 → 起步 yaw 误差与 P2 同档；2 rad/s 自转下航向误差不差于 Point-LIO；云台转起来后 `lidar_odometry` 频率不掉 |

P2 期间建议额外看两个日志：
`zlio_bridge` 的 `已捕获 T(base_link←front_mid360)`（确认抓到的是零位安装外参）与
`ZLIO 换代 ... 已锚定 T_off`（确认没有"跑了很久地形图突然错位"）。

### 7.1 三步跑法（复制粘贴可用）

```
# ── 终端 A：起栈（已默认 ZLIO，不用再给 --lio-frontend）
cd ~/srm_nav_27
./script/start_sim_nav.sh --lio -w rmuc_2025 -m rmuc_2025

# ── 终端 B：示波（先跑一条 90 s 的基线，出图 + CSV）
./debug_tools/plot_lio_gt_yaw.py --zlio --duration 90 --headless
#   产物：user_test_log/zlio_scope_<时间>.png / .csv

# ── 终端 C：制造运动（三种场景各一遍，每遍重跑一次终端 B）
#  ① 静置 60 s          —— 看静态漂移（判据 < 0.1 m）
#  ② 直行 5 m           —— 看位移夸大倍数（判据 0.9~1.1）
#  ③ 自转：--yaw-mode spin（重起终端 A） —— 看云台转动下的航向误差
rviz2   # 或用 ros2 topic pub / action 发目标
```

回退对照（就是方案 §8.4 的 A/B）：把终端 A 的命令加 `--lio-frontend point_lio` 再跑一遍同样的
三段，两张 `zlio_scope_*.png` 放一起比。

### 7.2 我想要的数据（跑完把这些发我）

| # | 内容 | 怎么拿 | 我用它判断什么 |
|---|---|---|---|
| 1 | `user_test_log/zlio_scope_*.png` + 同名 `.csv`（静置／直行／自转 **各一套**，以及 point_lio 的回退对照一套） | 终端 B | 位移夸大倍数、点数/帧、T_off 是否恒定、换代有没有跳变、航向误差 |
| 2 | `zlio` 与 `zlio_bridge` 两个终端的完整日志 | 从启动脚本的标签页拷 | `已捕获 T(base_link←front_mid360): ...`（T_off 数值对不对）、`已转发 N 帧 (丢弃 M)`、`ZLIO 换代 ... 已锚定`、`initialization incomplete`、`会话失败` |
| 3 | 频率与点数三行 | `ros2 topic hz /red_standard_robot1/zlio/frame`、`... /registered_scan`、`... /lidar_odometry` | ZLIO 前端出帧率（判据 ≥15 Hz）、点云是否真的在流、bridge 有没有丢帧 |
| 4 | 地形/避障有没有掉 | `ros2 topic hz /red_standard_robot1/terrain_map_ext`、`ros2 topic hz /red_standard_robot1/obstacle_scan` | §5.1 的点数变少会不会影响到避障 |
| 5 | 只有一份 `odom→base_link` | `ros2 node list | grep -c loam_interface`（应为 0）+ `ros2 run tf2_tools view_frames` | 确认 `loam_interface` 没和 bridge 抢话题；确认 TF 树没有多父 |
| 6 | 机器占用一屏 | `./debug_tools/system_monitor.py --text --once` | ZLIO 的 CPU 代价（和 Point-LIO 的那一屏对比，`定位/里程计` 那一行） |
| 7 | 航向独立校验（可选） | `./script/check_yaw_sources.py --duration 15`、自转时 `--spin 2.0` | 起步 yaw 误差（判据 < 3°），与方案 §8.3 的口径对齐 |

> 1 和 2 是**必需**的；3~7 有就更好。最省事的采集方式：跑完把 `user_test_log/` 里那几个文件
> 直接发我，再贴两个终端日志的尾部 50 行。

---

## 8. 本次用到的验证脚本（一次性，未随包安装）

写在 `.sim_run/`（已被 `.gitignore` 覆盖），需要时可直接复用：

| 脚本 | 作用 |
|---|---|
| `.sim_run/check_launch_wiring.py` | 构造 4 个 launch 文件并对每条 `condition` 求值，断言"哪个节点会被起"（17 项） |
| `.sim_run/test_zlio_bridge.py` | 自己发 TF + `FrontendFrame`，验证 `T_off`、位姿复合、点云变换、换代锚定、丢弃规则、session 失败上报（16 项） |
| `.sim_run/test_imu_guard.py` | 起真实 `imu_saturation_guard` 节点，验证"用上一帧替换"/无参考/超时透传/告警（11 项） |
| `.sim_run/run_check.sh` / `run_bridge_test.sh` / `run_imu_guard_test.sh` | 上面几个的干净环境包装（unset 掉其它工作区的 AMENT/COLCON 前缀，`ROS_DOMAIN_ID=77`，日志写进 `.sim_run/ros_home`） |

长期回归只看包内那个测试：

```bash
colcon test --packages-select srm27_nav_bringup
# 或直接跑：
PYTHONPATH=$PWD/src/srm27_navigation/srm27_nav_bringup/scripts:$PYTHONPATH \
  python3 src/srm27_navigation/srm27_nav_bringup/test/test_lio_frontend_wiring.py
```

---

## 9. 遗留与边界

| 项 | 现状 |
|---|---|
| `--gicp` + ZLIO | 未接入。方案 §10 说 `registered_scan` 天然满足，只需放宽条件；本实施按方案"暂不启用"处理，并在 `nav2_stack_launch` 里把 `zlio`+`slam` 组合显式拒绝 |
| `--slam` + ZLIO | 未接入，且有启动校验拦截（`nav2_slam_launch.py` 不经过 `localization_launch`，起不来 zlio/zlio_bridge） |
| 实车 | 未做。差异只有三处（`imu_acceleration_scale: 9.81`、`body_frame`/`lidar_frame = livox_imu`、`lidar_topic = livox/lidar` + CustomMsg）；`zlio_bridge` 与三条契约可直接复用 |
| z-loc 的 `backend` / map-based | 未引入（需要 GTSAM 与先验地图包，是独立的一次迁移） |
| 默认值切换 | 见 §6.3，等 P4 |
