# 仿真 `--lio` 链路把 Point-LIO 前端换成 ZLIO 实施方案

> 对应入口：`./script/start_sim_nav.sh --lio`
> 目标：`--lio` 时 `odom → base_link` 由 **ZLIO（z-localization 的 `frontend` 包）** 产生，
> `map → odom` 仍是静态恒等，地形/导航/fakeyaw/速度链路**一行不改**。
>
> 非目标（本文第 10 节给出边界）：`--gicp`、`--slam`、实车链路。
>
> 状态：**已实施**（2026-11-05）。P0/P1 已通过，`--lio-frontend zlio` 可用；
> 默认值仍为 `point_lio`（按 §8.0，P4 通过后再切）。实际改了什么、量到了什么、
> 与本文的偏差，见 [仿真LIO前端替换Z-LIO实施记录(ai).md](../records/仿真LIO前端替换Z-LIO实施记录%28ai%29.md)。
> P2~P4 的验收尚未执行。

---

## 0. 一句话

**只换"谁产生 odom→base_link"，不换"谁把雷达位姿变成车体位姿"。**

本仓库的定位链路其实分四层，`--lio` 只用到前三层：

| # | 职责 | 现 owner | 换后 owner |
|---|---|---|---|
| ① | 激光惯性里程计（`odom ← 雷达/IMU 刚体` 位姿 + 去畸变点云） | `point_lio` | **`zlio`** |
| ② | 坐标系改写：把 LIO 自己的世界系改写成 `odom`（叠 `T_base←lidar(t0)`） | `loam_interface` | **`zlio_bridge`（新增，唯一新代码）** |
| ③ | 雷达位姿 → 车体位姿（查实时 TF 过云台关节）、平面化、抖动抑制、差分出 twist | `sensor_scan_generation` | **不变** |
| ④ | `map → odom` | 静态 TF（`--lio`）/ small_gicp（`--gicp`） | **不变** |

②③ 拆开是本仓库的核心设计，也是这次迁移"看起来很轻"的根因：**ZLIO 只能替 ①，替不了 ③**，
而 ③ 恰恰是云台场景唯一正确的地方（详见 §2.2）。

---

## 1. 现状：`--lio` 链路上每个节点到底在干什么

### 1.1 启动分叉

`nav_srm_simulation_launch.py` 收到 `use_lio_odometry:=True` 后：

```text
use_ground_truth_odometry = not slam and not use_pcd_localization and not use_lio_odometry
                          = False                    # → 真值里程计适配器不起
gimbal_spin_gate           启动（condition=use_lio_odometry）
ign_sim_pointcloud_tool    恒启动（产出 velodyne_points）
nav2_stack_launch          use_lio_odometry 透传下去
```

`nav2_stack_launch.py` → `localization_launch.py`（`not slam` 分支）：

- `start_point_lio_node`，条件 `use_pcd_localization or use_lio_odometry`
- `start_static_transform_node`，条件 `not use_pcd_localization` → `map → odom` 静态恒等
  （`map_to_odom_*` 在本链路恒为 0：`script/start_sim_nav.sh` 只在 `--gicp` 分支里由
  `read_spawn` 推导初值，`--lio` 分支一个都不给；`script/start_sim_nav.sh` 里
  "地图是以出生点为原点建的、LIO 的 odom 原点同样是出生点 → map→odom 必须保持恒等"
  这段注释就是这个约定的出处）

> **这条是本次迁移的坐标系基准，写清楚以便后面所有判断都基于它**：
> 仿真的 `odom` 与 `map` **同源**，原点 = 出生点、轴向 = 出生时的 `base_link`。
> `simulation_ground_truth_odometry.py` 的 docstring 明确写了
> "odom 系是重力对齐的：原点取出生位置，姿态只取出生航向"；
> `--lio` 只是把"谁来产生这个 odom 位姿"从真值换成了 LIO，**原点约定不变**。
> 所以 ZLIO 的 `odom` 必须经过 §4.2 的 `T_off` 修正才能对上这个约定。

`nav2_stack_launch.py` → `navigation_launch.py`：

```python
has_odometry_source = slam or use_pcd_localization or use_lio_odometry   # --lio 下为 True
```

派生三件事：

1. `loam_interface` 启动（`navigation_launch.py:159`）
2. `sensor_scan_generation` 的 `publish_tf` 被 launch 覆盖成 `True`
3. `terrain_analysis` / `terrain_analysis_ext` 的 `registered_scan` 输入
   走 `'sensor_scan' if not has_odometry_source else 'registered_scan'` → **`registered_scan`**

### 1.2 数据流

```text
gz gpu_lidar (20 Hz, 600×32)
   └─ ros_gz_bridge ─► livox/lidar (PointCloud2, 无逐点时间)
          └─ ign_sim_pointcloud_tool ─► velodyne_points (PointXYZIRT, 有 time 字段)
gz imu (200 Hz, m/s²)
   └─ ros_gz_bridge ─► livox/imu ─┐
                                  ▼
                            point_lio ──aft_mapped_to_init──┐
                                └──cloud_registered──────┐  │
                                                          ▼  ▼
                                              loam_interface   ← 叠 T_base←front_mid360(t0)
                                                 │ registered_scan        (odom 系点云)
                                                 │ lidar_odometry         (odom←front_mid360)
                                                 ▼
                                       sensor_scan_generation ← 每帧查 TF front_mid360←base_link
                                                 │ odom→base_link（平面化 + 抖抑制）
                                                 │ /odometry（差分 twist）
                                                 │ sensor_scan
                          ┌──────────────────────┼──────────────────────┐
                          ▼                      ▼                      ▼
                  terrain_analysis       gimbal_spin_gate        fake_vel_transform
                  （吃 registered_scan）   （吃 lidar_odometry）   （吃 /odometry）
```

### 1.3 逐层契约（换前端时必须原样保住的三条）

| 契约 | 内容 | 谁依赖 |
|---|---|---|
| **C1** | `registered_scan`：`sensor_msgs/PointCloud2`，**odom 系**，与 `lidar_odometry` 同一时刻；**字段布局 `x, y, z, intensity`（全 float32）** | `terrain_analysis`、`terrain_analysis_ext`、`sensor_scan_generation`、（`--gicp` 时）`small_gicp` |
| **C2** | `lidar_odometry`：`nav_msgs/Odometry`，`header.frame_id = odom`，pose = `T_odom←front_mid360`，**带 `T_base←lidar(t0)` 静态修正** | `sensor_scan_generation`、`terrain_analysis*`、`gimbal_spin_gate` |
| **C3** | `odom` 帧的原点与轴向 = **出生点处的 `base_link`**（不是雷达系） | `map→odom` 静态恒等这条设定本身 |

> ⚠ C1 的字段要求是实施阶段补上的（2026-11-05）：ZLIO 的点云只有 `x/y/z`，两个地形节点用
> `pcl::PointCloud<pcl::PointXYZI>` 接，字段缺失会**每条消息**刷一行
> `Failed to find match for field 'intensity'`（20 Hz × 2 节点 = 40 行/秒，把真告警淹掉）。
> 所以 `zlio_bridge` 必须补一个常量 `intensity`。详见实施记录 §5.3。
>
> C3 最容易被忽略。`loam_interface` 缓存的是 `T_base_link←lidar`（在首帧时刻查一次 TF），
> 左乘到 LIO 的位姿上，所以 `odom` 的轴向是**底盘**的轴向，不是 63°/30° 倾斜的雷达系。
> 仿真里那次标定（`nav2_params_srm_fakeyaw.yaml:190` 附近注释）量到 `base_link + planarize=False → -0.7°`，
> 就是这条起的作用。**新前端如果不做这个修正，机器人出生就会偏 63°。**

---

## 2. 为什么不能只是"把 `point_lio` 节点换成 `zlio` 节点"

### 2.1 输入形态不同（可适配）

| | `point_lio` | `zlio` |
|---|---|---|
| 点云 | `velodyne_points`（PointXYZIRT，`lidar_type: 2`） | 支持 PointCloud2，但要显式声明逐点时间字段 |
| 点云坐标系 | 世界系（`camera_init`，已配准到世界） | **局部 IMU 系**（去畸变到观测端点） |
| IMU | `livox/imu`，参数 `acc_norm` 声明单位 | 同一话题，但单位由 `imu_acceleration_scale` 声明 |
| 输出 | `aft_mapped_to_init` + `cloud_registered` | `FrontendFrame`（位姿 + 点云打包）+ 三个独立话题 |

输入侧都能配，**输出侧不能直接给 `loam_interface` 用**，见下条。

### 2.2 `loam_interface` 的点云通路假设"点云在世界系"（不可适配，必须新写）

```cpp
// loam_interface.cpp:92-108（节选）
tf2::Transform tf_odom_to_camera_init = tf_odom_to_lidar_odom_;   // 就是静态 T_base←lidar(t0)
pcl_ros::transformPointCloud(odom_frame_, tf_odom_to_camera_init, *msg, *out);
```

它只给点云左乘一个**静态安装变换**，语义是"点云本来在 LIO 世界系，我只要把世界系的朝向改成底盘朝向"。
ZLIO 给的是**局部 IMU 系**点云，必须左乘**当时的位姿**。硬接的后果是点云只被旋转了 30°/63°，
平移完全丢失 → 地形图/避障全错，**而且任何节点都不会报错**。

**结论：②层必须新写一个组件。**

### 2.3 为什么不是"跑 `zlio_visualizer` 然后 remap 一下"

`zlio_visualizer` 确实会发 `/frontend/cloud_registered`（`visualization.cpp:110` 里 `registeredCloud()`
已经把点云转到前端 odom 系）。理论上可以让
`base_frame == lidar_frame` 使 `loam_interface` 的静态变换退化成恒等，再把 `registered_scan` remap 到它。

不采用，三个理由：

1. **坐标系不对**：那是 ZLIO 自己的 odom（轴向 = 63°/30° 倾斜的 IMU 系），不满足契约 C3，
   要么补 `map_to_odom_yaw ≈ -63°`（把 `-59.9°` 那个坑再挖一遍），要么放弃。
2. `zlio_visualizer` 还顺带维护一张 20 万格的可视化地图并周期发布，是不必要的 CPU 开销。
3. 它的 `frontend/frame` 订阅是 `best_effort`，且必须先收到 `frontend/session` 才出图，
   把定位主链路绑在"可视化节点活着"上，是把可选项变成了必选项。

### 2.4 为什么不能保留 `loam_interface` 只改参数

`loam_interface` 的 `state_estimation_topic` 可以 remap 到 ZLIO 的 odom，
但点云那一路（§2.2）没有开关能让它"换成用位姿变换"。除非改它的源码 —— 那就不如把
"改过的 `loam_interface`"直接叫 `zlio_bridge`，语义清楚、回退也干净。

---

## 3. 目标链路

```text
gz gpu_lidar ─► livox/lidar ─► ign_sim_pointcloud_tool ─► velodyne_points ─┐
gz imu ─► livox/imu ───────────────────────────────────────────────────────┤
                                                                            ▼
                                                                          zlio
                                                    ┌──────────────────────┴─────────┐
                                                    │ frontend/frame（位姿 + 局部点云）│
                                                    │ frontend/session（进程/代次）   │
                                                    ▼                               │
                                              zlio_bridge（新增）                    │
                                                    │ registered_scan（odom 系）      │
                                                    │ lidar_odometry（odom←lidar）    │
                                                    ▼                               │
                                          sensor_scan_generation  ◄─────────────────┘
                                                    │（以下与原链路完全一致）
                                                    ...
```

改动只发生在 `zlio` 之后。`loam_interface` 在 `--lio + zlio` 时**不起**。

---

## 4. 新增组件 `zlio_bridge`

位置：`src/srm27_navigation/srm27_nav_bringup/scripts/zlio_bridge.py`
（沿用本仓库既有风格：`gimbal_spin_gate.py` / `simulation_ground_truth_odometry.py` 都是这样装的，
见 `CMakeLists.txt` 的 `install(PROGRAMS ...)`）。

### 4.1 输入 / 输出

| 方向 | 话题（实际名见 §5.5 的 remap 方案） | 类型 | QoS |
|---|---|---|---|
| 订阅 | `frontend/frame` → 实际 `zlio/frame` | `localization_interfaces/msg/FrontendFrame` | reliable（与 zlio 发布端一致） |
| 订阅 | `frontend/session` → 实际 `zlio/session` | `localization_interfaces/msg/FrontendSession` | reliable + transient_local |
| 发布 | `registered_scan` | `sensor_msgs/msg/PointCloud2` | SensorDataQoS（与 `terrain_analysis` 订阅端一致） |
| 发布 | `lidar_odometry` | `nav_msgs/msg/Odometry` | SensorDataQoS（与 `gimbal_spin_gate` 一致） |
| 诊断（可选） | `zlio_bridge/epoch` | `std_msgs/msg/UInt64` | — |

> 两个输出话题**沿用原名字**，是为了让 §6 的改动清单只剩"关掉 `loam_interface`"这一条，
> 下游一个 remap 都不用加。

### 4.2 数学定义

设 ZLIO 给的原始位姿为 `R_k = T_zlodom←body`（`frame.odometry.pose`），
`T_off` 为**首帧时刻**查到的 `T_base_link←body`（含安装外参与当时的云台角）。

```text
T_off   = TF(base_frame ← lidar_frame) @ stamp(first frame) · R_0⁻¹   # 只查一次并缓存
          ↑ 安装外参 M                                    ↑ ★ 必须抵消首帧位姿 R_0
T_odom←lidar(k) = T_off · R_k
cloud_odom      = do_transform_cloud(frame.cloud, T_odom←lidar(k))
lidar_odometry.pose             = T_odom←lidar(k)
lidar_odometry.header.frame_id  = odom_frame
lidar_odometry.child_frame_id   = lidar_frame
lidar_odometry.header.stamp     = frame.odometry.header.stamp     # 与点云同戳
```

> ⚠ **`R_0⁻¹` 不能省**（2026-11-05 实施时修正了本文初版）。契约要求"首帧的 odom 位姿 ==
> 安装外参 M"（odom 原点/轴向 = 出生时的 `base_link`），即 `T_off · R_0 = M`。
> Point-LIO 的世界系按定义就是首帧机体系（`R_0 ≈ I`），所以 `loam_interface` 那样写没问题；
> **ZLIO 不是**：`initialization.cpp` 的 `alignGravity()` 会把世界系扶正到重力朝 -Z
> （`state.rotation = rotation * state.rotation`），于是 `R_0` 就是那次扶正旋转 —— 仿真雷达装了
> 30° roll，`R_0` 就带 30°。少乘逆 = 把安装 roll 算两次，**整个 odom 系被绕水平轴转约 30°**，
> `registered_scan` 与地图平面歪 30°，`terrain_analysis` 把地面当斜面。详见实施记录 §5.5。

`T_off` 只缓存一次（与 `loam_interface` 的 `tf_odom_to_lidar_odom_` 行为一致）。
**不要每帧查 TF**：那等于把云台角重新引进 odom 帧，`sensor_scan_generation` 会再减一次，姿态直接翻倍。

### 4.3 换代（epoch）锚定 —— 本方案里唯一"超出 `loam_interface` 能力"的部分

ZLIO 在输入中断后会**自己换代冷启动**（`coordinated_reset` 无后端时必须为 `false`），
新代次的 odom 原点重置为当时的位姿。若原样转发，`odom` 帧会发生一次等于"已走距离"的跳变：

- `sensor_scan_generation` 有跳变守卫（`max_position_jump: 0.75` / `max_yaw_jump: 1.20`），只清速度；
- **`terrain_analysis` 的 odom 系地形图会错位**，避障静默失效；
- （若将来接 `--gicp`）`small_gicp` 会踩进 `nav2_params_srm.yaml:272` 记过的死锁。

所以在 bridge 里做一次坐标锚定：

```text
epoch 变化时：
    T_off ← T_off · R_last_old · R_first_new⁻¹
```

- `R_last_old` 用变化前最后一帧的原始位姿，`R_first_new` 用变化后第一帧的原始位姿；
- 这个式子是**相对**的、与 `R_0` 无关，所以它本来就是对的（首帧那次抵消用的也是同一形式）；
- ⚠ 初版这里写"冷启动后 `R_first_new ≈ 单位阵`"是**错的**：那是 Point-LIO 的性质。ZLIO 新代次
  同样会 `alignGravity()` 扶正，`R_first_new` 带的是当时的安装倾角（仿真约 30°）。锚定只关心
  "新代次首帧 vs 老代次末帧"的相对关系，所以结论不变，但任何"假设 R ≈ 单位阵"的推导都不成立。

实现上只需要两个变量。**这一步不做，`--lio` 长时间跑一定会出问题**，属于必做项。

### 4.4 参数

| 参数 | 类型 | 建议值（仿真） | 说明 |
|---|---|---|---|
| `odom_frame` | string | `odom` | 输出父帧，必须与 `sensor_scan_generation.odometry_msg.frame_id` 一致 |
| `base_frame` | string | `base_link` | 查 `T_off` 用的车体帧 |
| `lidar_frame` | string | `front_mid360` | 查 `T_off` 用的帧，也是 `child_frame_id`；**必须 = ZLIO 的 `body_frame`**（见下） |
| `frame_topic` | string | `zlio/frame` | |
| `session_topic` | string | `zlio/session` | |
| `planarize_pose` | bool | `false` | 与 `loam_interface.planarize_pose: False` 保持一致；平面化交给 `sensor_scan_generation` 做一次就够 |
| `tf_timeout` | double | `0.5` | `T_off` 查询超时 |
| `anchor_on_epoch_change` | bool | `true` | §4.3 开关，便于 A/B 复现问题 |

⚠ `lidar_frame` 填的是 **ZLIO 的 `body_frame`**，不是雷达驱动的那个 frame：

| | ZLIO `body_frame` | 雷达驱动 frame | 桥接该填 |
|---|---|---|---|
| 仿真 | `front_mid360`（IMU 与雷达同 link） | `front_mid360` | `front_mid360` |
| 实车 | `livox_imu` | `livox_frame` | `livox_imu` |

填成 `livox_frame` 不会报错，只会让 `T_off` 少/多一个 `livox_frame → livox_imu` 的固定外参，
表现为整体位置偏 4.4 cm、姿态恒定偏差 —— 属于"能跑但精度不对"的隐性错误。

`planarize_pose` 保留但**默认关**：仿真 2026-11-04 那次标定明确写了"planarize 会把倾斜的雷达系当底盘压平，
yaw 被拉偏几十度"，所以这一层只能有一个平面化点，就是 `sensor_scan_generation`。

### 4.5 实现要点

- 点云变换直接用 `tf2_sensor_msgs.do_transform_cloud()`（C++ 实现，Python 侧零拷贝开销）。
  `srm27_nav_bringup/package.xml` **已经依赖 `tf2_sensor_msgs`**，不用加新依赖。
- `do_transform_cloud` 之后**必须显式**设置 `out.header.frame_id = odom_frame` 和
  `out.header.stamp`，不要依赖库对 header 的处理。
- `frame.cloud.header.frame_id` 应等于 `frame.odometry.child_frame_id`（ZLIO 的 `body_frame`），
  不相等就整帧丢弃并 `WARN_THROTTLE`。
- `T_off` 还没查到之前丢帧（不能发单位变换，这是本仓库反复强调的原则，
  见 `sensor_scan_generation.cpp` 里 `getTransform()` 的注释）。
- `package.xml` 需要新增 `exec_depend`: `localization_interfaces`、`sensor_msgs`、`std_msgs`（已有）。

---

## 5. 仿真专有对账表（与实车不同的地方，逐项都是坑）

### 5.1 IMU 单位：仿真用 `1.0`，实车才是 `9.81`

| 证据 | 结论 |
|---|---|
| 仿真 `point_lio.mapping.acc_norm: 9.81`（`nav2_params_srm_fakeyaw.yaml:143`） | 仿真 IMU 输出单位是 **m/s²** |
| 仿真 `satu_acc: 30.0`（实车 3.0） | 同上，量级差 9.81 倍 |
| 实车 `point_lio.mapping.acc_norm: 1.0` | 实车 MID-360 输出单位是 **g** |

→ `zlio` 参数 `imu_acceleration_scale` 在仿真必须 **`1.0`**。
⚠️ z-loc 的 `frontend/lio.launch.py` 默认值是 `9.81`（为实车设的），
**不能直接 include 那个 launch**，否则加速度被放大 9.81 倍，滤波器当场发散。

自检方法：静止时 `ros2 topic echo /red_standard_robot1/livox/imu --once` 看 `linear_acceleration` 的模长，
≈ 9.81 说明是 m/s²（`scale=1.0`）；≈ 1.0 说明是 g（`scale=9.81`）。

### 5.2 点云：必须吃 `velodyne_points`，不能吃 `livox/lidar`

| 事实 | 依据 |
|---|---|
| `livox/lidar` 是 gz 直出的 PointCloud2，**没有逐点时间字段** | `ros_gz_bridge.yaml` 里 `ignition.msgs.PointCloudPacked → sensor_msgs/PointCloud2` |
| `velodyne_points` 由 `ign_sim_pointcloud_tool` 生成，字段 `x,y,z,intensity,ring,time` | `src/point_cloud_converter.cpp` 的 `PointXYZIRT` |
| `time = (point_id % horizon_scan) * scan_period / horizon_scan`，单位**秒**，相对 `header.stamp` | `point_cloud_converter.cpp:94`，`scan_period: 0.05` |

→ zlio 参数：

```yaml
input_type: pointcloud2
lidar_topic: velodyne_points
point_time_field: time
point_time_scale: 1.0
point_time_absolute: false
```

注意 `point_time_field` 默认是 `offset_time`、`point_time_scale` 默认 `1e-9`，
**不显式覆盖会静默把时间当纳秒**，去畸变等于没做（正是仿真 `scan_rate` 那一大段注释踩过的坑）。

### 5.3 外参：仿真雷达与 IMU 同 link

| 事实 | 依据 |
|---|---|
| gz 里 IMU 传感器挂在**雷达 link** 上 | `model_builder.py`：`<sensor name="front_mid360_imu">` 在 `lidar['link_name']` 的 `<link>` 内 |
| 雷达传感器相对该 link 的位姿是 `0 0 0.03` | `model_builder.py`：`<pose>0 0 0.03 0 0 0</pose>` |
| 仿真 `point_lio.extrinsic_T: [0,0,0]`（忽略了这 3 cm） | `nav2_params_srm_fakeyaw.yaml:158` |

→ 第一版按**对齐 point_lio** 取 `[0, 0, 0]`，保证 A/B 只差 LIO 本身；
验证通过后再试 `[0, 0, 0.03]`（LiDAR 原点在 IMU 系中的位置），看残差是否更小。

（实车对应值是 `[-0.011, -0.02329, 0.04412]`，与 `extrinsic_T` 同源，别抄错。）

### 5.4 帧名与命名空间

| 项 | 值 | 说明 |
|---|---|---|
| `world_frame` | `odom` | 与 TF 树一致 |
| `body_frame` | **`front_mid360`** | 仿真的 IMU 帧名；实车是 `livox_imu` |
| `publish_tf` | **`false`** | 见下 |
| 全部话题名 | **相对名**（`velodyne_points` / `livox/imu`） | 节点跑在 `red_standard_robot1` 命名空间下 |

`publish_tf` 必须关，两个理由叠加：

1. URDF 里 `front_mid360` 的父节点是 `gimbal_yaw`，`odom → front_mid360` 会让它**有两个父**；
2. `nav2_stack_launch.py` 在 GroupAction 里做了 `SetRemap("/tf", "tf")`，
   ZLIO 内部 `sendTransform` 发的是绝对 `/tf`，在命名空间下会跑到全局去。

`odom → base_link` 继续只由 `sensor_scan_generation` 发布。

### 5.5 话题撞名（必须 remap）

| zlio 默认输出 | 已存在同名者 | 后果 |
|---|---|---|
| `odometry` | `sensor_scan_generation` 的 `/odometry`（odom←base_link + twist） | `fake_vel_transform` 收到两路不同语义的 odom，静默出错 |
| `lidar_odometry` | 本方案里由 `zlio_bridge` 发布（修正后） | 与 bridge 打架 |

→ 启动 zlio 时把它的输出整体 remap 到 `zlio/` 子命名空间：

```text
zlio/odometry           （高频预测，本链路无人消费，仅排障用）
zlio/lidar_odometry     （原始未修正位姿，仅排障用）
zlio/frame              （bridge 的实际输入）
zlio/session
```

`zlio/frame`/`zlio/session` 同时改 bridge 的订阅参数，两边一致即可。

---

## 6. 代码改动清单

> 原则：新增文件为主，改动集中在 `--lio` 相关的 4 个条件判断上，尽量不碰已稳定的导航链路。

### 6.1 新增（不动现有文件）

| 路径 | 内容 |
|---|---|
| `src/frontend/` | 从 `/home/srm/z-localization/src/frontend` 原样拷入 |
| `src/localization_interfaces/` | 同上，`frontend` 的编译依赖 |
| `src/srm27_navigation/srm27_nav_bringup/scripts/zlio_bridge.py` | §4 的桥接节点 |
| `src/srm27_navigation/srm27_nav_bringup/config/simulation/zlio_mid360.yaml` | §7.1 |

**不要拷 z-loc 的 `livox_ros_driver2`**：两边 `msg/` 目录 `diff -r` 为空，
本仓库 v1.1.0 已经满足 `#include <livox_ros_driver2/msg/custom_msg.hpp>`，同名包无法共存。
**不要拷 `backend` / `bringup`**：本方案不需要 GTSAM，也不需要它的地图包。

### 6.2 修改（4 个文件 + 2 个脚本）

| 文件 | 改动 | 说明 |
|---|---|---|
| `srm27_nav_bringup/CMakeLists.txt` | `install(PROGRAMS ...)` 里加 `scripts/zlio_bridge.py` | |
| `srm27_nav_bringup/package.xml` | 加 `exec_depend`: `frontend`、`localization_interfaces` | `tf2_sensor_msgs` 已有 |
| `launch/localization_launch.py` | ① 新增 launch arg `odometry_frontend`（`point_lio`/`zlio`）<br>② `start_point_lio_node` 条件改为 `(use_pcd_localization or use_lio_odometry) and frontend == point_lio`<br>③ 新增 `zlio` Node（condition: `use_lio_odometry and frontend == zlio`）<br>④ 新增 `zlio_bridge` Node（同一条件，`--gicp` 暂不启用） | 静态 `map→odom` 节点**不动**（条件仍是 `not use_pcd_localization`） |
| `launch/nav2_stack_launch.py` | 透传 `odometry_frontend` 给 `localization_launch.py` 和 `navigation_launch.py` | |
| `launch/navigation_launch.py` | ① 新增 `odometry_frontend` launch arg<br>② `loam_interface` 的 condition 从 `IfCondition(has_odometry_source)` 改为 `... and frontend == point_lio`<br>③ composable 分支同样处理（`Node` 与 `LoadComposableNodes` 是两条独立路径，**两处都要改**，见下） | 其余（terrain remap、sensor_scan_generation、fake_vel_transform）**一行不动** |
| `launch/nav_srm_simulation_launch.py` | ① 声明 `odometry_frontend`（默认 `point_lio`）<br>② 透传给 `nav2_stack_launch.py` | `gimbal_spin_gate` 不用改（它订阅的就是 `lidar_odometry`） |
| `script/start_sim_nav.sh` | ① 新增 `--lio-frontend {point_lio\|zlio}`（缺省见 §8.0）<br>② `--lio` 分支把 `odometry_frontend:=...` 拼进 `NAV_EXTRA_ARGS`<br>③ 启动横幅（`printf '定位方式 ...'`）区分两种前端 | |

**注意 `navigation_launch.py` 里 `loam_interface` 有两条独立的启动路径，两处都要加条件**：

| 位置 | 形态 | 现有条件 |
|---|---|---|
| `:157-168` | `Node(...)`（非组合） | `IfCondition(has_odometry_source)` |
| `:417-429` | `LoadComposableNodes`（组合，`plugin="loam_interface::LoamInterfaceNode"`） | `PythonExpression([use_composition, " and (", has_odometry_source, ")"])` |

`use_composition` 两条入口的默认值不同：`script/start_sim_nav.sh` 传 `use_composition:=False`
（`USE_COMPOSITION="${USE_COMPOSITION:-False}"`），而 `nav2_stack_launch.py` 的声明缺省是 `True`
（直接 `ros2 launch` 时生效）。**所以两条路径都可能被走到，必须同时改。**
漏改一条的后果是"`loam_interface` 仍在跑，和 `zlio_bridge` 抢 `registered_scan`"——
两个发布者在同一话题上轮流写，`terrain_analysis` 会吃到跳变的点云，**而且不会报错**。

### 6.3 明确不改的东西

- `sensor_scan_generation`：一行不改。它的 `registered_scan` 输入是同名话题，自动吃到 bridge 的输出；
  雷达系点云分支（`cloud_frame == lidar_frame_`）在本方案里**用不到**（bridge 已经转成 odom 系），
  但保留着不影响。
- `map → odom` 静态发布：不改。`--lio` 的语义就是"odom 原点 = 出生点"。
- `terrain_analysis` / `terrain_analysis_ext` 的 remap：不改（仍是 `registered_scan`）。
- `fake_vel_transform`、`srm_cmd_mux`、速度限幅：不改。
- `ign_sim_pointcloud_tool`：不改，继续产出 `velodyne_points`。
- `gimbal_spin_gate`：不改（`odometry_topic` 默认就是 `lidar_odometry`）。
- `map / spawn / 地图原点`：不改。

---

## 7. 参数文件

### 7.1 新增 `config/simulation/zlio_mid360.yaml`

以 z-loc `config/lio.yaml` 为基线，**只改下面这些**（其余保持默认，改动面越小越不容易和它的严格校验打架）：

```yaml
sensor:
  gravity: 9.81
  minimum_range: 0.5            # = point_lio 的 blind: 0.5，保持 A/B 一致
  maximum_range: 100.0
  body_from_lidar:
    rotation_xyzw: [0, 0, 0, 1]
    translation: [0, 0, 0]      # 第一版对齐 point_lio 的 extrinsic_T；验证后可试 [0,0,0.03]

initialization:
  window_frames: 20             # 20 Hz 下 = 1.0 s 初始化窗口
  seed_imu_samples: 31          # 200 Hz 下 = 0.155 s

window:
  maximum_interval: 0.12        # 必须 > 雷达周期 0.05
  minimum_interval: 0.03
```

其余（`filter` / `map` / `sampling` / `initialization.*`）先全部保持 z-loc 默认值。

**硬性要求**：这份 YAML 必须能被 z-loc 的严格校验接受（未知键会直接启动失败），
建议直接从它仓库的 `config/lio.yaml` 复制后改，不要手写。

### 7.2 `nav2_params_srm_fakeyaw.yaml` 新增段

```yaml
zlio_bridge:
  ros__parameters:
    use_sim_time: True
    odom_frame: "odom"
    base_frame: "base_link"
    lidar_frame: "front_mid360"
    frame_topic: "zlio/frame"
    session_topic: "zlio/session"
    planarize_pose: False
    tf_timeout: 0.5
    anchor_on_epoch_change: True
```

（`point_lio` / `loam_interface` 两段**保留不动**，回退时还要用。）

---

## 8. 分阶段落地与验收判据

### 8.0 默认值策略

| 阶段 | `odometry_frontend` 缺省 | 说明 |
|---|---|---|
| P0~P3 | `point_lio` | 不动既有行为，`--lio` 仍走 Point-LIO；用 `--lio-frontend zlio` 显式试新的 |
| P4 通过后 | `zlio` | 按需求把 `--lio` 的缺省切成 ZLIO；`--lio-frontend point_lio` 作为回退开关保留 |

> **2026-11-05 实际状态**：已按需求把仿真链缺省切成 `zlio`
> （`script/start_sim_nav.sh` 与 `nav_srm_simulation_launch.py`），但 **P2~P4 的验收还没跑过** ——
> 也就是说"默认值已切换"是工程决定，不是"ZLIO 已经证明更好"。导航栈/实车入口的缺省仍是
> `point_lio`（实车 ZLIO 参数不同，见 §10）。细节见实施记录 §6.3。

### 8.1 P0 —— 编译与空转（半天）

1. 拷 `frontend` + `localization_interfaces`，`colcon build --packages-select localization_interfaces frontend`。
   **不需要 GTSAM / TBB / PCL**，只有 Eigen / yaml-cpp / nanoflann / OpenMP（本机都有）。
2. 单独起 `zlio` 看它能不能初始化：
   ```bash
   ros2 run frontend zlio_node --ros-args \
     -p use_sim_time:=true \
     -p config_file:=<abs>/zlio_mid360.yaml \
     -p input_type:=pointcloud2 -p point_time_field:=time -p point_time_scale:=1.0 \
     -p lidar_topic:=/red_standard_robot1/velodyne_points \
     -p imu_topic:=/red_standard_robot1/livox/imu \
     -p imu_acceleration_scale:=1.0 \
     -p publish_tf:=false \
     -p world_frame:=odom -p body_frame:=front_mid360
   ```
3. **判据**
   - 日志出现初始化完成，无 `initialization incomplete`
   - `ros2 topic hz /red_standard_robot1/lidar_odometry` ≥ 15 Hz（20 Hz 雷达，允许少量丢帧）
   - 静止 60 s，`lidar_odometry` 位置漂移 < 0.05 m、yaw 漂移 < 0.5°
   - `ros2 topic echo` 看 `frame_id=odom`、`child_frame_id=front_mid360`

### 8.2 P1 —— bridge 单独验（半天）

只跑 `zlio` + `zlio_bridge`（先不起 nav2）。

**判据**

- `registered_scan` 与 `lidar_odometry` 的频率一致、时间戳一致
- `registered_scan.header.frame_id == odom`
- 用 RViz 叠加：静止时点云不抖；推 1 m，`odom→base_link`（此时还没人发，看 bridge 的 pose）与真值一致
- **轴向自检（关键）**：一个目标点放在车头正前方 3 m，点云里该点的 `x` 应 ≈ 3、`y` ≈ 0。
  若 `y` ≈ ±2.7，说明漏了 `T_off`（C3 契约没满足）

### 8.3 P2 —— 全链路 `--lio`（1 天）

```bash
./script/start_sim_nav.sh --lio --lio-frontend zlio -w rmuc_2025 -m rmuc_2025
```

**判据**

| 项 | 判据 | 工具 |
|---|---|---|
| 起步不偏 | 出生时 `map→base_link` 的 yaw 与真值差 < 3° | `./script/check_yaw_sources.py --duration 15` |
| 静态不漂 | 静置 60 s 位移 < 0.1 m | `terrain_map` 与真值轨迹 |
| 直行不错位 | 直行 5 m，位移倍数 0.9~1.1 | 见 §8.4 |
| 避障不失效 | `terrain_analysis` 有输出、`obstacle_scan` 正常 | `ros2 topic hz .../terrain_map_ext` |
| 无 TF 冲突 | 无 "multiple parents" / `TF_OLD_DATA` 告警 | `ros2 run tf2_tools view_frames` |
| 只有一份 `odom→base_link` | `ros2 node info` 确认 `loam_interface` 未启动 | — |

### 8.4 P3 —— A/B 量化对比（1 天）

同一世界、同一出生点、同一段目标点，分别跑 `--lio-frontend point_lio` 与 `zlio`，对比：

| 指标 | 采集方式 | 当前 Point-LIO 基线 |
|---|---|---|
| 位移夸大倍数 | `user_test_log/lio_*.csv` 里 `lio_x/gt` 累积比 | 仿真历史值 6.5× / 8.0× / 10.7×（见 `nav2_params_srm_fakeyaw.yaml:96-100` 注释） |
| 航向误差（自转时最敏感） | `script/check_yaw_sources.py --spin 2.0` | 参考 `user_test_log/lio_yaw_*.csv` 现有档 |
| `registered_scan` 点数/帧 | `ros2 topic hz` + 一次 `echo` 数点 | `filter_size_map: 0.3` 下的量 |
| CPU | `top -H` / `ros2 run ...` 的耗时日志 | — |

**这条是整个迁移里唯一能证明"值不值得换"的环节，不要跳过。**

### 8.5 P4 —— 云台自转下的稳定性（半天）

```bash
./script/start_sim_nav.sh --lio --lio-frontend zlio --yaw-mode spin
```

**背景**：`start_sim_nav.sh` 在 `--lio` 时把 `GIMBAL_DRIVER` 切成 `external`，
云台由 `gimbal_spin_gate` 在 `lidar_odometry` 稳定 `settle_sec` 之后才开始转
（`gimbal_spin_gate.py` 的 `min_messages: 10` / `settle_sec: 2.0`）。

**判据**

- `T_off` 捕获时云台仍锁定 → 起步 yaw 误差不受自转影响（与 P2 同量级）
- 2 rad/s 自转下 `check_yaw_sources.py` 的误差与 Point-LIO 同档或更好
- 云台开始转之后 `lidar_odometry` 频率不掉

⚠️ ZLIO 初始化不要求静止（这是它的设计目标），所以**不需要**为了它去改 `gimbal_spin_gate`；
但 `T_off` 必须在云台为零位时抓到，而 `gimbal_spin_gate` 恰好保证了这一点 —— 两者互相成全，
**不要把 gate 删掉**。

---

## 9. 风险与回退

| 现象 | 最可能根因 | 处理 |
|---|---|---|
| 起步姿态偏约 63° | `T_off` 没做 / 抓取时刻云台已转 | 检查 `T_off` 日志；确认 `gimbal_driver:=external` |
| 位移速度快 3~10 倍 | `point_time_scale` 错（时间当纳秒）→ 去畸变失效 | 确认 `time` + `1.0` + `absolute: false` |
| 滤波器直接发散 / 加速度爆表 | `imu_acceleration_scale` 用了 `9.81` | 改成 `1.0` |
| `terrain_map` 空 / `obstacle_scan` 无输出 | `registered_scan` 没人发，或 `loam_interface` 与 bridge 同时在发 | 检查 `navigation_launch.py` **两处**分支 |
| tf2 "multiple parents" 告警 | `publish_tf` 没关 | `publish_tf: false` |
| `fake_vel_transform` 报 odom 过期/抖动 | `/odometry` 撞名 | 把 zlio 的 `odometry` remap 到 `zlio/odometry` |
| 跑一段时间后地形图整体错位 | epoch 换代没锚定 | `anchor_on_epoch_change: true` + 查 epoch 日志 |
| `initialization incomplete` | 世界几何不足（如 `-w srm_empty`） | 同 Point-LIO：换有结构的世界（脚本已对 `srm_empty` 拦截） |

**回退方式（三级，随时可用）**

1. 运行期：`--lio --lio-frontend point_lio`
2. 构建期：不装 `frontend` / `localization_interfaces` 两个包，其余照旧（`zlio` 节点条件不成立即可）
3. 代码期：`odometry_frontend` 缺省停留在 `point_lio`（P0~P3 阶段），等价于旧行为

---

## 10. 边界：本方案不覆盖的部分

| 场景 | 现状 | 接入本方案要补什么 |
|---|---|---|
| `--gicp` | `small_gicp` 订阅 `registered_scan`（odom 系） | bridge 输出的 `registered_scan` **天然满足**，只需在 `localization_launch.py` 里把 zlio + bridge 的条件从"仅 `--lio`"放宽到"`use_pcd_localization` 也算"，无需改 bridge |
| `--slam` | `nav2_slam_launch.py` 里直接起 `point_lio`（独立于 `localization_launch.py`），`pointcloud_to_laserscan` 吃 `terrain_map_ext` 再出 `obstacle_scan` | ① 同样加 `odometry_frontend` 分叉；② 该分支下 `terrain_analysis*` 的输入是 `registered_scan`，需确认此时 bridge 已经先起（存在启动顺序依赖）；③ 复核 `slam_toolbox` 的 `throttle_scans` / `minimum_time_interval` 能否接受 ZLIO 的自适应观测率 |
| 实车 | 尚未写方案 | 与本文的差异只有三处：`imu_acceleration_scale: 9.81`（MID-360 输出为 g）、`body_frame` / `lidar_frame` = `livox_imu`、`lidar_topic` = `livox/lidar`（CustomMsg）。§4 的 `zlio_bridge` 与 §1.3 的三条契约完全复用 |
| z-loc 的 map-based backend | 未引入 | 需要 GTSAM 4.2 + 先验地图包，是独立的一次迁移，与本方案不冲突 |

---

## 11. 与 z-loc 上游的对应关系（便于将来跟进）

| 本方案用的 z-loc 能力 | 上游位置 |
|---|---|
| ZLIO 前端节点与参数 | `src/frontend/`（`apps/node.cpp`、`config/lio.yaml`、`docs/parameters.md`） |
| `FrontendFrame` 位姿+点云打包 | `src/localization_interfaces/msg/FrontendFrame.msg` |
| `output_resolution`（输出点云体素，默认 0.10 m） | `apps/node.cpp:105` |
| 换代/会话通知 | `apps/node.cpp` 的 `publishSession()` / `FrontendSession` |
| **不用**的部分 | `src/backend/`、`src/bringup/`、`config/backend.yaml` |

> 上游 `docs/design.md` §1.2 把"前端维护里程计、全局定位另算"作为设计出发点，
> 与本仓库"`sensor_scan_generation` 负责车体位姿、`map→odom` 由第三层给"的分层是**同构**的，
> 这也是这次迁移能只换一层的原因。
