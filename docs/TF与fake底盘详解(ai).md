# TF 树与 fake 虚拟底盘详解

> [!WARNING]
> **历史资料（旧步兵链路）**：本文描述的 `fake_vel_transform` / `gimbal_yaw_fake` /
> `base_footprint` / 仿真 `MecanumDrive2` 底盘链路属于**旧步兵方案**，已从 SRM 仿真中删除
> （见 [`SRM仿真与导航自转控制实现(ai).md`](./SRM仿真与自转控制实现%28ai%29.md)）。
> SRM 仿真与实车现在统一使用真实随底盘自转的 `base_link`，TF 链为
> `map → odom → base_link → front_mid360 / *_wheel`。阅读本文时请把其中的 frame 名与节点
> 当作旧链路对照，不要照抄到当前配置。

> 简明版在 `调试日志——by maori.md` 的 `2026-10-4` 两节，这里是完整版。
>
> **路径说明**：下面的文件名 + 行号基于 `/home/srm/pb2025_sentry_ws`（那里逐个核对过）。
> `srm_nav_27` 是同一套代码的改名版，逻辑一致、路径和行号有偏移，按符号名搜即可。

| pb2025_sentry_ws | srm_nav_27 |
|---|---|
| `pb2025_sentry_nav/pb2025_nav_bringup/config/reality/srm_nav2_params.yaml` | `srm27_navigation/srm27_nav_bringup/config/real/nav2_params_srm.yaml` |
| `.../config/reality/srm_slam.yaml` | `.../config/real/mapping_params.yaml` |
| `.../launch/srm_robot_state_publisher_launch.py` | `.../launch/real_robot_state_publisher_launch.py` |
| `.../launch/srm_slam_launch.py` | `.../launch/real_mapping_launch.py` |
| `.../config/simulation/nav2_params.yaml` | `.../config/simulation/nav2_params.yaml` |
| `fake_vel_transform` / `sensor_scan_generation` / `loam_interface` / `point_lio` / `small_gicp_relocalization` | `srm27_navigation/` 下同名包 |
| `script/start_real_nav.sh` | 还没搬过来（`srm_nav_27/script/` 目前只有 `start_sim_nav.sh`） |

已核对两边一致的点：`fake_vel_transform.cpp` 的旋转逻辑、`nav2_params_srm.yaml` 里的 fake 段、以及"Nav2 全线用 `base_link`"这件事。所以结论对两套工作空间都成立，只有行号要重新定位。

---

## 1. TF 是什么

TF 是一棵相对位姿的树。每条边 = 一个刚体变换（子坐标系的原点在父坐标系里的位置 + 朝向）。
任意两帧之间的换算 = 把路径上的边依次乘起来。

### 1.1 三个基本 frame

| frame | 含义 | 会不会动 |
|---|---|---|
| `map` | 上一次建图那次会话的栅格图坐标系。以 `srm_site_01` 为例：5cm/px、173×138 像素，覆盖 x∈[-1.99, 6.66]、y∈[-1.43, 5.47] | 死，永远不动 |
| `odom` | 本次上电、Point-LIO 初始化成功那一瞬间车所在的位置和朝向 | LIO 积分出来，平滑连续，但会漂 |
| `base_link` | 底盘坐标原点 | 车一动它就动 |

> `map` 和 `odom` 是**两次不同会话各自的"原点"**。它俩差多少，就是 `map→odom` 那条边要回答的事。

### 1.2 最核心的一个乘积

```text
T(map -> base) = T(map -> odom) ∘ T(odom -> base)
     全局位姿         定位修正          里程计
```

---

## 2. 实车 TF 树（SRM 那套）

```text
map ──┬─ static_transform_publisher_map2odom   (默认 / --lio：写死)
      ├─ slam_toolbox                          (--slam)
      └─ small_gicp_relocalization             (--reloc)
        │
      odom ── sensor_scan_generation ──────→ base_link
                                                │
                                                ├─ livox_frame ── livox_imu   ┐
                                                ├─ livox_scan                 ├ robot_state_publisher
                                                ├─ wheel_1..4                 ┘ (全静态)
                                                └─ base_link_fake             (fake_vel_transform)
```

| 边 | 谁发布 | 依据 | 什么时候 |
|---|---|---|---|
| `map → odom` | `tf2_ros/static_transform_publisher`（节点名 `static_transform_publisher_map2odom`），默认 x=0.10 y=0.00 yaw=4.9393 rad(283°) | `localization_launch.py:167-178`，条件 `not use_pcd_localization`；数值来自 `start_real_nav.sh:102-104` | 默认 / `--lio` |
| `map → odom` | `slam_toolbox`（map / odom / base_link） | `config/reality/srm_slam.yaml` | `--slam` |
| `map → odom` | `small_gicp_relocalization` | `srm_nav2_params.yaml:235-250` | `--reloc` |
| `odom → base_link` | `sensor_scan_generation`：拿 LIO 的雷达位姿 → 乘雷达安装外参换成底盘 → 平面化(z=0, roll=0, pitch=0) → 抖动抑制 | `sensor_scan_generation.cpp:140-146`；参数 `srm_nav2_params.yaml:136-142` | LIO 模式 |
| `odom → livox_imu`（只发话题 `lidar_odometry`，不发 TF） | `loam_interface`，把 Point-LIO 的 `camera_init` 原点平移到 `base_link` | `loam_interface.cpp:115-162` | 同上 |
| `base_link → livox_frame` | `robot_state_publisher`，URDF 由 SRM xacro + `lidar_xyz/lidar_rpy` 现场拼出：xyz 0.15 −0.15 0.22、rpy −4° 0 −90° | `srm_robot_state_publisher_launch.py:98-126`，启动于 `start_real_nav.sh:438` | 静态 `/tf_static` |
| `livox_frame → livox_imu` | 同上，偏移 = −`point_lio.mapping.extrinsic_T` = (0.011, 0.02329, −0.04412) | `srm_nav2_params.yaml:105` | 静态 |
| `base_link → livox_scan` | 同上，偏移 = `lidar_xyz`（二维投影用的水平系） | 同上 | 静态 |
| `base_link → wheel_1..4` | 同上，xacro 里是 **fixed** joint（±0.18, ±0.18, 0.075） | SRM `sentry_robot_cylinder.xacro` | 静态 |
| `base_link → base_link_fake` | `fake_vel_transform`（yaw = −当前底盘朝向） | `fake_vel_transform.cpp:137-147` | 50Hz |

`odom → base_link` 具体在算什么（一步步）：

```
Livox MID360 ──> /livox/lidar + /livox/imu
                      │
              point_lio（激光+惯性里程计）
                      │  aft_mapped_to_init: frame_id=camera_init，pose=雷达在 camera_init 下的 6DoF
                      │  + cloud_registered（配准后点云，同在 camera_init 系）
                      ▼
              loam_interface（只发话题，不发 TF）
                      │  开机查一次 T(base_link ← livox_imu) 锁存
                      │  T(odom←雷达) = T(base_link←雷达) ∘ T(camera_init←雷达)
                      ▼
              sensor_scan_generation
                      │  每帧查 T(雷达 ← base_link)
                      │  T(odom←base) = T(odom←雷达) ∘ T(雷达←base)
                      │  平面化 + 抖动抑制（<2mm 且 <2mrad 就沿用上一帧）
                      ▼
                  TF: odom → base_link   (+ 话题 /odometry)
```

开机瞬间 LIO 位姿≈单位阵，所以 `odom→base_link = 0`：**`odom` 系的原点就是开机时车的位置，x 轴就是开机时车头的朝向**（与雷达怎么装无关，−90° 的安装角在 `T(base_link←雷达)` 里被约掉）。

---

## 3. 仿真 TF 树（上游 pb2025 那套）

| 边 | 谁发布 | 依据 |
|---|---|---|
| `map → odom` | 同实车三选一（static / slam_toolbox / small_gicp） | `localization_launch.py`；`config/simulation/nav2_params.yaml:184-189, 219-221` |
| `odom → base_footprint` | `simulation_ground_truth_odometry.py`，订 `/chassis_odometry_gt`（Gazebo `MecanumDrive2` → `ros_gz_bridge.yaml`） | `rm_navigation_simulation_launch.py:191` |
| `odom → base_footprint` | `sensor_scan_generation`（launch 强制 `publish_tf=true`） | `navigation_launch.py:196-213` |
| `base_footprint → chassis → gimbal_yaw / front_mid360 / front_rplidar_a2 / wheel_*` | `robot_state_publisher`，URDF 由 `simulation_robot.sdf.xmacro` 转出，关节角来自 gz bridge 的 `joint_states` | `spawn_robots.launch.py:85-138` |
| `gimbal_yaw → gimbal_yaw_fake` | `fake_vel_transform`（仿真里 Nav2 的 `robot_base_frame` 就是 `gimbal_yaw_fake`） | `nav2_params.yaml:164-165, 291` |

---

## 4. 坑（踩过的记下来）

1. **`map → odom` 只能有一个发布者**：static / slam_toolbox / small_gicp 三者互斥。`start_real_nav.sh:296-303` 会直接拒绝同时给 `--slam --reloc --lio`。
2. **frame 名必须和车体模型配套**：上游 `nav2_params.yaml` 要 `base_footprint / gimbal_yaw / front_mid360`，SRM 的 `srm_nav2_params.yaml` 要 `base_link / livox_frame / livox_imu / livox_scan`。混用会满屏 TF 报错，而且 GICP 构造时死等 TF、把整个 launch 卡住。`start_real_nav.sh:339` 有自检。
3. **仿真里 `odom → base_footprint` 别双发**：纯真值模式下 `sensor_scan_generation` 的 `publish_tf` 被 launch 强制为 false（`navigation_launch.py:196-213`）。两个广播者抢同一条边 → 车体抖。
4. **Point-LIO 自己不发 TF**：`publish.tf_send_en: False`（`srm_nav2_params.yaml:117`、`srm_slam.yaml:34`），它默认的 `camera_init → aft_mapped` 不参与 TF 树。
5. **实车底盘串口节点不发 TF、也不发里程计**：`srm27_nav_protocol.cpp` 里只有 referee 话题的 publisher 和 `cmd_vel` 的 subscription。所以实车 `odom → base_link` **100% 来自 Point-LIO**。（上游文档说"云台关节 TF 由串口模块提供"，在这台车上不成立。）
6. **`livox_imu` 是 LIO 和 URDF 的接头**：Point-LIO 输出的就是这一系的位姿，URDF 里这条固定边把它换算成 `base_link`。所以 `extrinsic_T`（参数文件）和 `lidar_xyz/lidar_rpy`（launch）必须是同一次标定的结果。错一个，车在地图里就整体斜一个固定角——能跑，但一直蹭墙，很难查。

---

## 5. 雷达挂点决定哪条边是"活的"

| | 雷达在底盘（SRM） | 雷达在 yaw（上游 pb2025 实车模型） |
|---|---|---|
| URDF 里雷达父节点 | `base_link` | `gimbal_yaw` |
| `T(雷达 ← 底盘)` | **静态**，写死在 URDF | **动态**，随云台关节角变 |
| 需要 `/joint_states` 吗 | 不需要（全是 fixed joint） | **必须要**，否则算不出云台角 |

`sensor_scan_generation` 两种都能吃，因为它是**每帧查一次** `T(雷达←底盘)` 再合成（`sensor_scan_generation.cpp:134-138`）：静态车牌下它是常量，动态车牌下它自动把云台旋转折掉。

雷达挂在会转的部件上而**没有关节角**时的症状：

revolute 关节拿不到值 → `robot_state_publisher` 发不出雷达那一支的 TF → `sensor_scan_generation` 查 TF 失败、退回单位阵并打印 `TF lookup failed ... Returning identity`（`sensor_scan_generation.cpp:199-205`）→ **车在地图上原地画圈**。

---

## 6. 验证

```bash
ros2 run tf2_tools view_frames                 # 生成 frames.pdf，一眼看父子关系
ros2 run tf2_ros tf2_echo map odom             # 静态模式：数值应一直不变
ros2 run tf2_ros tf2_echo odom base_link       # 手推车跟着动，z 恒为 0（被平面化了）
ros2 run tf2_ros tf2_echo base_link livox_imu  # 静态外参，应 ≈ -extrinsic_T + 安装角
ros2 topic echo /odometry --once               # sensor_scan_generation 发的那份
```

各条边缺了会怎样：

- 缺 `map→odom` → `global_costmap` 报 `Timed out waiting for transform from base_link to map`，规划器直接失败。
- 缺 `odom→base_link` → 局部代价地图和控制器起不来，RViz 里车永远贴在原点。
- `map→odom` 数值错 → 车照跑，但在地图里整体偏掉，规划出的路径从错误位置出发，典型表现是**直奔墙**。

---

## 7. fake 虚拟底盘（`fake_vel_transform`）

### 7.1 它是干嘛的

上游那台车的云台会自旋扫描，而 Nav2 的机器人参考系挂在云台上，参考系一直在猛转 → 局部规划器误以为"车头方向 = 路径方向" → 车乱动、跟踪不上。

解决办法：造一个**永不旋转的虚拟底盘** `gimbal_yaw_fake` 给 Nav2 用。`fake_vel_transform` 负责三件事：

1. 50Hz 发 TF `robot_base_frame → fake_robot_base_frame`
2. 订 `input_cmd_vel_topic`，把速度从虚拟系转回真实底盘系，发到 `output_cmd_vel_topic`
3. 订 `cmd_spin`，把自旋角速度**加**到输出的 `angular.z`（`fake_vel_transform.cpp:153`）

上游参数（`config/reality/nav2_params.yaml`）：`robot_base_frame: gimbal_yaw`、`fake_robot_base_frame: gimbal_yaw_fake`、`init_spin_speed: 3.14`，且 Nav2 里 `robot_base_frame` 全是 `gimbal_yaw_fake`。

### 7.2 本车现状（SRM，`srm_nav2_params.yaml:197-208`）

```yaml
fake_vel_transform:
  robot_base_frame: "base_link"           # 真实底盘
  fake_robot_base_frame: "base_link_fake" # 虚拟底盘
  input_cmd_vel_topic: "cmd_vel_nav2_result"
  output_cmd_vel_topic: "cmd_vel_chassis"
  init_spin_speed: 0.0                    # 本车没有 cmd_spin 发布者
```

- **自旋叠加 = 0**：上游是 3.14，本车没有 `cmd_spin` 发布者，已改成 0.0。
- **虚拟系没人查**：Nav2 里 `robot_base_frame` 全是 `base_link`（`srm_nav2_params.yaml:351/642/693/788/841`），而上游用的是 `base_link_fake`。所以 `base_link_fake` 这条 TF 发了没人用。
- **它现在真正的作用**：把 `cmd_vel_nav2_result` 转发成 `cmd_vel_chassis`——底盘串口节点订阅的就是这个名字，不经过它车不动。

### 7.3 ⚠ 一处参数不一致（待确认）

节点内部**仍然**按"输入速度是虚拟系的"来旋转（`transformVelocity`，`fake_vel_transform.cpp:149-157`），旋转角取当前底盘在 odom 里的朝向 θ（从 `odom_topic` 的 pose 读）。但 Nav2 现在发的速度是**真实 `base_link` 系**的，不是虚拟系的，两者对不上 → 速度会被多转一次 −θ。

- 车头正对 odom 的 x 轴时 θ≈0，看不出来
- 车转过一个大角度后，理论上"前进"会被转成"横着走"

一句话验证：让导航跑着，把车推到朝 odom 偏 90° 的位置再给前进指令，

```bash
ros2 topic echo /cmd_vel_nav2_result     # Nav2 输出的
ros2 topic echo /cmd_vel_chassis         # 实际发给底盘的
ros2 run tf2_ros tf2_echo odom base_link # 看当前 θ
```

`/cmd_vel_chassis` 的 `linear.y` 明显不为 0 → 就是这个旋转在起作用。

### 7.4 虚拟系的 yaw 应该怎么给

关键一行公式：

```text
fake 系在 odom 里的朝向 = θ(底盘真实朝向) − 你塞进节点里的那个角
```

所以塞什么决定 Nav2 看得见什么：

| 塞什么 | Nav2 看到的效果 | 用途 |
|---|---|---|
| `θ`（现在的代码） | fake 永远朝 odom 正前方 → 车**永远不转**，连正常转弯都看不见 | 上游云台一直转的场景 |
| `0` | fake ≡ `base_link`，什么都没剥掉 | **阶段一：不测陀螺** |
| `s`（累计自旋角） | 看得见导航转弯、看不见自旋 | **阶段二：带陀螺** |

也就是说 yaw 应该给 **`−s`**，其中 `s = ∫ω_自旋 dt`，即下位机转过的角度。

`s` 从哪来（按推荐顺序）：

1. **下位机直接回传累计自旋角**（它自己转的，最准、无积分漂移）
2. 下位机回传 ω_自旋，ROS 积分（会漂，可用 LIO 的总 yaw 做一次重置）
3. 定速自旋：`s = ω0 × (t − t_start)`，零改动但只适用于转速不变
4. 兜底：`ω_自旋 ≈ ω_LIO − ω_自己发的导航 ω`（噪声大，只适合临时验证）

### 7.5 两阶段配置

**阶段一（不测陀螺）**
- 角度源设成 0 → TF 变成恒等（`base_link_fake` 与 `base_link` 重合）
- Nav2 参数不用改，继续用 `base_link`
- ⚠ 前提是先把现在那个 −θ 旋转关掉

**阶段二（带陀螺）**
- 角度源 = `s`（下位机回传）
- Nav2 里 `robot_base_frame` 全部从 `base_link` 改成 `base_link_fake`（照上游配）
- `output_cmd_vel_topic` 继续是 `cmd_vel_chassis`

### 7.6 速度侧

下位机的 `vx` = **电池那条边** = **车体系**。Nav2 在 de-spun 系里出速度，交给底盘前必须转回车体系，转的角度是 `R(−s)`（只需要 `s`，不需要云台角、也不需要世界朝向）。

注意：小陀螺是轮子转整车，所以**世界系里的一条直线**在车体系里是**随时间旋转的向量**。`vx` 恒定不变 = 车在世界里画圈，半径 `R = v/ω`（v=1m/s、ω=3.14rad/s → R=0.32m）。

### 7.7 `angular.z` 只能在一处加自旋

既然自旋由下位机加，节点输出就原样透传（`init_spin_speed: 0.0`、不订 `cmd_spin`）。否则 Nav2 的 ω + 节点的 ω + 下位机的 ω 会叠三次。

### 7.8 验证（自旋时一眼可见）

```bash
ros2 run tf2_ros tf2_echo odom base_link        # yaw 线性增长（真值，含自旋）
ros2 run tf2_ros tf2_echo odom base_link_fake   # yaw 基本不动 ← 这才是对的
```

现在（塞 θ）第二条恒等于 0；改成 `s` 之后它只在导航转弯时才动；阶段一两条完全一致。

### 7.9 待办

- [ ] 跟下位机确认：收到的 `wz` 是"和自旋相加"还是"被自旋覆盖"？覆盖的话 Nav2 完全控不了朝向，得先解决
- [ ] 让下位机回传累计自旋角 `s`（或 ω_自旋）
- [ ] 确认"电池那条边"是否等于 URDF 里 `base_link` 的 +x，不一致会整体拧一个固定角
