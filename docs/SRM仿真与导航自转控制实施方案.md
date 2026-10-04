# SRM 仿真与导航自转控制实施方案

## 1. 目标与边界

本方案基于当前 Ignition Gazebo Fortress 仿真，重构 SRM 模型和底盘速度控制链路。导航任务只负责平面平移速度 `vx`、`vy`；导航链路始终输出 `wz = 0`。自转由独立的仿真测试接口产生，最后与导航的 `vx`、`vy` 合成为仿真底盘速度，用来测试“导航平移与底盘自转同时发生”的工况。

实施目标包括：

- 使用 SRM 的车体、轮组和雷达安装几何，删除旧步兵的装甲、射击、云台及底盘控制组件；雷达采集、点云处理和定位继续沿用当前实现。
- 继续使用 Gazebo 世界和传感器能力，复用 Nav2、地图、代价地图、规划器和局部控制器；重构机器人描述、底盘速度执行和命令接口。
- 仿真和实车沿用相同的三种 `map→odom` 发布方式，由现有启动模式选择发布者。
- 仿真采用速度级执行，用于观察导航平移叠加底盘自转的效果。
- 本次只考虑单机器人；速度命令统一使用 `geometry_msgs/msg/Twist`。行为树暂不修改，效果由使用者后续观察评估，不预设量化验收门槛。

本次工作集中在导航速度输出、自转测试、SRM 模型和仿真速度执行。雷达获取点云及定位链路维持当前实现，下位机控制、串口协议和实车执行适配不列入实施范围。

## 2. 总体架构

```text
RViz/Nav2 Goal
      │
      ▼
Nav2 controller（只产生 vx、vy；wz 被强制置零）
      │  /<robot_ns>/cmd_vel_nav
      ▼
srm_cmd_mux（导航平移 + 独立自转 + 超时保护）
      │  /<robot_ns>/cmd_vel_sim
      ├──────────────► SRM Gazebo velocity adapter
      │                         │
      │                         ├─ 执行 SRM base 速度
      │                         └─ 输出仿真真值供调试
独立自转测试发送器（根据 launch/YAML 生成恒速或周期速度）
      │ /<robot_ns>/rotation_cmd
      ▼
rotation_controller（独立仿真自转速度输出）
      └─ /<robot_ns>/rotation_velocity → mux 的 wz 输入
```

`srm_cmd_mux` 合成导航平移和仿真自转速度，SRM Gazebo 适配器负责执行。导航节点不直接写入 `wz`，自转测试节点也不修改 Nav2 的控制器状态。Gazebo 时钟桥、雷达采集、点云处理和定位沿用现有启动链路。

## 3. 速度和自转接口

### 3.1 导航速度接口

新增或统一使用：

```text
/<robot_ns>/cmd_vel_nav   geometry_msgs/msg/Twist
```

约定如下：

| 字段 | 含义 |
|---|---|
| `linear.x` | 机器人本体坐标系前向速度 `vx`，m/s |
| `linear.y` | 机器人本体坐标系左向速度 `vy`，m/s |
| `angular.z` | 必须为 `0`，由 mux 再次限幅为 `0` |

Nav2 局部控制器保留全向平移能力，但关闭独立旋转控制：`enable_rotation: false`、`use_rotate_to_heading: false`。mux 只取导航输入的 `linear.x`、`linear.y`，丢弃其 `angular.z`，自转角速度只取独立自转输入。

导航统一使用 SRM 的 `base_link` 作为速度参考系。重构后的 SRM 仿真不再使用 `gimbal_yaw_fake`、`fake_vel_transform` 或官方步兵的云台耦合坐标系。

**只控制平移，不代表忽略底盘 yaw。** 当前 Omni PID 把路径转换到 Costmap 的机器人参考系后生成速度；因此必须让控制器和 Costmap 使用真实、随底盘转动的 `base_link`，使控制器每次计算都使用最新姿态。若未来控制器改为输出 odom 系速度，则由一个明确的适配器完成 `R(-yaw)` 转换，不能再重复转换。

设底盘朝向为 `ψ`，车体速度和 odom 系速度的关系为：

```text
vx_odom = cos(ψ) * vx_body - sin(ψ) * vy_body
vy_odom = sin(ψ) * vx_body + cos(ψ) * vy_body

vx_body =  cos(ψ) * vx_odom + sin(ψ) * vy_odom
vy_body = -sin(ψ) * vx_odom + cos(ψ) * vy_odom
```

“固定车体系 vx + 自转”应产生曲线轨迹；“保持 odom 系方向平移 + 自转”则需要持续更新车体系 vx/vy。两种情况都要测试，不能把前一种曲线轨迹误判为导航坐标变换错误。Nav2 不追踪目标朝向，但 odometry 必须报告实际 yaw 和 wz。

### 3.2 独立自转接口

自转请求与自转输出也统一使用 `Twist`，只使用 `angular.z`，其余分量为零：

```text
/<robot_ns>/rotation_cmd        geometry_msgs/msg/Twist
/<robot_ns>/rotation_velocity   geometry_msgs/msg/Twist
```

恒速、周期、幅度等是测试发送器的 launch/YAML 参数，不塞进 `Twist` 的其他分量，也不再新增 `RotationCommand` 消息：

| 参数 | 含义 |
|---|---|
| `rotation_mode` | `stop`、`constant` 或 `periodic` |
| `angular_speed` | 恒速模式的有符号角速度，rad/s |
| `offset`、`amplitude` | 周期模式的平均角速度和非负变化幅度，rad/s |
| `period`、`phase` | 周期（s）和初相位（rad） |
| `sine_wave` | true 为正弦，false 为方波 |

发送器在每次发布时计算当前角速度：恒速为 `angular_speed`；正弦为 `offset + amplitude * sin(2πt/period + phase)`；方波取相应正负幅度再加 offset。停止模式持续发送零角速度。`offset=0` 可实现周期换向，`offset>amplitude` 可实现同方向周期变速。

波形时间取仿真时间，从启用或波形参数改变时开始计算，周期性发布本身不重置相位。暂停时停止执行；恢复或 reset 后先清空旧命令，由新的启用操作开始输出。拒绝非有限速度、负幅度和周期模式下的非正周期。

rotation controller 只订阅 `rotation_cmd` 并发布 `rotation_velocity`，用于输出仿真自转速度，不修改导航速度。发送器和控制器初始频率均为 200 Hz；自转请求超时默认 0.5 s，由守护线程检测后清零。正常变速可配置角加速度上限，停止命令和超时清零优先。所有速度均约定为 `base_link` 车体系，`Twist` 本身不携带 frame 或时间戳，由接口约定和接收端计时确定。

### 3.3 合成和安全优先级

`srm_cmd_mux` 以固定频率运行，合成规则为：

```text
vx = clamp(nav_vx, -vx_max, vx_max)
vy = clamp(nav_vy, -vy_max, vy_max)
scale = min(1, v_max / hypot(vx, vy))   # 零向量时 scale=1
vx *= scale
vy *= scale
wz = clamp(rotation_wz, -wz_max, wz_max)
```

仿真器收到的最终命令为 `/<robot_ns>/cmd_vel_sim`（`geometry_msgs/msg/Twist`，车体系），mux 是唯一发布者。接收侧由守护线程检测输入中断：导航输入超时清零 `vx/vy`，自转输入超时清零 `wz`；执行端收不到 mux 输出时清零全部速度。队列采用 KEEP_LAST=1。

默认导航超时 0.3 s、自转输出超时 0.1 s、最终执行命令超时 0.1 s，由接收侧记录最近一次收消息的时间，守护线程检测超时并自动清零。超时计时使用单调时钟；周期波形使用仿真时间。初始平移上限沿用 SRM 的 0.5 m/s，自转上限暂设 2 rad/s，可通过配置扩大测试范围。平移按向量模长限幅，避免对角运动超过总速度上限。暂停/reset/退出/急停时清零并清空旧速度缓存。导航与自转输入都拒绝 NaN/Inf。

到达或取消导航目标时清零平移，独立自转按发送器的启用状态继续；另提供 `stop_all` 服务停止两路输入并保持零输出，直到显式重新启用。行为树和手柄接入暂不作为本次工作；当前接入的导航速度和自转速度统一由 mux 合成。

## 4. SRM 自有模型与速度执行

新增独立 ROS 2 包 `srm27_gazebo_simulator`，使用当前 Gazebo Fortress，目录建议为：

```text
srm27_gazebo_simulator/
├── models/srm_sentry/model.sdf   # SRM 机器人模型
├── worlds/*.sdf                 # 世界、材质、灯光和障碍物
├── config/*.yaml                # 传感器、桥接和执行参数
├── launch/srm_sim.launch.py
└── plugins/srm_velocity_system  # SRM 自有底盘速度执行插件
```

独立自转控制器和 mux 放在 `srm27_chassis_control`，仿真执行插件放在仿真包。

模型以本仓库的 `srm27_nav_bringup/urdf/sentry_robot_cylinder.xacro` 为 SRM 几何来源：底盘半径 0.27 m、高 0.2 m；轮半径 0.075 m、宽 0.04 m、中心位置 `(±0.18, ±0.18, 0.075)`。这些是当前简化模型的参数，不视作已经标定的整车动力学参数。清理该 xacro 中残留的 `package://pb_rm_simulation/` 引用，指向本地 SRM 描述包。新增公共 `srm27_robot_description` 管理尺寸、安装位姿、URDF 和 SDF 生成，避免仿真/实车两份模型独立修改。

旧步兵的装甲、射击、云台关节链和底盘控制器不进入新速度执行链路。首版只使用 SRM 圆柱底盘、轮组和雷达；现有雷达采集插件、ROS 点云桥接、点云处理和定位继续复用。

模型重构时，删除 SRM 模型和对应启动配置中的旧装甲、灯条、射击、云台关节及其控制器，包括 `LightBarController`、`ProjectileShooter` 和云台 `JointController`；`MecanumDrive2` 随 SRM 自有速度插件接入一起替换。同步删除引用这些组件的命令桥接和启动项，保留场地资源与通用传感器能力。

首版底盘采用直接速度执行：

1. SRM 执行适配器订阅 `cmd_vel_sim`，将 Twist 校验后转换为 Gazebo Transport 的底盘速度消息。
2. SRM 自有 system plugin 在仿真 PreUpdate 中读取最新 `vx`、`vy`、`wz`，按 Gazebo API 的参考系约定设置底盘平面速度；ROS/Transport 回调只更新输入缓存。
3. 插件只负责速度执行和超时归零，不增加轮速 PID、力矩控制、底盘跟随云台或额外旋转策略。
4. Gazebo 更新模型位置、接触和传感器状态；PostUpdate 读取真实执行后的位姿和速度，输出真值供调试。不能用指令积分的理想轨迹冒充碰撞后的真实运动。
5. 空场单独调试时可用真值发布 odometry/TF；启用现有雷达定位链路后，导航使用其 odometry/TF，仿真真值仅供对照，避免重复发布 `odom→base_link`。

执行插件明确区分车体系与世界系速度，转换只能做一次。底盘平移和 yaw 来自合成指令；z、roll、pitch 根据场景保留物理约束，不给正常运动额外施加持续旋转。轮组碰撞与地面摩擦需要独立验证，避免固定轮子的接触再次使底盘卡住。

模型保留 SRM 和墙体的有效碰撞几何。直接设置速度不等于每一步直接修改位置，不能据此认定一定会穿墙；位置推进和接触处理仍需交给物理引擎。当前 `MecanumDrive2` 的速度分支调用 `SetLinearVelocity` / `SetAngularVelocity`，没有逐步改写底盘位置；仅凭这段代码不能判定碰撞效果，仍需实际观察。先保留直接速度方案，观察撞墙、擦墙、薄墙和隧道场景；发现问题时先检查碰撞几何、物理步长和具体执行 API，再决定是否需要额外碰撞处理。预测碰撞算法不作为首版预设开发项。首版先验证平面场景，坡地、腾空和轮地接触动力学单独扩展。

现有 `MecanumDrive2` 的速度模式只能作为问题定位参考；最终交付为 SRM 自有速度插件，替换旧底盘插件和 `rmoss_gz_base` 底盘控制器。仿真对外始终使用 Twist 速度接口。

## 5. 传感器、时间和 TF

- 仿真继续使用现有 Gazebo 时钟桥发布 `/clock`，仿真节点使用 `use_sim_time=true`。
- 雷达获取点云、IMU、消息转换、点云处理和定位沿用现有链路。现有话题、消息类型、传感器帧名、扫描频率及 Point-LIO/GICP/SLAM 参数保持当前配置。
- SRM 新模型承接现有雷达安装和静态 TF，导航速度参考系使用真实随底盘旋转的 `base_link`；接入时保持雷达与底盘之间的静态变换正确。
- `odom→base_link` 继续由现有里程计及 TF 适配链路提供。空场单独调试的真值输出与导航定位输出分开，接入定位后不重复发布导航 TF。
- 地图和 Nav2 代价地图继续使用现有配置，仿真世界中的障碍物与地图保持对齐。

仿真和实车的 `map→odom` 均沿用以下三种发布方式：

| 当前模式 | `map→odom` 发布者 |
|---|---|
| 仅使用 LIO 里程计 | `tf2_ros/static_transform_publisher`，采用现有静态 TF 配置 |
| SLAM 建图 | `slam_toolbox` |
| 使用 GICP 定位 | `small_gicp_relocalization` |

发布者由现有启动模式选择，同一时刻只启用其中一种。LIO 模式的静态 TF 继续使用现有偏移参数，不新增“仿真默认必须为零”的约定。

速度执行可从 1 ms 仿真步长、200 Hz 命令合成开始调试；点云、IMU 和定位发布频率沿用现有配置。暂停/reset 时清空速度缓存，定位和传感器按现有启动流程处理。

现有场地世界和 mesh 可作为环境素材复用，不携带旧机器人插件。核对场地 mesh 的碰撞精度，必要时给墙、地面、坡道和隧道配置独立、经过验证的碰撞几何。二维地图不包含隧道高度，另从三维场景检查车体高度、雷达遮挡和通行净空。

## 6. Nav2 适配项

1. 新增 SRM 仿真导航 launch，参数保持现有 `nav2_params_srm.yaml` 的职责划分。
2. 仿真参数中的 `robot_base_frame` 统一为 `base_link`，删除 `gimbal_yaw_fake` 和 `fake_vel_transform` 相关配置。
3. 保留平滑器时，明确串联为 `controller → cmd_vel_controller → velocity_smoother → cmd_vel_nav → mux → cmd_vel_sim`；跳过平滑器时由 controller 直接发布 `cmd_vel_nav`。两种配置互斥，速度话题均为 `Twist`。
4. `controller_server` 和 `velocity_smoother` 的输出由 mux 再限幅；行为树及恢复行为暂不修改。
5. 复用现有雷达点云、定位、全局规划、代价地图、障碍物层和 RViz 链路，改动集中在机器人模型、底盘速度输出及仿真执行入口。
6. 提供 `--rotation-mode stop|constant|periodic`、`--rotation-speed`、`--rotation-period` 等 launch 参数，便于固定测试复现。

还需调整以下已有实现：

- 新增 `config/simulation/nav2_params_srm.yaml`，从现有仿真配置复用雷达与定位参数，仅调整 SRM 机器人参考系、平移控制及速度命令出口。只支持单机器人，本文 `robot_ns` 默认取 `srm_sentry`；沿用全局 `/clock` 和现有 TF 发布方式。
- GoalChecker 暂沿用现有 SRM 参数；行为树、Spin/恢复流程暂不列入本次修改。
- 核对 `min_y_velocity_threshold`：现有 SRM 配置为 0.5 m/s，与平移限速相近，会抹掉低速横移反馈，建议设为 0.001 m/s 再验证。
- 观察 velocity_smoother 对自转时平移方向的影响：车体系 vx/vy 会随底盘朝向变化，过强平滑可能产生方向滞后，按导航效果调参。
- 首版用圆形 footprint/真实 SRM 外接包络检查碰撞；外接半径根据完整轮组和传感器尺寸确认，不能直接把圆柱半径视作整车最大边界。自转碰撞按实际姿态/包络检查。
- 更新 SRM 仿真 launch 和脚本，新旧底盘执行器互斥，复用现有 clock/odom/TF 发布者。

### 6.1 文件改动清单

| 位置 | 计划改动 |
|---|---|
| `src/srm27_robot_description/`（新增） | SRM 公共几何/外参、模型生成、独立本地 mesh |
| 自转测试参数与状态 | 波形使用 launch/YAML 参数，速度统一 Twist；状态优先使用标准诊断消息，不为速度新增消息包 |
| `src/srm27_chassis_control/`（新增） | mux、导航角速度过滤、自转命令生成与守护线程超时清零 |
| `src/srm27_gazebo_simulator/`（新增） | SRM SDF、自有速度插件与调试真值输出；复用现有传感器桥接 |
| `srm27_nav_bringup/launch/nav_srm_simulation_launch.py`（新增） | 复用现有点云、定位和 Nav2 启动链路，接入 SRM 模型与速度执行 |
| `srm27_nav_bringup/config/simulation/nav2_params_srm.yaml`（新增） | 复用现有传感器和定位参数，调整 SRM frame、平移控制、自转隔离参数 |
| `srm27_nav_bringup/launch/navigation_launch.py` | 将 fake_vel_transform 变为可选；给命令出口和 mux 留出配置，覆盖 composition 两条启动分支 |
| 旧 SRM SDF 与启动配置 | 删除装甲、灯条、射击、云台及其控制器；替换旧底盘插件与对应桥接 |
| `script/start_sim_nav.sh` | 改为 SRM 仿真入口，保留现有地图与定位模式选择，增加自转参数 |

## 7. 分阶段实施

### Phase 0：接口冻结

- 确认 SRM 的 `base_link` 能接入现有 TF 链，沿用既有传感器帧和三种 `map→odom` 发布方式。
- 合并 `cmd_vel_nav`、`rotation_cmd`、`cmd_vel_sim` 的消息和超时规则。
- 为 Nav2 输出增加 `wz=0` 的单元测试和话题诊断。

观察内容：导航和自转能分别输入、分别停止；导航输入中的角速度不会混入最终自转输出。

### Phase 1：SRM Gazebo 空场速度仿真

- 导入 SRM 圆柱底盘、轮组、雷达和碰撞几何。
- 复用 `/clock`，实现 `cmd_vel_sim` 速度执行及空场调试用的真值输出。
- 用固定命令验证四种基本运动：纯 `vx`、纯 `vy`、`vx+vy`、纯 `wz`。

观察内容：直行、横移、斜向移动、自转的方向与速度符合输入；停止后不继续运动；观察撞墙和擦墙效果。具体效果由使用者后续判断，不预设误差阈值。

### Phase 2：独立自转控制

- 实现恒速、方波周期、正弦周期三种模式。
- 在自转期间叠加固定 `vx`、`vy`，记录车体轨迹、yaw、命令和实际速度。
- 接入守护线程超时清零，以及暂停/reset 和重复启动清零逻辑。

观察内容：恒速、周期换向和周期变速能运行；叠加平移后运动符合车体系约定；发送程序停止、mux 退出和仿真暂停/reset 后不会沿用旧速度。具体周期和跟踪效果由使用者后续评估。

### Phase 3：接入现有定位和 Nav2

- 沿用现有仿真雷达采集点云和定位链路。
- 复用现有地图、代价地图和导航行为树，接入新的速度输出链路。
- 完成静止导航、平移导航、导航叠加恒速自转、导航叠加周期自转四组回归测试。

观察内容：先比较不自转与恒速自转时的导航效果，再试周期变速和换向。可使用 wz=`0`、`±1`、`±2` rad/s，以及 offset=1、amplitude=0.5、period=4 s 的正弦测试作为起点；次数、误差指标和最终验收由使用者后续确定。

## 8. 测试矩阵和记录指标

| 场景 | 导航 `vx/vy` | 自转 `wz` | 主要指标 |
|---|---:|---:|---|
| 空场直行 | 固定 `vx` | 0 | 速度误差、里程计漂移 |
| 空场横移 | 固定 `vy` | 0 | 横向误差、坐标变换 |
| 定点自转 | 0 | 恒定 | yaw 速率、周期、停止残差 |
| 平移+恒速自转 | 固定 | 恒定 | 轨迹、合成速度、TF 连续性 |
| 平移+周期自转 | 固定 | 周期 | 相位、峰值、导航路径偏差 |
| 障碍物导航 | Nav2 | 0 | 成功率、最小距离、恢复次数 |
| 障碍物导航+自转 | Nav2 | 恒定/周期 | 路径跟踪、避障效果、局部控制稳定性 |

每次回归至少记录 `/clock`、`cmd_vel_nav`、`rotation_cmd`、`rotation_velocity`、`cmd_vel_sim`、`odometry`、TF、原始点云、IMU、障碍扫描、Nav2 action feedback 和诊断状态。记录中必须能区分“导航请求的速度”“合成后的执行命令”和“实际运动速度”，并保存世界/模型参数、随机种子和软件版本以便复现。

## 9. 主要风险与处理

- **速度仿真过于理想**：当前用于观察导航平移和自转组合的效果，效果评估按使用者要求进行。
- **车体系约定错误**：所有接口文档固定 x 前、y 左、z 上；在测试中用纯 vx、纯 vy 和纯 wz 自动检查符号。
- **导航误发 wz**：控制器关闭旋转，mux 丢弃导航输入中的角速度，并在诊断中统计非零输入。
- **自转控制覆盖导航状态**：rotation controller 只写 mux 的 wz，不能发布或修改导航主题。
- **输入超时后继续运动**：接收侧守护线程检测输入中断并清零，暂停/reset 时清空速度缓存。
- **仿真与地图错位**：Gazebo 世界、SRM 初始位姿、栅格地图和 odom 原点通过同一份配置生成，不在 launch 中重复写偏移。

## 10. 建议的落地顺序

先统一 Twist 速度接口和 `base_link` 坐标约定，再实现 SRM Gazebo 空场速度执行及独立自转测试，确认纯 `vx`、`vy`、`wz` 和组合运动正确。随后接入现有雷达点云、定位和 Nav2 链路，观察“导航平移 + 自转”的组合效果，由使用者判断是否满足当前需求。
