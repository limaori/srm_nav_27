# wz 控制迁移导航与 fakeyaw TF 仿真实施方案

## 1. 目标与现状

目标是先在 Gazebo 仿真中完成两件事，再决定是否进入实车联调：

1. 将底盘角速度 `wz` 的控制权从独立自转链路迁移到 Nav2/MINCO 控制器。
2. 恢复 `fakeyaw`（仓库现有命名为 `base_link_fake`）坐标系，并把导航速度从 fakeyaw 系转换到真实 `base_link` 系。

当前实现存在一个必须先处理的接口冲突：

- `MincoMpcController` 已经可以输出带 `angular.z` 的 `TwistStamped`，但阶段一参数 `yaw_policy.mode=xy_only` 将其置零。
- 仿真 `srm_cmd_mux` 当前丢弃导航输入的 `angular.z`，只从 `rotation_velocity` 取 `wz`。
- 旧 `fake_vel_transform` 同时做速度旋转、发布 fakeyaw TF、叠加 `cmd_spin`；其中叠加自转的行为不能直接保留，否则会造成双重旋转。
- 当前 SRM 仿真导航参数统一使用真实 `base_link`，与本次 fakeyaw 导航方案不同，需要单独增加可回退配置。

本方案只改仿真和导航接口，不修改串口协议、下位机固件或实车启动默认值。

现有实现的主要落点是：`src/srm27_navigation/fake_vel_transform/`、
`src/srm27_chassis_control/src/cmd_mux_node.cpp`、
`src/srm27_chassis_control/src/velocity_mix.cpp`、
`src/srm27_navigation/srm27_minco_controller/src/minco_mpc_controller.cpp`、
`src/srm27_navigation/srm27_nav_bringup/launch/navigation_launch.py` 和
`src/srm27_navigation/srm27_nav_bringup/launch/nav_srm_simulation_launch.py`。
实施时先在这些现有边界内扩展，避免另起一套速度节点。

## 2. 目标数据流

```text
Nav2/MINCO（base_link_fake 系，输出 vx/vy/wz）
                 │ cmd_vel_nav_fake
                 ▼
fake_vel_transform
  发布 base_link -> base_link_fake TF
  速度：v_base = R(-yaw_base) * v_fake，wz 原样透传
                 │ cmd_vel_nav_body
                 ▼
srm_cmd_mux（control_mode=nav_se2，唯一最终发布者）
                 │ cmd_vel_sim
                 ▼
Gazebo SRM 速度适配器
                 │
                 ▼
真实 base_link、odom、chassis_odometry_gt
```

`rotation_cmd -> rotation_controller -> rotation_velocity` 只保留为旧模式回退和对照测试。`nav_se2` 模式下它必须被禁用或明确拒绝，不能与导航 `wz` 相加。

## 3. 坐标系和速度约定

### 3.1 TF 拓扑

保留唯一的动态底盘链：

```text
map -> odom -> base_link -> base_link_fake
                         └-> front_mid360 / livox_scan / wheels
```

- `odom -> base_link` 继续由真值里程计、Point-LIO、GICP 或 SLAM 中的唯一一个发布者提供。
- `base_link -> base_link_fake` 由 `fake_vel_transform` 动态发布，平移为零，旋转为真实底盘 yaw 的逆。
- `base_link_fake` 与 `base_link` 原点重合、z/roll/pitch 相同，只有 yaw 不同；其朝向在 odom 平面内保持稳定。
- 禁止让 `fake_vel_transform` 再发布 `odom -> base_link`，也禁止同时启用第二个 fake TF 发布者。

若真实底盘在 `odom` 中的 yaw 为 `ψ`，则：

```text
q(base_link -> base_link_fake) = yaw(-ψ)
v_base.x =  cos(ψ) * v_fake.x + sin(ψ) * v_fake.y
v_base.y = -sin(ψ) * v_fake.x + cos(ψ) * v_fake.y
wz_base  = wz_fake
```

角速度绕 z 轴在这次纯 yaw 坐标旋转下数值不变，不能再叠加 `cmd_spin`。

### 3.2 Nav2 参考系

在迁移配置中，将 Nav2 的 `robot_base_frame`、local/global costmap 的机器人 frame、MINCO 的 `command_frame` 统一设为 `base_link_fake`。传感器外参、里程计 `child_frame_id`、Gazebo 执行器仍使用 `base_link`。

MINCO 内部状态继续在 `odom` 系计算；输出阶段增加明确的命令系选择：

- `command_frame=base_link`：沿用当前实现，直接输出真实车体系速度。
- `command_frame=base_link_fake`：输出 `odom` 对齐的平移速度，不再提前乘一次 `R(-yaw)`，交给 `fake_vel_transform` 做唯一转换。

不能把已经是 `base_link` 系的 MINCO 输出再送入 fake 转换器。

## 4. 实施改动

### 4.1 MINCO/Nav2 角速度控制

1. 在 `srm27_minco_controller` 增加迁移用 `yaw_policy.mode=se2`（或等价命名），允许 MPC 输出 `wz`；`xy_only` 继续保留为回退模式。
2. 明确 yaw 参考策略：首版使用路径切线/目标 yaw 中已有的一种，禁止把独立自转请求偷偷混入 yaw 参考。记录 `requested_wz`、`applied_wz`、yaw 参考和限幅原因。
3. 增加 `command_frame` 参数，并按 §3.2 选择是否执行 `odomToBodyVelocity()`。对 `base_link_fake` 模式补充单元测试，验证 `ψ=0、±π/2、π` 的输出方向。
4. Nav2 Humble 外部话题继续使用现有 `Twist`/`TwistStamped` 兼容链路；如果 `velocity_smoother` 仍使用无时间戳 `Twist`，由现有导航时间语义保证，不在本阶段改消息协议。
5. 导航拥有 `wz` 后，目标检查器必须把角速度纳入停止条件。迁移配置使用 `StoppedGoalChecker` 或等价实现，设置 `rot_stopped_velocity`；不再使用只检查平动的 `OmniStoppedGoalChecker` 作为正式验收配置。

### 4.2 fake_vel_transform 改造

保留包名以减少启动改动，但将职责收敛为“动态 fakeyaw TF + 速度坐标转换”：

- 新增/统一参数：`robot_base_frame=base_link`、`fake_robot_base_frame=base_link_fake`、`odom_topic=odometry`、`input_cmd_vel_topic=cmd_vel_nav_fake`、`output_cmd_vel_topic=cmd_vel_nav_body`。
- 删除正式模式中的 `cmd_spin_topic` 和 `init_spin_speed` 叠加逻辑；收到非零 `cmd_spin` 时只报警，不影响导航输出。
- 优先在 odometry 回调中使用消息时间戳发布 TF；TF 发布失败、里程计超时或 yaw 非有限时，停止发布有效速度并输出零速。
- 转换器只读取最新有效 yaw，输入和输出队列保持 KEEP_LAST=1；每条输出都携带诊断计数，便于发现过期 yaw、无 odom 或重复转换。
- `wz` 必须逐条透传，增加 `wz_in - wz_out` 的误差统计。

### 4.3 srm_cmd_mux 仲裁

给 `srm_cmd_mux` 增加显式模式，不能通过“`wz` 是否为零”猜测所有权：

| 模式 | 平移来源 | `wz` 来源 | rotation 输入 |
|---|---|---|---|
| `legacy_independent_rotation` | 导航 | `rotation_velocity` | 启用 |
| `nav_se2` | fake 转换后的导航速度 | 导航 | 禁用/拒绝 |

`nav_se2` 下：

- `MixInput` 增加 `nav_wz`，最终 `wz=clamp(nav_wz, -wz_max, wz_max)`。
- 不允许 `nav_wz + rotation_wz`；收到 rotation 输入时发布一次告警并计数。
- 导航速度超时同时清零 `vx/vy/wz`；mux 的 `stop_all`、退出和执行端看门狗仍然有效。
- 诊断增加 `wz_source`、`dropped_rotation_count`、`nav_wz_clamped_count`，保留现有 `nav_wz_dropped_count` 供旧模式回归。

### 4.4 仿真启动与参数

新增迁移配置，不覆盖现有 SRM 基线：

- `config/simulation/nav2_params_srm_fakeyaw.yaml`：Nav2/MINCO 使用 `base_link_fake`，启用 `se2` 和角速度限幅。
- `nav_srm_simulation_launch.py` 增加 `use_fakeyaw_transform`、`wz_control_mode`、`command_frame` 参数；迁移默认开启，旧基线默认关闭。
- `fake_vel_transform` 放在 Nav2 输出和 mux 之间；mux 的 `nav_topic` 指向转换器输出。
- 先用 `use_ground_truth_odometry=true` 空场验证，再接入 Point-LIO/GICP/SLAM；定位模式切换时仍只允许一个 `odom -> base_link` 发布者。
- `start_sim_nav.sh` 增加 `--wz-source nav|rotation`、`--fakeyaw` 开关，启动日志打印最终速度链和 TF owner。

建议的交付文件如下：

| 文件 | 内容 |
|---|---|
| `fake_vel_transform/src/fake_vel_transform.cpp` | fakeyaw TF、一次性平移旋转、odom 超时保护 |
| `fake_vel_transform/include/.../fake_vel_transform.hpp` | 新参数、状态和诊断字段 |
| `srm27_chassis_control/include/.../velocity_mix.hpp`、`src/velocity_mix.cpp` | `nav_se2` 的 `nav_wz` 输入与模式化合成 |
| `srm27_chassis_control/src/cmd_mux_node.cpp` | 模式仲裁、rotation 拒绝、超时和诊断 |
| `srm27_minco_controller/src/minco_mpc_controller.cpp` | `se2` yaw 策略和 `command_frame` 输出 |
| `config/simulation/nav2_params_srm_fakeyaw.yaml` | fakeyaw frame、角速度限幅、停止条件 |
| `nav_srm_simulation_launch.py`、`start_sim_nav.sh` | 参数传递、节点顺序和可回退入口 |

## 5. 分阶段仿真计划

### P0：纯逻辑和 TF

- 运行 `srm27_chassis_control`、`fake_vel_transform` 相关单测。
- 用人工 odometry yaw `0、π/2、π、-π/2` 检查 TF 四元数、速度方向和 `wz` 透传。
- 检查 `view_frames`：`base_link_fake` 只有一个父节点，没有 TF 闭环或重复 child。

通过条件：线速度变换误差 `<1e-6 m/s`，`wz` 输入输出误差 `<1e-6 rad/s`，无重复 TF。

### P1：Gazebo 空场执行

不启动 Nav2，直接向转换器/mux 输入：

1. `vx=0.3, vy=0, wz=0`；
2. `vx=0, vy=0.3, wz=0`；
3. `vx=0.3, vy=0.2, wz=0.5`；
4. 逐步改变 yaw 后重复前三组。

记录 `cmd_vel_nav_fake`、`cmd_vel_nav_body`、`cmd_vel_sim`、`odometry` 和 TF。确认 Gazebo 实际 yaw 由 `wz` 积分，停止输入后不残留旧速度。

### P2：Nav2 静态地图 + 真值里程计

- 启动 `nav_srm_simulation_launch.py`，使用 `nav2_params_srm_fakeyaw.yaml`、`use_ground_truth_odometry=true`、`wz_control_mode=nav`。
- 先测不带障碍的单目标直行、横移、斜向移动，再测路径切线变化和目标停止。
- 对同一目标分别执行 `wz=0`、小正负角速度、角速度限幅场景；检查导航 `wz` 到底盘的单路径传递，不启动 `rotation_controller`。

### P3：定位链路回归

依次启用 Point-LIO、GICP/先验点云、SLAM，每次只启用一个 `odom -> base_link` 来源。重点观察：

- fakeyaw TF 是否跟随定位 yaw 连续变化；
- 定位重置、时间跳变、TF 暂时缺失时是否输出零速；
- Nav2 的局部代价地图、footprint 和全局路径是否仍与真实障碍对齐。

### P4：障碍物和故障注入

加入窄通道、急转、目标取消、控制器退出、odom 超时、TF 缺失、mux 超时、仿真暂停/reset。每项都应先停车，再恢复；不能沿用上一条非零 `wz`。

需要重点防止三类错误：

1. **重复旋转**：MINCO 已将 odom 速度转换成 `base_link`，又送入 fake 转换器；表现为 `ψ≈π/2` 时前进变侧移。
2. **角速度双重叠加**：导航 `wz` 与 `rotation_velocity` 同时进入 mux；表现为实际 yaw 速率约为两者之和。
3. **TF 竞争**：真值里程计、定位适配器或 fake 节点重复发布同一个 child；表现为 RViz 抖动、局部代价地图跳变或控制器间歇性 TF lookup 失败。

## 6. 验收指标与记录

每次仿真至少录制：

```text
/clock
/tf /tf_static
/odometry /chassis_odometry_gt
/cmd_vel_nav_fake
/cmd_vel_nav_body
/cmd_vel_sim
<controller>/diagnostics
<mux>/diagnostics
```

首轮验收门槛：

| 项目 | 门槛 |
|---|---|
| TF | `base_link_fake` 单父节点；无同一 child 多发布者；yaw 连续无跳变 |
| 速度坐标转换 | 四个基准 yaw 方向正确；转换前后平移速度模长保持不变 |
| `wz` 所有权 | `nav_se2` 下最终 `wz` 只来自导航；rotation 输入不会叠加 |
| 停车 | 取消、目标到达、导航/odom/TF/mux 超时均在看门狗时间内归零 |
| 导航回归 | 无自转时与现有 SRM 基线的路径成功率和到点行为不出现明显回退 |
| 组合运动 | 直行+自转、横移+自转、周期 yaw 变化时无发散、无重复速度变换 |

记录中必须区分“导航请求 `wz`”“转换器输出 `wz`”“mux 最终 `wz`”“里程计实际 `wz`”，不能只看 Gazebo 模型是否转动。

## 7. 回退和进入实车的前置条件

- 回退方式：关闭 `use_fakeyaw_transform`，恢复 `command_frame=base_link`、`wz_control_mode=rotation`、`yaw_policy.mode=xy_only`，使用现有 `nav2_params_srm.yaml` 和 `srm_cmd_mux` 行为。
- 仿真未完成前不改 `script/start_real_nav.sh` 默认出口、不打开实车角速度限幅、不修改串口协议。
- 只有 P0-P4 全部通过，并确认下位机接收的 `wz` 语义、最大角速度、停止看门狗和 odometry yaw 定义后，才另起实车联调方案。
- 实车第一阶段仍应保留独立急停和零速帧；导航拥有 `wz` 后，必须再次确认没有下位机自转链路与导航角速度重复叠加。
