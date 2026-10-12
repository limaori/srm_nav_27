# fake_vel_transform

本功能包启动时，Fake Velocity Transform 会创建一个 `fake_robot_base_frame` 的坐标系，其 x, y, z 与 `robot_base_frame` 的坐标系一致，但 yaw 固定指向正前方，然后将生成的该坐标系发布到 tf2。同时，Fake Velocity Transform 会订阅 `input_cmd_vel_topic` 话题，将接收到的速度指令转换到 `robot_base_frame` 坐标系下，并发布到 `output_cmd_vel_topic` 话题。

主要目的是用于适配 NAV2 局部路径规划器，当速度参考坐标系 `robot_base_frame` 变化剧烈时，如云台处于自旋扫描时，NAV2 局部路径规划器会将机器人的方向视为与当前路径规划方向一致，导致机器人无法正常运动。而使用 `fake_robot_base_frame` 可以规避这个问题，实现较稳定的轨迹跟踪效果。

由于 NAV2 humble 发行版出于避免破坏原有接口的原因，依然使用 Twist 类型（不含时间戳），humble 往后的版本才使用 TwistStamped，导致无法直接实现 cmd_vel 与 odometry 的时间戳对齐。因此，本功能包暂时订阅 local_plan 话题（由局部路径规划器发布），以获取时间戳，将它的时间戳视为 cmd_vel 的时间戳，以间接实现时间戳对齐。
Related issue: [Switch from Twist to TwistStamped for cmd_vel #1594](https://github.com/ros-navigation/navigation2/issues/1594)

## 两种模式（`transform_mode`）

2026-10 的 wz 控制迁移（见 `docs/wz控制迁移导航与fakeyaw仿真实施方案.md`）把本节点
的职责收敛成"动态 fakeyaw TF + 速度坐标转换"，并引入显式模式参数：

* `transform_mode=legacy`（**默认**，与上面的历史描述一致）：`odom` 与 `local_plan` 近似
  时间同步后再转换；`cmd_spin`（及 `init_spin_speed`）会**叠加**到输出的 `angular.z` 上；
  导航控制器不活跃时输入速度直通（供手柄等已处于车体系的输入使用）。
  实车链路 `config/real/nav2_params_srm*.yaml` 与旧仿真链路都走这一条。
* `transform_mode=nav_se2`（wz 控制迁移模式）：**导航拥有 `wz`**。
  * 不再做任何自转叠加 —— 收到非零 `cmd_spin` 只告警、只计数（`non_zero_spin_count`），
    避免"导航 wz + cmd_spin"的双重叠加；
  * 每条输入都用**最新一条里程计**的 yaw 转换一次（`v_base = R(-psi) * v_fake`，
    `wz` 原样透传），不做 `local_plan` 同步；里程计订阅是 KEEP_LAST=1 的普通订阅；
  * TF 在里程计回调里用**消息时间戳**发布（`base_link -> base_link_fake`，yaw = -psi，平移为零）；
  * 里程计缺失 / 超过 `odom_timeout` / yaw 非有限时，输出零速并停止更新 TF，同时计数；
  * 诊断（`diagnostics`）给出 `yaw_deg` / `odom_age` / `zero_output_count` /
    `stale_odom_count` / `nonfinite_yaw_count` / `wz_error_max` / `wz_error_mean` 等计数。

两种模式都不发布 `odom -> base_link`（那由定位或真值里程计单独拥有）。

## Published Topics

* `tf` (`tf2_msgs/msg/TFMessage`) - 与机器人可移动关节相对应的变换
* `output_cmd_vel_topic` (`geometry_msgs/msg/Twist`) - 转换后的速度指令

## Subscribed Topics

* `input_cmd_vel_topic` (`geometry_msgs/msg/Twist`) - 机器人的速度指令
* `local_plan_topic` (`nav_msgs/msg/Path`) - 局部路径规划器的路径
* `odom_topic` (`nav_msgs/msg/Odometry`) - 里程计数据
* `cmd_spin_topic` (`example_interfaces/msg/Float32`) - 控制底盘固定旋转速度，将会叠加到 `output_cmd_vel_topic` 中

## Parameters

* `odom_topic` (`string`, default: "odom") - 里程计话题。里程计的 frame_id 与 `robot_base_frame` 参数保持一致
* `robot_base_frame` (`string`, default: "gimbal_link") - 速度参考坐标系
* `fake_robot_base_frame` (`string`, default: "gimbal_link_fake") - 伪速度参考坐标系
* `local_plan_topic` (`string`, default: "local_plan") - 局部路径规划器的路径话题
* `cmd_spin_topic` (`string`, default: "cmd_spin") - 控制底盘固定旋转速度的话题
* `input_cmd_vel_topic` (`string`, default: "") - 输入速度指令的话题
* `output_cmd_vel_topic` (`string`, default: "") - 输出速度指令的话题。将原本基于 `fake_robot_base_frame` 的速度变换到 `robot_base_frame` 后发布
* `init_spin_speed` (`double`, default: 0.0) - 若没有接收 `cmd_spin_topic`，则使用该值作为固定旋转速度（仅 `legacy` 模式）
* `transform_mode` (`string`, default: "legacy") - `legacy` 或 `nav_se2`，见上一节
* `odom_timeout` (`double`, default: 0.25) - `nav_se2` 模式下的里程计最大年龄（s），超过则输出零速
* `diagnostics_period` (`double`, default: 0.5) - `nav_se2` 模式下的诊断发布周期（s）

## Subscribed Topics (`nav_se2` 模式)

* `diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`) - **发布**：转换计数与有效性
* `nav_se2` 下 `local_plan_topic` 不再被订阅；`cmd_spin_topic` 只用于告警计数
