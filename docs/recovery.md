# 导航恢复：职责、约束与验证

## 恢复由谁执行

决策层发送导航目标并处理结果。导航内部保留两层恢复，任何时刻只应有一层输出速度：

1. `MincoMpcController` 在当前任务的规划连续失败、机器人自身净空不足时启动 `RecoveryDecider`。它每周期根据最新 ESDF 选择方向，在有限窗口内尝试离开拥挤位置；结束后交回正常重规划。
2. 控制器上报失败或全局规划失败时，Nav2 行为树按预算清图、重试，并调用 `BackUpFreeSpace`。这个插件朝选定的空旷方向平移，名字沿用 Nav2 的 BackUp Action，实际可以前进、侧移或后退。

默认单点树是 `navigate_to_pose_w_replanning_and_recovery.xml`，默认多点树是 `navigate_through_poses_route_aware.xml`。旧的多点树仍可显式选用，三份树的恢复预算保持一致：外层重试 10 次、规划子任务重试 3 次、跟踪子任务重试 10 次；BackUp 最多移动 0.6 m，速度 0.2 m/s，期限 5 s。预算耗尽时导航返回失败，取消立即停止当前动作。

## 安全约束

- 方向和速度都必须注明其坐标系。控制器恢复与正常 MPC 输出共用 `velocityInCommandFrame()`；fakeyaw 链只在 `fake_vel_transform` 旋转一次。BackUp 选择的方向保存在地图系，每周期再转换到 `robot_base_frame` 发布。
- ESDF 环形采样只产生候选方向。整段线必须留在地图内，不能穿过原始障碍或未知区；起点净空不足时，允许沿净空逐步增大的方向退出，但不能先靠近另一面墙再穿过去。控制器恢复使用 `safety.robot_radius + safety.clearance_margin`，不另设一份包络参数。
- BackUp 必须初始化父类的前视时间，并按实际二维速度预演整个机器人 footprint。碰撞检查器不可用、地图或 TF 不可信、无可用方向、目标参数非法时停车失败。地图服务等待有期限。
- Humble 的 behavior server 使用 `costmap_topic` / `footprint_topic` 这一组碰撞输入，必须与它的 `global_frame` 同系。当前配置用全局地图及全局 footprint；新版本的 local/global 话题配置也保留。
- 脱困期限先于“成功”判断。输入失效会立即终止内部脱困，正常建轨宽限随后可以上报失败；不能因“正在脱困”反复延长异常期限。定位重置会清掉旧会话，方向丢失后不沿用旧速度。
- 后台规划清理使用任务代数，使已经开始计算的旧任务也不能写回结果；成功、失败、`already_at_goal` 都先检查当前目标、路径和限速版本。

净空是车心到原始障碍的距离，包含了机器人半径的用途；不是车壳与墙的缝隙。参数在各个实际启动的 YAML 中配置。修改源文件后必须重新构建、重启并核对运行日志；目前恢复参数不支持运行期热更新。

## 不上实车的验证

执行：

```bash
./tests/recovery/validate_recovery.sh
```

脚本使用独立 Docker 容器，网络关闭，不挂载设备或实车连接。首次构建导航测试镜像，之后复用；已有包含 ROS 2 Humble、Nav2、Gazebo Fortress、ROS-Gazebo bridge 和编译依赖的镜像可以通过 `SRM_RECOVERY_IMAGE` 指定。

验证分三层：

| 层次 | 检查内容 |
| --- | --- |
| 算法及组件 | ESDF、MINCO/MPC、重规划、命令坐标系、后台任务清理、恢复门限/冷却/期限、封闭围墙和完整线段、BackUp 扇区及前视时间、fakeyaw、mux 和启动配置 |
| ROS 平面闭环 | 真实 Nav2/MINCO/fakeyaw/mux：不同航向下脱困、侧移遇到新障碍、无方向拒绝、取消与期限停车、输入断流、恢复后换目标、Nav2 行为树兜底、窄道和绕障的多点导航 |
| Gazebo | 使用生产 SRM 几何、速度执行插件与适配器、实际物理引擎、雷达/IMU、生产真值里程计适配器与机器人 TF 链：90° 航向脱困及断流后恢复、绕障、窄道、到点停车，并检查实际轨迹相对障碍的车体净距 |

所有源码只读挂载，输出保存在 `.cache/recovery-validation/`：`manifest.json`、JUnit、各包测试结果、场景日志、实际轨迹、诊断、净空和传感器计数。断言失败会使脚本非零退出，不会生成新的成功清单。测试使用真实 ROS Action 和插件；外部输入仅替换为独立地图、平面运动模型或 Gazebo。

`tests/recovery/inject_recovery_stuck.py` 用于隔离仿真中的地图故障注入。它直接处理 ROS OccupancyGrid 的行序：x、y 从地图原点递增；图片文件的翻转规则不能用于消息数据。脚本打印的环形端点净空仅供参考，最终速度还要经过控制器的整段通行检查。

## 验收边界

仿真采用已知静态地图与真值里程计；雷达和 IMU 的实际输出只作传感器可用性检查，不代表 Point-LIO/GICP/地形感知链已验收。恢复速度为 0.2 m/s，闭环导航验证使用 0.5 m/s 上限。实车制动、通信延迟、定位漂移、动态障碍感知，以及默认高速档的表现，需要分别在现场验收。

## 本次验证记录

2026-11-16，WSL 上执行完整入口 `./tests/recovery/validate_recovery.sh`，专用镜像从导航测试 Dockerfile 构建。结果：

- 35 个定向 CTest 程序全部通过，其中 C++ gtest 共 365 个用例。
- 地图注入脚本 3 个回归用例全部通过。
- ROS 闭环 11 个场景、Gazebo 3 个场景全部通过，Gazebo 三个场景均收到实际点云和 IMU。
- 场景检查包含取消、断流、非法参数、服务失联、侧向新障碍、行为树兜底后换目标，以及真实模型的绕障、窄道和停车；三组 Gazebo 轨迹均通过车体净距检查。

详细计数、JUnit、日志、传感器输出计数和净距在 `.cache/recovery-validation/`；这些运行产物不进入 Git。该记录仅对应上面的隔离验证层次，不扩展为现场验收结论。
