# 实车分段验收：雷达 → ZLIO 前端 → odom→cloud → backend → 速度链 → Nav2

> 状态：现场操作手册。**一次只加一层**，前一层不过不要进下一层。
> 背景与契约见 [backend 接入契约](../architecture/localization_backend.md)、
> [迁入实施记录](../records/2026-11-16-zlocalization-backend-migration.md)。
> 最后核对：2026-11-17（含首轮实测发现的 2 个已知问题）

## 0. 每次开工前（30 秒）

```bash
cd ~/srm_nav_27
source /opt/ros/humble/setup.bash
source install/setup.bash
env | grep -c srm_auto_sentry          # 必须是 0；不是 0 就先开新终端
pkill -f "[r]obot_state_publisher"     # 清掉上一次残留（方括号不能省）
pkill -f "[l]ivox_ros_driver2_node"
pkill -f "[z]lio_node"
pkill -f "[n]av2"                      # 确认没有别的栈在跑
```

一条命令全停（会先发零速）：

```bash
./scripts/bringup/start_real_nav.sh --stop
```

公共变量（后面每条命令都用得到）：

```bash
export PARAMS=$PWD/src/bringup/srm27_nav_bringup/config/real/current/nav2_params_srm_minco_fakeyaw.yaml
export MAP=$PWD/src/bringup/srm27_nav_bringup/map/real/srm_site_01.yaml
export ZLIO=$PWD/src/bringup/srm27_nav_bringup/config/real/zlio_mid360.yaml
export LIDAR_CFG=$PWD/src/bringup/srm27_nav_bringup/config/real/mid360_user_config.json
export LOGDIR=$PWD/.deps/field; mkdir -p $LOGDIR
# ⚠ 本机 /opt/MVS 的老 libusb 会让 PCL 节点 exit 127，统一剔掉（脚本里也会自动剔）
export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v '/opt/MVS' | paste -sd:)
# ⚠ livox 驱动需要 SDK 库：install 里没装，指到源码树那份
export LD_LIBRARY_PATH=$PWD/src/localization/frontend/livox_ros_driver2/Livox-SDK2/lib/amd64:$LD_LIBRARY_PATH
```

## 1. 雷达 + IMU + 车体 TF

```bash
# 终端 A：车体 TF
ros2 launch srm27_nav_bringup real_robot_state_publisher_launch.py \
  use_sim_time:=False params_file:=$PARAMS lidar_config:=$LIDAR_CFG \
  gimbal_joint_type:=revolute gimbal_is_root:=true 2>&1 | tee $LOGDIR/rsp.log

# 终端 B：雷达驱动（-r __node 不能省，否则参数整段被忽略）
ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -r __node:=livox_ros_driver2 --params-file $PARAMS \
  -p user_config_path:=$LIDAR_CFG 2>&1 | tee $LOGDIR/lidar.log
```

| 检查 | 命令 | 通过标准 |
|---|---|---|
| 雷达频率 | `ros2 topic hz /livox/lidar` | ≈10 Hz（实测 9.1 Hz、最大间隔 0.3 s 属可接受，但要记录） |
| IMU 频率 | `ros2 topic hz /livox/imu` | ≈200 Hz |
| 帧名 | `ros2 topic echo /livox/lidar --field header --once` | `frame_id: livox_frame` |
| IMU 单位 | `ros2 topic echo /livox/imu --field linear_acceleration --once` | 静止时 \|a\|≈1.0（**g**） |
| TF 树 | `ros2 run tf2_tools view_frames` → `evince frames_*.pdf` | `gimbal_yaw→base_link→…→livox_frame→livox_imu`、`livox_scan`、四轮 |
| 雷达外参 | `ros2 run tf2_ros tf2_echo base_link livox_imu` | ≈(0.127, −0.048, 0.297)，RPY(30°,0,63°) |
| 云台关节 | `ros2 topic echo /joint_states --once` | 有 `gimbal_yaw_joint`（需要底盘串口回传；没接时是固定 0°，可接受） |

> 已知问题①：`install/livox_ros_driver2/lib` 缺 `liblivox_lidar_sdk_shared.so`，且
> `liblivox_ros_driver2.so` 的 RUNPATH 指向重组前的旧路径。上面 `LD_LIBRARY_PATH` 是临时绕过；
> 正解是重建该包（或给它的 CMake 加 install 规则）。

## 2. ZLIO 前端 + zlio_bridge（**当前实测卡住，见 §2.1**）

```bash
# 终端 C（雷达/QtF 仍在跑）
ros2 launch srm27_nav_bringup localization_launch.py \
  map:=$MAP params_file:=$PARAMS use_sim_time:=False use_composition:=False \
  use_lio_odometry:=True use_pcd_localization:=False odometry_frontend:=zlio \
  zlio_config:=$ZLIO zlio_input_type:=livox zlio_lidar_topic:=livox/lidar \
  zlio_imu_topic:=livox/imu_saturated zlio_imu_acceleration_scale:=9.81 \
  zlio_body_frame:=livox_imu 2>&1 | tee $LOGDIR/zlio.log
```

| 检查 | 命令 | 通过标准 |
|---|---|---|
| 观测帧 | `ros2 topic hz /zlio/frame` | ≈10 Hz |
| 桥输出 | `ros2 topic hz /registered_scan` / `/lidar_odometry` | ≈10 Hz |
| 会话 | `ros2 topic echo /zlio/session --once` | 有 instance_id / epoch |
| 匹配质量 | `ros2 topic echo /zlio_bridge/matches --once` | 明显 >30 |
| 关键日志 | 见 `$LOGDIR/zlio.log` | `已捕获 安装外参 T(...)`、`首帧 ZLIO 位姿 R_0 ... 已按 T_off = ... 抵消`、`已转发 N 帧` |
| RViz | Fixed Frame=`odom`，加 PointCloud2(`registered_scan`)、Odometry(`lidar_odometry`) | 点云在 odom 系、地面平整；推车 1 m 位姿跟 1 m |

### 2.1 已知问题②：ZLIO 在真实 10 Hz 数据下起不来（2026-11-17 实测定性）

现象（可复现）：

- `/zlio/frame` **一直没有**；7–10 s 后报
  `Livox input failed: input event queue capacity exceeded`；
- 把 `maximum_buffered_points` 加到 6M/20M 后不再溢出，但改成
  `odometry input interrupted: posterior state precedes the retained IMU history watermark`，仍无帧；
- 换 `pointcloud2` 输入（`/livox/lidar/pointcloud` + `point_time_field:=timestamp`、
  `point_time_scale:=1e-9`、`point_time_absolute:=true`）**同样**失败；
- 换仿真版 `zlio_mid360.yaml` **同样**失败；
- 进程启动期 CPU≈95%，之后掉到 0.3%、线程全在 futex 等待。

复现命令（单节点，便于抓日志）：

```bash
ros2 run frontend zlio_node --ros-args \
  -p config_file:=$ZLIO -p input_type:=livox -p lidar_topic:=livox/lidar \
  -p imu_topic:=livox/imu -p imu_acceleration_scale:=9.81 -p body_frame:=livox_imu \
  -p publish_tf:=false -p world_frame:=odom -p reset_mode:=warm -p coordinated_reset:=false \
  -p wait_for_start:=false 2>&1 | tee $LOGDIR/zlio_standalone.log
```

要留的证据：上面这条命令的完整输出 + `ros2 topic echo /livox/lidar --field timebase --once`
+ `/livox/imu` 的 `header`（确认两者同一时钟）。

临时可试（只用于定性，不是结论）：

```bash
# 1) 加大输入预算，看是否只是初始化耗时超过队列
-p maximum_buffered_points:=20000000 -p maximum_queue_events:=65536
# 2) 不用 timebase，用 header.stamp 作逐点时间原点
-p livox_use_timebase:=false
```

## 3. `sensor_scan_generation`（odom→gimbal_yaw）

`localization_launch` **不含**这个节点，单独起：

```bash
ros2 run sensor_scan_generation sensor_scan_generation_node --ros-args \
  -r __node:=sensor_scan_generation --params-file $PARAMS \
  2>&1 | tee $LOGDIR/sensor_scan.log
```

| 检查 | 命令 | 通过标准 |
|---|---|---|
| 车体位姿 | `ros2 topic hz /odometry` | ≈10 Hz，`child_frame_id=gimbal_yaw` |
| TF | `ros2 run tf2_tools view_frames` | 出现 `odom→gimbal_yaw→base_link` |
| 三路 yaw 一致 | `./debug_tools/diag_gimbal_chain.py` | 退出码 0（云台转 ≥360° 时） |
| 不画圆 | `./debug_tools/capture_gimbal_spin.py --duration 30` | `odom→gimbal_yaw` 的 xy 不随云台转画圆 |
| RViz | Fixed Frame=`odom`，看 TF + `/odometry` + `sensor_scan` | 云台转时车头跟转、位置不漂 |

> 云台判据需要 `/joint_states` 有发布者 → 需要底盘串口在（`srm27_nav_protocol`）。

## 4. backend（map→odom）

先用本场地图生成地图包（**必须**地图系那份 `_mapframe.pcd`）：

```bash
source scripts/setup/env_srm27.sh         # ROS + 本工作区 + GTSAM；没装 GTSAM 见 scripts/setup/install_gtsam_4_2.sh
ros2 run srm27_zlocalization_backend srm27_zlocalization_backend_build_pcd_map_bundle \
  --input maps/pcd/<site>_mapframe.pcd \
  --out-bundle maps/pcd/map_bundles/<site> \
  --x-min <min> --x-max <max> --y-min <min> --y-max <max> \
  --sensor-z 0.3464 --query-step 2.0 --bundle-step 1.5 \
  --map-voxel 0.15 --scan-voxel 0.15 --sc-lidar-height 0.3464
```

起 backend（定位链，不含 Nav2）：

```bash
ros2 launch srm27_nav_bringup localization_launch.py \
  map:=$MAP params_file:=$PARAMS use_sim_time:=False use_composition:=False \
  use_lio_odometry:=True use_pcd_localization:=False odometry_frontend:=zlio \
  zlio_config:=$ZLIO zlio_input_type:=livox zlio_lidar_topic:=livox/lidar \
  zlio_imu_topic:=livox/imu_saturated zlio_imu_acceleration_scale:=9.81 \
  zlio_body_frame:=livox_imu \
  use_zlocalization_backend:=True zlocalization_prior_map:=<bundle 绝对路径> \
  2>&1 | tee $LOGDIR/backend.log
```

| 检查 | 命令 | 通过标准 |
|---|---|---|
| 状态机 | `ros2 topic echo /backend_node/localization_state` | `INITIALIZING → NORMAL`，`pose_trusted=true` |
| 唯一 TF owner | `ros2 node list \| grep -E "static_transform\|backend\|gicp\|slam"` | **只有一个** |
| TF 结构 | `ros2 run tf2_tools view_frames` | `map` 只有 `odom` 一个子节点 |
| 地图对齐 | RViz Fixed Frame=`map`，加 `/backend_node/prior_map`、`/localization/odometry` | 先验点云与 PGM 对齐 |
| 手动 reset | `ros2 service call /localization_reset std_srvs/srv/Trigger "{}"` | epoch+1；`map→odom` 不跳；`odom→gimbal_yaw` 连续 |
| 错图拒绝 | 换别的场地 bundle | 停在 `INITIALIZING/SUSPECT`，**不能**进 `NORMAL` |
| 退化几何 | 正对单墙/走廊 | 掉到 `SUSPECT`，`matches` 明显下降，无跳变 |
| 断流恢复 | 拔雷达 2 s | 前端 `reset_required` → backend 事务 → epoch+1、位姿连续 |

## 5. 速度链（**架起车**，急停到手）

```bash
./scripts/bringup/start_real_nav.sh --lio --lio-frontend zlio --no-joy
```

| 检查 | 命令 | 通过标准 |
|---|---|---|
| 串口反馈 | `ros2 topic hz /chassis_speed` | 有反馈 |
| 跟随 | `./debug_tools/plot_vxvywz.py` | 发送 vs 反馈跟随（滞后 ~0.15 s 正常） |
| 执行坐标系 | `./debug_tools/frame_probe.py --drive --speed 0.3 --yes --cmd-topic /cmd_vel_nav_body` | 判据给出"下位机按云台系/底盘系执行" |
| 方向 | 前进/左移/自转各一次 | 方向、符号正确 |

## 6. Nav2 闭环

```bash
./scripts/bringup/start_real_nav.sh --lio --lio-frontend zlio
# 0.5 m 目标 → 1 m → 2 m → 绕障 → 叠加云台转动
```

每次盯：`ros2 topic hz /tf`、`ros2 node list | grep -E "static_transform|backend|gicp|slam"`（唯一 owner）、
`ros2 topic echo /cmd_vel_chassis`（有输出）、RViz `nav2_default_view.rviz`。

## 7. 每层都要留证据

```bash
ros2 bag record -o ~/srm_nav_27_data/bags/<阶段>_$(date +%H%M) \
  /livox/imu /tf /tf_static /zlio/frame /zlio/session \
  /registered_scan /lidar_odometry /odometry \
  /backend_node/localization_state /backend_node/localization_pose 2>&1 | tee -a $LOGDIR/bag.log
cp $PARAMS $LOGDIR/params_$(date +%H%M).yaml
```

> 不录 `/livox/lidar`（CustomMsg，体积大）；需要复现 ZLIO 问题时再单独录 30 s。

## 8. 出问题先看哪

| 现象 | 先查 |
|---|---|
| 驱动起不来，`liblivox_lidar_sdk_shared.so: cannot open` | `LD_LIBRARY_PATH` 加上源码树 `Livox-SDK2/lib/amd64`（或重建包） |
| 节点 `exit 127` / `undefined symbol: libusb_set_option` | `LD_LIBRARY_PATH` 里剔掉 `/opt/MVS` |
| 位姿整体歪 ~30° | 首帧 `T_off` 日志、IMU 是否 g 单位、`body_frame` |
| backend 永远 INITIALIZING | `sensor_frame` 是否等于 `livox_imu`、backend 是否收到 `frontend/frame` |
| 地图/点云错位 | 用的是 `*_mapframe.pcd` 生成的 bundle 吗 |
| TF 打架/地图来回跳 | 是否同时起了 static/backend/gicp/slam |
| 云台转时位置画圆 | `pose_body_offset_xyz` 需标定（`capture_gimbal_spin.py` 给建议值） |
