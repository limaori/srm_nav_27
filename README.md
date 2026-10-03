# srm_nav_27

![PolarBear Logo](https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/.github/main/.docs/image/polarbear_logo_text.png)

## 1. 项目介绍

深圳北理莫斯科大学北极熊战队哨兵机器人 ROS 工作空间，包含串口通信、导航、行为决策和仿真模块。

## 2. Quick Start

SRM 实车 MID360 建图启动入口为 `bash script/start_real_slam.sh`，雷达配置见 [MID360 使用指南](./docs/mid360使用指南.md)。

### 2.0 Workspace Layout

- `src/srm27_bringup`：实车总启动入口、参数、地图和 RViz 配置。
- `src/srm27_behavior`：行为树服务端、客户端和插件。
- SRM 机器人模型：实车状态描述在导航 bringup 中维护，仿真 SDF 模型随 `rmu_gazebo_simulator` 安装。
- `src/srm27_navigation`：导航相关 ROS 包集合；其中各子目录是独立的 ROS 包。
- `src/dependencies`：第三方依赖；`src/interfaces/rm_decision_interfaces`：统一的 SRM 裁判、决策和云台消息接口；`src/tools`：开发工具。
- `script`：现场启动与诊断脚本；`docs`：使用指南和设计记录。

主要组件名称按职责统一为 `srm27_*`：

| 包 | 职责 |
| --- | --- |
| `srm27_bringup` | 实车总启动入口 |
| `srm27_nav_bringup` | Nav2 启动、地图、参数和机器人状态发布 |
| `srm27_nav_plugins` | SRM Nav2 行为插件和强度体素代价地图层 |
| `srm27_omni_pid_controller` | 全向底盘 PID 路径跟踪控制器 |
| `srm27_teleop_twist_joy` | 手柄速度和云台控制 |
| `srm27_behavior` | 行为树服务端、客户端及决策节点 |

工作区根目录按职责保留入口文档、依赖清单、脚本和源码。构建输出、运行日志和临时诊断结果不属于源码目录。

### 2.0.1 启动文件命名

导航启动文件按「入口 / 内部」两层命名，前缀即用途，避免同名歧义：

| 启动文件（`srm27_nav_bringup`） | 用途 |
| --- | --- |
| `nav_real_launch.py` | 实车导航入口（雷达 + 导航栈 + RViz + 手柄） |
| `nav_simulation_launch.py` | 仿真导航入口 |
| `nav_multi_simulation_launch.py` | 多机仿真导航入口 |
| `real_mapping_launch.py` | 实车 MID360 建图入口 |
| `real_robot_state_publisher_launch.py` | 实车整车 TF 发布（SRM 模型 + 雷达外参） |
| `standalone_robot_state_publisher_launch.py` | 独立调试用备用 TF 发布（读 `urdf/srm_robot.urdf`） |
| `nav2_stack_launch.py` | 内部：Nav2 主栈，按 `slam` 参数二选一拉起建图或定位 |
| `nav2_slam_launch.py` | 内部：`slam:=True` 分支（slam_toolbox + map_saver + Point-LIO） |
| `localization_launch.py` | 内部：`slam:=False` 分支（AMCL / small_gicp 重定位） |
| `navigation_launch.py` | 内部：控制器、规划器与行为服务器 |
| `rviz_launch.py` / `joy_teleop_launch.py` | 内部：RViz 与手柄遥控 |

参数与地图按「实车 / 仿真」分目录：`srm27_nav_bringup/config/{real,simulation}`、`map/{real,simulation}`、`pcd/{real,simulation}`。
实车参数有两份，用途写在文件名里：`nav2_params_srm.yaml`（SRM 车当前使用）、`nav2_params_upstream.yaml`（上游原版对照）；
建图参数与 RViz 配置分别为 `mapping_params.yaml`、`rviz/mapping.rviz`。

> [!NOTE]
> 本工作区未纳入版本控制，重命名后请重新构建以刷新 `install/` 中的软链接：
> `colcon build --symlink-install --packages-select srm27_nav_bringup srm27_bringup`

### 2.1 Setup Environment

- Ubuntu 22.04
- ROS: [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- Ignition: [Fortress](https://gazebosim.org/docs/fortress/install_ubuntu/)
- Install [small_icp](https://github.com/koide3/small_gicp):

    ```bash
    sudo apt install -y libeigen3-dev libomp-dev

    git clone https://github.com/koide3/small_gicp.git
    cd small_gicp
    mkdir build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
    sudo make install
    ```

### 2.2 Create Workspace

```bash
sudo apt install git-lfs
sudo pip install vcstool2
```

```bash
cd ~/srm_nav_27
```

```bash
vcs import --recursive . < dependencies.repos
```

> [!NOTE]
> `dependencies.repos` 文件已包含所有模块所依赖的仓库地址，无需手动查阅子模块的 README 手动克隆依赖。

> [!TIP]
> 使用命令 `vcs pull ./src` 可更新所有子模块。

### 2.3 Build

```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 10
```

> [!NOTE]
> 推荐使用 --symlink-install 选项来构建你的工作空间，因为它广泛使用 launch.py 和 YAML 文件。调整参数文件后通常不需要重建，只需重新启动节点。

### 2.4 Running

将会运行串口通信、导航和行为决策模块，参数读取自配置文件 [node_params](./src/srm27_bringup/params/node_params.yaml)。

```bash
ros2 launch srm27_bringup bringup.launch.py \
map:=/absolute/path/to/<YOUR_MAP>.yaml \
params_file:=/absolute/path/to/node_params.yaml \
use_rviz:=True
```

## 3. 常用调试启动命令

> [!NOTE]
> `map`、`prior_pcd_file` 和 `params_file` 使用绝对路径。上面的 `params_file` 可指向工作区自带的 [node_params](./src/srm27_bringup/params/node_params.yaml)。直接启动导航子模块时，再将 `<YOUR_WORLD_NAME>` 替换为 Nav2 地图文件名。

### 3.1 子模块

LiDAR (Livox Mid-360)

Mid-360 通过网口连接。电脑连接雷达的网卡需要配置为 `192.168.1.50/24`（不需要网关），雷达地址在
`src/srm27_navigation/livox_ros_driver2/config/MID360_config.json` 的 `lidar_configs[].ip` 中配置。
如果系统没有自动生成有线连接，可执行以下命令，其中 `<网卡名>` 用 `ip -br link` 查到的实际名称替换：

```bash
sudo nmcli con add type ethernet ifname <网卡名> con-name mid360 \
  ipv4.method manual ipv4.addresses 192.168.1.50/24 \
  ipv4.never-default yes ipv6.method disabled
sudo nmcli con up mid360
```

确认雷达在线后，启动驱动和 RViz（同时发布 Point-LIO 所需的自定义消息与 RViz 所需的 PointCloud2）：

```bash
source install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

默认点云话题为 `/livox/lidar/pointcloud`，坐标系为 `front_mid360`。可用以下命令检查数据：

```bash
ros2 topic hz /livox/lidar/pointcloud
ros2 topic hz /livox/imu
```

若雷达 IP 或 RViz 配置文件不同，可通过启动参数覆盖：

```bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py \
  config_file:=/绝对路径/MID360_config.json \
  rviz_config:=/绝对路径/display_point_cloud.rviz
```

Serial

```bash
ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py use_rviz:=True params_file:=<YOUR_PARAMS_FILE>
```

Navigation

```bash
ros2 launch srm27_nav_bringup nav_real_launch.py \
world:=<YOUR_WORLD_NAME>  \
slam:=False
```

Behavior Tree

```bash
ros2 launch srm27_behavior srm27_behavior_launch.py params_file:=<YOUR_PARAMS_FILE>
```

SRM 仿真模型

```bash
source install/setup.bash
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
```

仿真使用本工作区内置的 SRM 圆柱底盘、轮组和 MID360 模型，已适配 Gazebo Fortress。模型参数和兼容组件见 [模型说明](./src/rmu_gazebo_simulator/rmu_gazebo_simulator/resource/models/srm_sentry_robot/README.md)。模型资源随仿真包安装；实车 `real_robot_state_publisher_launch.py` 仍支持从 `srm_auto_sentry` 工作区读取原始模型。

### 3.2 Tools

Teleop gimbal

```bash
ros2 run teleop_gimbal_keyboard teleop_gimbal_keyboard
```

Convert .pcd to .pgm

```bash
ros2 launch pcd2pgm pcd2pgm_launch.py
```

Save map

```bash
ros2 run nav2_map_server map_saver_cli -f <YOUR_WORLD_NAME>
```

### 3.3 Rosbags

#### 3.3.1 Record

> [!TIP]
> [!NOTE]
> 本命令仅录制了传感器数据，没有直接录制 tf 信息，因此回放时启动导航应设置 `use_robot_state_pub:=True`，以使用 joint_state 数据生成并发布整车 TF。

方法一：根据裁判系统数据自动触发录包

设置 [node_params.yaml](./src/srm27_bringup/params/node_params.yaml) 中的 `standard_robot_pp_ros2.record_rosbag` 参数为 `True`，设置 `rosbag_recorder.topics` 为要录制的话题，裁判系统进入 5s 倒计时阶段时自动开启录制，进入比赛结算阶段时自动结束录制并保存。

方法二：命令行手动触发录包

```bash
source install/setup.zsh

ros2 bag record -o sentry_$(date +%Y%m%d_%H%M%S) \
/serial/gimbal_joint_state \
/livox/imu \
/livox/lidar \
--compression-mode file --compression-format zstd -d 30
```

#### 3.3.2 Play

> [!NOTE]
> 使用 `--clock` 参数，以发布 rosbag 中的时间戳到 `/clock` 话题。这意味着运行其他算法节点时，应设置 `use_sim_time:=True`。

```bash
ros2 bag play <YOUR_ROSBAG>.bag --clock
```

Example:

```bash
ros2 launch srm27_bringup bringup.launch.py \
world:=<YOUR_WORLD_NAME> \
use_composition:=False \
use_rviz:=True \
use_sim_time:=True \
use_robot_state_pub:=True
```
