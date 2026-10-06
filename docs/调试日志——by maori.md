> 历史记录：文中的旧仿真包及其入口现已删除；当前仿真入口为 `ros2 launch srm27_gazebo_simulator srm_sim.launch.py`，场地资源位于该包的 `resource/`。下文保留当时的命令与路径。

# 2026-8-10
## 1.连接WSL远程桌面
win+R,输入mstsc,连接

## 2.开启UbuntuSSH
### 开启ssh
sudo service ssh start
### 检查ssh状态（active running则ok）
sudo service ssh status

然后codex就可以连进wsl里了

### 关闭ssh
sudo service ssh stop

## 3.启动小海龟

ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key

## 4.快捷键打开终端
ctrl+alt+T

# 2026-8-14

## ros2 node list读不到节点
强制重置 daemon:
在终端执行：
```bash
ps -eo pid=,args= | awk '$0 ~ /python3 .*ros2-daemon/ {print $1}'
```
找到 daemon 的 PID 后：
```bash
kill -9 <PID>
```
```bash
ros2 daemon start
```
# 2026-8-15
### vscode不显示报错

Ctrl+ ,打开设置，修改Error Squiggles项

# 2026-8-16
### vscode配置c_cpp_properties.json添加路径
Ctrl+Shift+P,选择c/c++配置json

### 编译与运行
在workspace下colcon build编译
然后source install/setup.bash，重新加载环境
之后ros2 run...即可

# 2026-8-22
### 通过rqt看节点/话题关系：
终端输入rqt，然后选择：Plugins -> Introspection -> Node Graph

### PB25建图流程
##### 第一个终端：启动gazebo

```bash
source install/setup.bash

ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
```

默认世界由以下文件决定：

```text
src/rmu_gazebo_simulator/rmu_gazebo_simulator/config/gz_world.yaml
```
##### 第二个终端：启动rviz
```bash
source install/setup.bash

ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
  world:=rmuc_2025 \
  slam:=True \
  use_sim_time:=True \
  use_rviz:=True
```
##### 第三个终端：启动键鼠控制
```bash
source install/setup.bash

ros2 run rmoss_gz_base test_chassis_cmd.py \ --ros-args \ -r __ns:=/red_standard_robot1/robot_base \ -p v:=0.5 \ -p w:=0.5
```

# 2026-8-23
### 扫图文件
仿真里扫完图之后，pcd文件存放在：pb2025_sentry_ws/src/pb2025_sentry_nav/point_lio/PCD/scans.pcd

### 三种地图文件
rmuc_2025.yaml：二维栅格地图图像，供 Nav2 规划使用。
  - 白色：可通行区域
  - 黑色/深色：障碍物
  - 灰色：未知区域或代价区域
  
rmuc_2025.pgm ：栅格地图的描述文件，告诉 Nav2 如何解释 .pgm

rmuc_2025.pcd：               
  - Point-LIO 先验地图
  - small_gicp_relocalization 点云重定位
  - 通过点云匹配确定机器人在场地中的位置

### 启动导航仿真
##### 第一个终端：启动gazebo

```bash
source install/setup.bash

ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
```

默认世界由以下文件决定：

```text
src/rmu_gazebo_simulator/rmu_gazebo_simulator/config/gz_world.yaml
```
##### 第二个终端：启动rviz
```bash
source install/setup.bash

ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py world:=rmuc_2025 slam:=False use_pcd_localization:=False use_composition:=False use_sim_time:=True use_rviz:=True
```

Rviz上方选择nav2 goal就可以设置目标点和朝向

# 2026-8-24
### 查看用户后台进程
 ps -u "$USER" -o pid,ppid,stat,etime,cmd --forest
 
可以按命令名停止特定模块：

  pkill -TERM -f 'rviz2'
  pkill -TERM -f 'component_container_isolated'
  pkill -TERM -f 'ros2 launch pb2025_nav_bringup'
  pkill -TERM -f 'ros2 launch rmu_gazebo_simulator'

# 2026-8-26
### 小电脑密码
srm111

# 2026-8-30
### 小电脑打开steam++
点开主文件夹下的WattTookit,右键Steam++.sh,以程序运行

# 2026-8-31
### 地图：
pgm:二维静态栅格地图
同名yaml:描述它的一些参数
##### 查看方式：
eog + pgm地图地址
##### 修改方式:
gimp + pgm地图地址
左边栏套索图标，框取多边形，取消抗锯齿，点击上边栏编辑，以前景颜色填充（白色），Enter确认，然后导出即可完成修改
### 手动导航果冻版本仿真./script/start_oneclick.sh
##### 第一个终端：启动gazebo
```bash
source install/setup.bash

ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
``` 
##### 第二个终端：启动rviz
```bash
source install/setup.bashscript/*.sh

ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py world:=rmuc_2025 map:=/home/srm/pb2025_sentry_ws/src/pb2025_sentry_nav/pb2025_nav_bringup/map/simulation/rmuc_2025_tunnel.yaml slam:=False use_pcd_localization:=False use_rviz:=True

```
# 2026-9-1
### 地图加载不出来
重新编译仿真包：

```
source /opt/ros/humble/setup.bash

colcon build \
    --symlink-install \
    --packages-up-to rmu_gazebo_simulator pb2025_nav_bringup \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 8

source /home/srm/pb2025_sentry_ws/install/setup.bash
```
### 转URDF
[【诺丁汉大学联队】电控最应该看的课程，机械组入门教程01——如何导出URDF（维护TF树）-RoboMaster 社区](https://bbs.robomaster.com/article/810105?source=8)
# 2026-9-3
### fast-lio先查三类参数
（feature_extract_enable先false关掉）
雷达/IMU时间戳是否对其
雷达到imu的旋转方向和平移单位
filter_size/cube_side_length地图范围和降采样
# 2026-9-5
### nomachine
nuc的IP：192.168.1.101
自己的电脑IP配成：192.168.1.100
### 关于导航的理解（定位 / 重定位）

定位拆成两段：`odom -> base`（script/*.sh里程计给的）+ `map -> odom`（重定位给的）。
建图(`slam:=True`)产出两张地图：`slam_toolbox` 生成 PGM 二维栅格图（给 Nav2 规划），`point_lio` 生成 PCD 三维先验点云（给重定位用）。

比赛时 `slam:=False`。
没开重定位：`map -> odom` 是固定静态 TF（仿真默认恒等/0），完全靠里程计，越跑漂移越大。
开了重定位(`use_pcd_localization:=True`)的数据流：
  ```text
  雷达点云+IMU -> Point-LIO(激光+惯性里程计) -> 里程计(odom->base) + cloud_registered(配准点云)
                          cloud_registered --> small_gicp_relocalization(和先验PCD做GICP匹配) --> map->odom
  ```
##### 注意：
真正的点云匹配是 `small_gicp_relocalization` 做的，Point-LIO 不是纯预处理，它是里程计(状态估计)。
最终 pose = `map->odom` ∘ `odom->base`。GICP 锚定的是 `map->odom`，间接修正里程计漂移，不改 base 位姿。
开重定位时 Point-LIO 自己也会开 `prior_pcd.enable` 用先验 PCD 初始化位姿，所以有"两层锚定"，但对外发布 `map->odom` 的公开机制是 small_gicp。

### 检查建图是否漂移
```bash
ros2 run tf2_ros tf2_echo map odom
```
如果z 一直往上/往下飘，或 roll/pitch 持续变化 -> 漂移
z 稳定、roll/pitch 不变 -> 不漂移

# 2026-9-6
### 一键启动slam建图脚本
```bash
bash ./script/start_slam.sh
```
### 一键启动手动导航脚本
```bash
bash ./script/start_nav.sh
```
### 一键清理后台gazebo进程
```bash
bash ./script/kill_gzb.sh
```

# 2026-9-8
### 一键清理后台rviz进程
```bash
bash ./script/kill_rviz.sh
```
### 一键启动重定位仿真（基于先验 PCD）
```bash
bash ./script/start_nav_reloc.sh
```
### 修改代码后如何生效
`source install/setup.bash` **只是设置环境变量，不会加载/编译新代码**。

| 改动类型 | 是否需要编译 | 是否需要重启 launch |
|---|---|---|
| .py / .yaml / launch / 配置（--symlink-install） | 不需要 | **是**（进程启动时读入配置） |
| .cpp / C++ | **是**：colcon build | **是** |

```bash
colcon build --packages-select small_gicp_relocalization --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```
改完一定要**先停掉当前 launch（Ctrl+C），再重新运行**；否则跑的还是旧配置/旧二进制。

### rviz里查看先验点云地图
##### 启动
```bash
source install/setup.bash
python3 script/publish_pcd.py
```
##### rviz
左侧 Add → By topic → PointCloud2，Topic 选 prior_map
确认 Fixed Frame = map

# 2026-9-9
### 脚本启动不了权限不够
前面加bash 
### 点云地图怎么来
自己家里测试可以先跑一遍图直接建，然后回到原点开始跑
直接在官方给的场地地图上批点云（鱼姐视频、派大星息流）
但是实际场地有装配误差
### lio
lio同时用到点云和imu数据
问题：雷达一直转，lio定位会飘，不知道为什么
### TEB
也要用点云
局部规划器主要用来避障
目前参数可以用的
后续可以看一下MPC/MPPI
### wzc修的操作逻辑
拨轮：
小陀螺,正反拨各有快慢两档。

左拨杆：
下，无上位机辅助
中，使能云台+自瞄
上，使能导航+自瞄

右：
下，底盘，云台，机器人，摩擦轮关
中，使能遥控云台，底盘
上，使能扫头，（云台底盘分离）

# 2026-9-11
### 实车导航启动脚本
```bash
bash script/start_real_nav.sh --lio # 只有point-lio定位
bash script/start_real_nav.sh --lio --no-chassis # 不发串口消息
```

### 和底盘通信通不上
```bash
sudo chmod 666 /dev/ttyACM0
```

# 2026-9-13
### 启动小角落导航
```bash
bash ./script/start_real_nav.sh --lio -m xiaojiaoluo0913 --map-to-odom 0.0 0.0 0.0
```
### rviz设置多目标点
在rviz左下角nav2的设置，点waypoint那个，然后设置完多个目标点后点start

### p图相关
画笔和翻转很好用
记得设置前景色和背景色不要反了
地图之外都批黑

# 2026-9-14
### 重新扫了个图
```bash
bash ./script/start_real_nav.sh --lio -m xjl0914 --map-to-odom 0.0 0.0 0.0
```

# 2026-9-16
### 膨胀半径 0.7 -> 0.5
文件: `src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/srm_nav2_params.yaml`
(实车导航实际加载的那份, `start_real_nav.sh` 默认参数文件)
- 508 行 `local_costmap` -> `inflation_layer.inflation_radius: 0.5`
- 554 行 `global_costmap` -> `inflation_layer.inflation_radius: 0.5`
两张图一起改, 否则全局规划器会规划出局部代价地图认为太窄的路。
`robot_radius` 仍是 0.33, `cost_scaling_factor` 仍是 4.0 (没动)。

注意:
- 只是改小软缓冲区, 硬性不可通行区仍由 `robot_radius=0.33` 决定
  (缝隙 < 0.66m 依旧规划不过去)。现在 0.5 表示离障碍 0.5m 以外代价为 0,
  0.33~0.5m 之间是低代价软避让带。
- symlink 安装, 不用编译, 但要**重启导航栈**才生效 (参数只在节点构造时读一次)。
- 生效后确认:
  ```bash
  ros2 param get /local_costmap/local_costmap inflation_radius
  ros2 param get /global_costmap/global_costmap inflation_radius
  ```
- 若改完还是绕大圈, 把 `cost_scaling_factor` 从 4.0 降到 3.0 试试
  (它控制代价衰减速度, 越小高代价区摊得越广)。

# 2026-9-26
### TEB实车测试
```bash
bash /home/srm/pb2025_sentry_ws/script/start_real_nav.sh --lio -m 227_0916 --map-to-odom 0.0 0.0 0.0
```
# 2026-10-2
### 雷达内外参
##### 内参：
垂直角，偏移零点等
mid360出场已经标定好了
##### 外参：
旋转+平移六个自由度的相对机体的位姿（TF）
时间外参（mid360默认给0就行）

# 2026-10-4
### TF
##### TF树意义与维护者

TF 是一棵相对位姿的树，每条边 = 一个刚体变换；两帧之间的换算 = 把路径上的边依次乘起来。
导航要的是 `T(map→base) = T(map→odom) ∘ T(odom→base)`：`map` = 上次建图那张栅格图的坐标系（不动）、`odom` = 本次上电 Point-LIO 初始化那一刻车的位置（平滑连续但会漂）、`base_link` = 底盘。

`map→odom` 三选一且互斥：static（默认 / `--lio`）、`slam_toolbox`（`--slam`）、`small_gicp`（`--reloc`）。

`odom→base_link` 由 `sensor_scan_generation` 发：LIO 的雷达位姿 × 雷达安装外参 → 平面化。

`base_link` 下的 livox_frame / livox_imu / livox_scan / wheel_1..4 全是 `robot_state_publisher` 发的静态外参。

##### 小陀螺fake底盘维护

作用：造一个**不转的虚拟底盘**给 Nav2 用，再把速度转回真车体系。
核心公式：`fake 在 odom 里的朝向 = θ(底盘真实朝向) − 塞进节点里的那个角`。

塞 `θ`（现代码）→ 车永远不转，连正常转弯都看不见；塞 `0` → 等于 `base_link`（**阶段一：不测陀螺**）；塞 `s`（累计自旋角）→ 看得见转弯、看不见自旋（**阶段二：带陀螺**，yaw 给 `−s`）。

阶段一：角度源设 0，先关掉那个 −θ；
阶段二：角度源 = `s`（下位机回传），Nav2 的 `robot_base_frame` 全改 `base_link_fake`。
交给底盘前速度要转 `R(−s)`；`angular.z` 只在一处加自旋；

# 2026-10-6
### 远程控制
nomachine和todesk一起开的时候，拔nomachine网线的时候todesk关掉，然后再todesk接进去

### 1006建圖腳本
```bash
./script/start_real_slam.sh                #开始扫图
./script/start_real_slam.sh --save 227_1006  #扫完之后存地图到maps文件夹
```