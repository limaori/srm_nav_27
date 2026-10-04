# 将 TDT 导航算法移植到 `srm27_sentry_ws`

> [!NOTE]
> 本文写于 SRM 仿真重构之前。下文链路图中的 `fake_vel_transform` 属于**旧步兵链路**，
> SRM 仿真已删除该层：导航速度出口为 `cmd_vel_nav`，再由 `srm_cmd_mux` 与独立自转合成为
> `cmd_vel_sim`（见 [`SRM仿真与自转控制实现(ai).md`](./SRM仿真与自转控制实现%28ai%29.md)）。
> 算法移植、Costmap 与行为树相关的结论不受影响。

## 目标

TDT 仓库开源的是二维栅格导航算法组件，主要包括：

- `YAstar`：普通 A*、代价地图、人工势场、多层动态障碍物 Mask、路径简化和碰撞检查。
- `KinodynamicAstar`：带速度、加速度和时间约束的动力学 A*。
- `MinimumSnapOsqp`：多项式轨迹优化，支持闭式求解和 OSQP 求解。
- `SfcSquare`：根据二值栅格地图生成方形可行区域。

它不是完整的 ROS 2 导航系统，因此移植时应把它作为 Nav2 的算法库或插件接入，继续复用现有的定位、建图、Costmap、行为树和底盘控制链路。

## 当前 `srm27_sentry_ws` 导航链路

当前导航流程可以概括为：

```text
Point-LIO / SLAM / PCD 定位
          ↓
terrain_analysis 和 Nav2 Costmap
          ↓
ThetaStar 全局规划器
          ↓
SimpleSmoother 路径平滑器
          ↓
OmniPidPursuitController 路径跟踪
          ↓
fake_vel_transform / velocity_smoother
          ↓
底盘 cmd_vel
```

相关配置位于：

`src/srm27_navigation/srm27_nav_bringup/config/real/nav2_params_upstream.yaml`

其中当前使用：

- 全局规划器：`nav2_theta_star_planner/ThetaStarPlanner`
- 平滑器：`nav2_smoother::SimpleSmoother`
- 控制器：`srm27_omni_pid_controller::OmniPidPursuitController`

## 可以替换的模块

### 1. 用 YAstar 替换 ThetaStar 全局规划器

可以将 `YAstar::search()` 封装为一个继承 `nav2_core::GlobalPlanner` 的 ROS 2 插件，用它替换当前的 ThetaStar。

规划器插件需要完成以下工作：

1. 从 Nav2 接收全局 Costmap、起点和终点。
2. 将 Costmap 转换为 YAstar 所需的地图格式。
3. 设置地图分辨率和地图原点。
4. 调用 `YAstar::search(start, goal)`。
5. 将返回的 `Eigen::Vector2f` 路径转换为 `nav_msgs::msg::Path`。

YAstar 还可以在搜索后调用路径简化函数，减少路径点数量，并保留对障碍物的直线碰撞检查。

需要注意，YAstar 的普通 A* 是八邻域栅格搜索，ThetaStar 使用了更强的可见性直连策略，两者生成的路径形状和性能会不同，不能假设替换后行为完全一致。

### 2. 用 MinimumSnap 替换 SimpleSmoother

这是最适合 TDT 算法的接入位置。可以将 `MinimumSnap::solve()` 封装为 Nav2 的路径平滑器插件：

```text
nav_msgs/Path
      ↓
Eigen 控制点
      ↓
MinimumSnap::solve()
      ↓
采样后的平滑路径
      ↓
nav_msgs/Path
```

MinimumSnap 可以提供：

- 多项式轨迹平滑；
- 闭式求解、OSQPPath 和 OSQPCorridor 后端；
- 基于路径点的方形可行区域；
- 途经点速度约束；
- 时间分配和时间归一化；
- 轨迹采样和碰撞检查；
- 发生碰撞后的迭代重新求解。

建议先用 `Close` 后端完成基础接入，再根据需要启用 `OSQPCorridor`。

### 3. YAstar + MinimumSnap 组合替换现有规划后处理

推荐的基础流程是：

```text
YAstar A*
    ↓
YAstar 路径简化
    ↓
MinimumSnap + SfcSquare
    ↓
现有 OmniPidPursuitController
```

这样可以让前端负责寻找可达通路，让后端负责路径平滑和轨迹质量，同时保留现有控制器和底盘接口。

## 不建议替换的模块

### 1. 不替换 OmniPidPursuitController

TDT 的 MinimumSnap 输出的是路径点或采样轨迹，不输出 `geometry_msgs/msg/Twist` 或 `cmd_vel`。因此它不能直接替代 `srm27_omni_pid_controller`。

现有 Omni Pure Pursuit 控制器可以继续跟踪 MinimumSnap 输出的 `nav_msgs/Path`。

### 2. 不替换 fake_vel_transform 和 velocity_smoother

这两个模块负责速度坐标系转换和速度指令平滑，TDT 没有对应实现，应继续保留。

### 3. 不替换点云、定位和地形处理链路

以下模块与 TDT 导航算法职责不同，应继续使用：

- Livox 驱动；
- Point-LIO；
- SLAM Toolbox；
- Small GICP 重定位；
- `terrain_analysis` 和 `terrain_analysis_ext`；
- `IntensityVoxelLayer`；
- Nav2 地图服务器、Costmap、行为树和恢复行为。

TDT 接收的是已经整理好的二维栅格地图，不负责从点云生成地图。

### 4. 不直接用 TDT 替换 Nav2 Costmap

YAstar 自带代价地图和人工势场计算，但当前工作区的 Costmap 还承担动态点云障碍物、地形分析、膨胀和机器人 footprint 处理。建议保留 Nav2 Costmap，将它的结果转换给 YAstar 或 MinimumSnap 使用。

## KinodynamicAstar 的接入方式

`KinodynamicAstar` 可以作为新增的动力学规划模式，也可以实现为自定义 Nav2 全局规划器，但它不是普通的直接替换：

- 输入包含起点速度和加速度；
- 状态为 `[x, y, vx, vy]`；
- 输出包含位置、速度、加速度和时间；
- 普通 Nav2 `nav_msgs/Path` 接口通常只保存位姿，会丢失速度和加速度信息。

如果只是把 Kinodynamic A* 的位置点转换为 `nav_msgs/Path`，它的动力学信息无法传递给控制器。要完整发挥它的作用，需要新增一个能够读取时序轨迹的控制器，或者让 Kinodynamic A* 独立发布自定义轨迹消息。

因此建议：

1. 第一阶段先接入普通 YAstar 和 MinimumSnap。
2. 第二阶段再评估 Kinodynamic A* 的实时性和轨迹接口。
3. 如果需要使用速度、加速度信息，再开发配套的时序轨迹控制器。

## 可以新增的功能

### 动态 Mask 障碍物

YAstar 的 `Mask` 支持多层、命名和定时失效的障碍物掩码。可以新增一个 ROS 2 节点，将以下信息转换为 Mask：

- 临时禁行区域；
- 敌方机器人或动态障碍物；
- 比赛阶段限制区域；
- 雷达或视觉产生的局部障碍物；
- 需要短时间锁定的通道。

### 方形可行区域可视化

将 `SfcSquare` 输出的方形区域发布为 RViz Marker，用于显示：

- A* 原始路径；
- 简化后的路径；
- 每个控制点对应的方形区域；
- MinimumSnap 最终轨迹；
- 碰撞检测失败的轨迹段。

### 独立 TDT 导航算法包

建议在 `srm27_sentry_ws/src` 下新增一个 ROS 2 包，例如：

```text
tdt_nav_kit/
├── include/tdt_nav_kit/
├── src/
│   ├── tdt_global_planner.cpp
│   ├── tdt_minimum_snap_smoother.cpp
│   └── tdt_mask_bridge.cpp
├── plugin.xml
├── CMakeLists.txt
└── package.xml
```

原始 TDT 算法源码可以放在该包内部，或者单独编译为库后由插件链接。

## 地图和坐标转换

这是移植中最容易出错的部分。

### YAstar 地图语义

`YAstar::setMap(width, height, mapping, originx, originy, vector<int8_t>)` 的接口中：

- `0` 表示空闲；
- 非 `0` 表示障碍物。

因此可以将 Nav2 Costmap 转成：

- 空闲栅格：`0`；
- 致命障碍、膨胀区域和未知区域：`1`。

### SfcSquare 和 MinimumSnap 地图语义

这两个模块使用相反的二值地图语义：

- `0` 表示障碍物；
- 非 `0` 表示可行区域。

推荐转换为：

- 障碍物：`0`；
- 可行区域：`255`。

### 其他转换要求

- 使用 Nav2 Costmap 的 `resolution`、`origin_x` 和 `origin_y`。
- 保证 Costmap 索引和 Eigen 行主序矩阵的行列方向一致。
- 统一使用 `map` 坐标系进行全局规划和轨迹优化。
- 机器人半径应继续由 Nav2 Inflation Layer 或预膨胀地图处理。
- 每次规划时应获取 Costmap 快照，避免规划过程中直接读取正在更新的内存。

## 依赖和编译问题

TDT 的独立 CMake 示例依赖：

- Eigen3；
- OSQP；
- OsqpEigen；
- OpenCV（主要用于示例和可视化）。

移植到 ROS 2 后不建议直接在 colcon 构建过程中运行 `scripts/setup.sh`，应在 `package.xml` 和 `CMakeLists.txt` 中声明依赖，并通过系统包、工作区依赖或已安装的 CMake 包提供 OSQP 和 OsqpEigen。

还需要注意，TDT 当前源码使用了 `std::span`、`std::ranges` 等 C++20 特性，而现有 `srm27_sentry_ws` 的部分包使用 C++17。新包应设置为 C++20，或者将这些实现改写为 C++17 兼容代码。

## 推荐实施顺序

### 第一阶段：只接入 YAstar

1. 新建 `tdt_nav_kit` ROS 2 包。
2. 加入 YAstar 源码和 Eigen 依赖。
3. 实现 `nav2_core::GlobalPlanner` 包装器。
4. 将 `planner_server` 中的 ThetaStar 改为 YAstar 插件。
5. 保留现有 Costmap、SimpleSmoother 和 OmniPid 控制器。
6. 在仿真地图上验证路径、坐标和障碍物语义。

### 第二阶段：接入 MinimumSnap

1. 将当前规划路径转换为控制点。
2. 调用 `YAstar::simplifyPath()` 减少冗余点。
3. 使用 `MinimumSnap::solve()` 生成平滑路径。
4. 实现 Nav2 smoother 插件或独立路径后处理节点。
5. 保留 OmniPid 控制器，验证跟踪效果和碰撞情况。

### 第三阶段：加入 Mask 和 SFC 调试功能

1. 添加动态 Mask ROS 接口。
2. 将方形可行区域发布到 RViz。
3. 调整安全膨胀、走廊范围、走廊收缩量和碰撞迭代次数。

### 第四阶段：评估 KinodynamicAstar

1. 测量搜索耗时和节点数量。
2. 确认当前底盘是否需要速度、加速度和时间信息。
3. 如果需要，设计自定义轨迹消息和时序轨迹控制器。
4. 再决定是否让它替代普通全局规划器。

## 最终建议

最小风险的移植结果是：

```text
ThetaStar
    ↓ 替换为
YAstar A*

SimpleSmoother
    ↓ 替换为
MinimumSnap + SfcSquare

OmniPidPursuitController、Costmap、定位、地形处理和底盘接口
    ↓ 保留
```

这样可以获得 TDT 的路径搜索、人工势场、轨迹优化和方形可行区域能力，同时不破坏 `srm27_sentry_ws` 已经具备的 ROS 2 导航基础设施。
