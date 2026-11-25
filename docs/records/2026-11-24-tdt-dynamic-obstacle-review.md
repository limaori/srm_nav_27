# TDT 实时避障与限时动态掩码方案核对记录

- 日期：2026-11-24
- 仓库：`/home/srm/nav_ws/srm_nav_27_tdt`
- 核对对象：YAstar vendor、`TdtGlobalPlanner` wrapper、terrain 点云、实车默认配置及 MINCO 障碍输入链路。
- 核对方式：本地源码检查，以及直接编译当前 vendor 的独立 C++ 最小验证。
- 本次范围：记录方案核对结果；未修改导航实现、未启停导航栈、未进行本轮仿真或实车闭环测试。
- 版本口径：以核对时的工作区为准，包含已有的未提交动态掩码实现，不能只按 Git HEAD 或历史 README 判断接入状态。

## 1. 结论

限时动态掩码是 YAstar 可以使用的一种实时障碍表示方式，值得继续完善。但“实时障碍只能经限时 Mask 进入”“超过有效期自动消失”“加约 200 行接入后即可实现比赛级避障”等判断，均超出了当前证据。

当前工作区的主要问题是：实车默认参数中的全局、局部实时障碍层均关闭；已有的 wrapper 动态掩码也默认关闭。即使只启用 YAstar 私有掩码，MINCO 的局部 ESDF 与轨迹校验仍然看不到这部分障碍。

后续实现应让经过可靠筛选的实时障碍同时进入全局规划和局部碰撞校验，并统一坐标、时间、净空和失效处理语义。仅增加一个限时接口调用，不能形成完整的动态避障能力。

本轮只核实了本地 kit 的接口能力与本仓库接线，未检查东大整车比赛栈。不能据此断言其比赛时采用的完整感知、规划、控制方案。

## 2. 实时障碍已有的输入路径

`TdtGlobalPlanner::createPlan()` 调用 `makeSnapshot()`，读取的是 Nav2 合成后的全局代价地图，再转换为 YAstar 的 `occMap`。因此快照本身并不等于“只有静态地图”。

当 `intensity_voxel_layer` 启用且正常标记障碍时，已有链路可以是：

```text
terrain_map_ext（odom）
  → global_costmap.intensity_voxel_layer
  → global_costmap 合成结果
  → makeSnapshot / convertCostmapCells
  → YAstar occMap
  → 全局路径
```

转换器按 `cost > occupancy_threshold` 判断占用，并单独处理未知格。当前实车 TDT 注入参数为 `occupancy_threshold=252`，所以正常进入合成 costmap 的 253/254 障碍代价可以被规划器识别。

核对时，默认实车参数文件 `nav2_params_srm_minco_fakeyaw.yaml` 中：

| 对象 | 配置状态 | 源码位置（核对时行号） |
|---|---|---|
| 局部 `intensity_voxel_layer` | `enabled: false` | 约 1053 行 |
| 全局 `intensity_voxel_layer` | `enabled: false` | 约 1126 行 |
| wrapper 动态掩码 | `dynamic_mask_topic: ""`，不创建订阅 | `nav_srm_real_launch.py` 约 323 行 |

这里记录的是默认配置及 launch 注入逻辑，未读取正在运行的节点参数。launch 允许替换参数文件，且 `use_tdt_planner` 默认值为 `False`；实际运行链路需要结合启动命令核对。

源码依据：

- [Costmap 快照与占用转换](../../src/tdt_nav_kit/src/costmap_converter.cpp)：`convertCostmapCells()`、`makeSnapshotFromCostmap()`。
- [TDT 实车参数注入](../../src/bringup/srm27_nav_bringup/launch/nav_srm_real_launch.py)：`RealNavParams.perform()`。
- [默认实车参数](../../src/bringup/srm27_nav_bringup/config/real/current/nav2_params_srm_minco_fakeyaw.yaml)：`local_costmap`、`global_costmap`。

## 3. vendor 限时掩码的实际语义

### 3.1 到期清理由下一次同名更新触发

`YAstar::setMaskMapTimed()` 遍历同一 `name + "@"` 前缀的历史层，比较其更新时间与本次传入的 `duration`，清空超时层并复用槽位。

时间戳使用 `std::chrono::steady_clock`，是掩码写入时刻，不是点云消息中的 ROS 时间戳。

`Mask::masked()`、`YAstar::isInObstacle()` 与 `getMap()` 没有按有效期清理的逻辑，也没有独立定时器。因此：

- 持续调用同名 `setMaskMapTimed()` 时，会清理已超时历史层。
- 停止调用后，仅等待时间流逝不会清掉旧掩码。
- 需要清理且本轮无障碍时，也应有明确更新或清除动作，不能只在“有障碍点”时更新。
- `maxCount` 满后会覆盖最旧层，因此实际历史长度同时受有效期、容量和更新频率影响。

### 3.2 `maskValue` 必须与编码一致

接口默认 `maskValue=0`，表示输入矩阵中等于 0 的位置是障碍。

如果 wrapper 生成的矩阵采用“0 为自由、1 为障碍”，则应显式传入：

```cpp
planner.setMaskMapTimed(mask, "dynamic", duration, 16, /*maskValue=*/1);
```

原方案最前面的示例显式传了 `1`，该编码是匹配的；实施步骤中省略后续参数的调用，只有在输入使用默认的“0 为障碍”编码时才正确。

### 3.3 存储容量与限时接口限制需要区分

底层 `Mask` 的存储支持 `64 × num_layers` 个位层，但限时接口的新增层判断仍写为：

```cpp
mask.size() < sizeof(unsigned long long int) * 8
```

它没有乘上配置的 `num_layers`。本轮设置 2 个存储层、`maxCount=80`，连续写入 65 个不同位置，最终只有 64 个位置保持占用。因此不能把底层存储容量直接等同于该限时接口可用的历史层容量，也不能仅凭函数存在就认定边界逻辑已完整验证。

源码依据：[yastar.cpp](../../src/tdt_nav_kit/vendor/src/YAstar/yastar.cpp) 的 `setMaskMapTimed()`（约 452 行）、`Mask::masked()` 与 `isInObstacle()`；[yastar.hpp](../../src/tdt_nav_kit/vendor/src/YAstar/yastar.hpp) 的接口默认参数。

## 4. wrapper 当前已经存在动态掩码实现

核对开始时，以下文件已有未提交改动：

- `src/tdt_nav_kit/include/tdt_nav_kit/tdt_global_planner.hpp`
- `src/tdt_nav_kit/src/tdt_global_planner.cpp`
- `src/tdt_nav_kit/package.xml`
- `src/bringup/srm27_nav_bringup/launch/nav_srm_real_launch.py`

已有实现包含点云订阅、TF 查询、z 带筛选、栅格化、圆形膨胀、时间窗缓存，以及规划前的掩码叠加。

`createPlan()` 每次创建局部 `YAstar planner`，并重新调用 `setMap()`、`setMaskNumLayers()`。因此 vendor 内部的历史层无法跨 `createPlan()` 保留；单纯在新实例上调用限时接口，不能形成跨规划的 0.8 s 记忆。

当前 wrapper 的设计是：

```text
点云回调
  → 坐标变换、筛选、栅格化与膨胀
  → dynamic_cells_ 保存各帧格子与接收时刻

createPlan
  → 清理缓存中的超时帧
  → 合并时间窗内格子
  → setMaskMap(mask, "dynamic", 1)
  → initCostMap()
  → 搜索
```

时间窗记忆由 wrapper 承担，普通 Mask 只对本次搜索有效。这与当前局部 YAstar 实例生命周期相容。规划入口也会清理超时缓存，所以该实现的断流后下一次规划行为，与第 3 节中单独使用 vendor 限时接口的行为不同。

源码依据：[tdt_global_planner.cpp](../../src/tdt_nav_kit/src/tdt_global_planner.cpp) 的 `createPlan()`（约 240 行）、`dynamicCloudCallbackImpl()`、`applyDynamicMask()`（约 441 行）。

### 4.1 当前实现仍需处理的具体问题

- **筛选不足**：回调主要按原始 z 带过滤，尚无最小观测距离、自反射过滤、地形高度字段筛选及点数确认。
- **时间语义**：缓存使用接收时刻；旧点云或上游重复发布的历史点会被重新赋予保留窗口。
- **TF 语义**：当前查询最新变换，仅使用 yaw 和 x/y 平移，保留原始 z。需要明确 map/odom 是否允许这种平面假设，以及是否应按消息时间做完整变换。
- **地图几何**：缓存为行列索引；地图原点、尺寸或分辨率变化后，需要清理缓存或重新投影，并保证读取几何的一致性。
- **校验一致性**：端点吸附、`pathIsFree()` 和简化后校验仍读取原始 `snap`，尚未统一使用叠加动态掩码后的占用结果。
- **统计口径**：`painted` 对不同帧的同一格重复计数，不代表去重后的占用格数。

launch 注释记录了此前启用后出现问题，并引用“11999/12720 格、94%”的说法；本轮未复核该现场事件。由于当前 `painted` 统计会跨帧重复计数，仅凭该日志不能推出 94% 的唯一栅格被占用。应先统计合并掩码的非零格数，再分析覆盖率。

## 5. 时间窗不能单独解决墙点抖动

“单次误检在停止刷新后可以消退”成立；“墙点抖动会在 0.8 s 内自动自愈”缺少必要前提。

当前方案将时间窗内各帧障碍取并集。如果同一段墙在多帧中持续产生偏移点，旧误检尚未清掉，新误检已补入，自由带可能持续变窄。延长保留时间还可能加厚墙体或留下运动障碍的历史轨迹。

此外，`terrain_map_ext` 是经过地形处理和历史累积的点云，并非纯粹的本帧新增障碍集合：

- 发布坐标系为 `odom`。
- 外扩部分的 `intensity` 被赋值为 `abs(point.z - estimated_ground_z)`，表达离地高度。
- 近处合并来自局部 terrain 点云的数据。
- 上游已有衰减和缓存，旧点重新出现在输出消息中，会被下游按接收时刻再次保留。

因此需要结合 z、离地高度、观测距离、车身遮挡和时序确认筛选障碍，并避免把已有墙点无差别再次膨胀。“只保留新增障碍”也需考虑定位误差，避免误删靠近静态墙体的真实障碍。

源码依据：[terrainAnalysisExt.cpp](../../src/planner/frontend/terrain_analysis_ext/src/terrainAnalysisExt.cpp) 的历史点筛选、地面估计、离地高度赋值（约 504 行）与发布逻辑（约 534 行）；现有 [IntensityVoxelLayer](../../src/planner/frontend/srm27_costmap_plugins/src/layers/intensity_voxel_layer.cpp) 已包含高度、intensity 和距离筛选。

## 6. 几何净空与 MINCO 输入必须一起处理

### 6.1 0.15 m 不能直接认定为物理车身半径

核对时默认实车参数中的 costmap `robot_radius=0.15`，但 MINCO `safety.robot_radius=0.245`、`clearance_margin=0.0`；配置注释记录的 CAD 包络约为 0.2743 m，并解释了 ESDF 保守距离补偿。

这些数值属于不同层的判据，不能直接互换。本轮未测量车体尺寸，CAD 数值仅按已有配置注释记录。

仅对动态障碍膨胀 0.15 m，再靠软代价把路径推远，不能保证路径与真实车体保持足够物理间距。应明确真实包络、定位和跟踪误差、栅格保守量，统一全局硬占用与局部净空校验的含义。

### 6.2 YAstar 私有掩码不会自动进入 MINCO

MINCO 的数据链是：

```text
local_costmap
  → CostmapAdapter::snapshot()
  → GridSnapshot
  → Esdf2D::build()
  → 路径净空调整、MINCO 优化、TrajectoryValidator
```

`CostmapAdapter` 从局部 costmap 读取原始代价，以 `LETHAL_OBSTACLE` 为占用种子，并按配置处理未知和内切膨胀格。它不读取 YAstar 实例中的 `dynamic` Mask。

因此，只启用 wrapper 动态掩码而保持局部实时障碍层关闭，会出现全局规划看见新障碍、局部优化与校验看不见的情况。优化轨迹可能切回障碍区域，突发进入路径的障碍也缺少对应局部碰撞约束。

“全局路径已绕行 + 1.0 m/s”只能作为受控实验条件，不能替代一致的障碍输入、碰撞复验和停车能力验证。速度是否可用还取决于感知、重规划和控制延迟，以及实测制动距离。

源码依据：

- [costmap_adapter.cpp](../../src/controller/srm27_minco_controller/src/costmap_adapter.cpp)：快照与占用转换（约 109 行）。
- [minco_mpc_controller.cpp](../../src/controller/srm27_minco_controller/src/minco_mpc_controller.cpp)：`refreshMap()`（约 906 行）。
- [planning_worker.cpp](../../src/controller/srm27_minco_controller/src/planning_worker.cpp)：`optimizer.setEsdf()` 与 `validator.validate()`。
- [实车安全参数](../../src/bringup/srm27_nav_bringup/config/real/current/nav2_params_srm_minco_fakeyaw.yaml)：`FollowPath.safety`（约 917 行）。

## 7. 本轮最小运行验证

直接使用当前 `yastar.cpp`、`yastar.hpp` 与本机 Eigen 编译独立程序，不依赖 ROS 节点。程序使用自由底图，显式构造掩码后读取 `getMap()`。

| 验证项 | 构造方法 | 实际结果 | 说明 |
|---|---|---|---|
| 停止更新后的到期行为 | 中心格写入障碍，duration 为 5 ms，等待 25 ms 后直接读取地图 | 中心格仍为障碍 | 到期不会自行清理 |
| 下一次同名调用清理 | 上一步之后提交同名、全自由掩码，显式 `maskValue=1` | 原中心格恢复自由 | 清理由后续调用触发 |
| 默认编码 | 3×3 矩阵中心为 1、其余为 0，省略 `maskValue` | 8 格为障碍，中心自由 | 默认 0 是障碍 |
| 限时层容量 | 65×1 自由图，预分配 2 层，`maxCount=80`，依次标记 65 个不同位置 | 最终 64 格为障碍 | 限时接口未使用全部 128 位层容量 |

程序输出：

```text
blocked_after_ttl_without_update=1
blocked_after_empty_update=0
default_mask_value_blocked_cells=8
default_mask_value_center_blocked=0
timed_active_cells_with_128_layer_storage=64
```

编译方式如下，其中 `probe.cpp` 是上述最小构造程序，生成在本机临时目录：

```bash
g++ -std=c++17 -O0 -pthread \
  -I /usr/include/eigen3 \
  -I src/tdt_nav_kit/vendor/src/YAstar \
  /tmp/tdt-mask-review.XXXXXX/probe.cpp \
  src/tdt_nav_kit/vendor/src/YAstar/yastar.cpp \
  -o /tmp/tdt-mask-review.XXXXXX/probe
```

上面的 `XXXXXX` 表示 `mktemp` 生成的临时目录后缀，命令用于记录编译方式。该验证证明接口行为，未覆盖完整参数边界、ROS TF/QoS、运行时性能或实车避障能力。

## 8. 后续实施建议与验收内容

建议先完善障碍输入，再选择接入形式：

1. 核对启动命令和运行参数，确认实际启用的规划器、参数文件及实时障碍层状态。
2. 明确 terrain 点云字段、时间戳、历史点保留方式和 TF 约定；补齐地面、自反射及噪点筛选。
3. 采用统一的障碍有效期与失效处理，让全局规划和 MINCO 局部 ESDF/碰撞校验都看到新增障碍。可以完善 costmap 层，也可以由共享障碍缓存分别生成两侧输入。
4. 若保留 wrapper Mask，统一端点吸附、搜索、简化与最终路径校验的占用数据，处理地图几何变化、缓存清理和唯一占用格统计。
5. 统一物理包络与栅格净空判据，再确认动态膨胀半径；避免独立沿用 0.15 m 作为物理半径。

实施后需要验证：

| 场景 | 要确认的行为 |
|---|---|
| 新增桌子或障碍物 | 全局路径绕行，局部 ESDF 可见，优化轨迹保持物理净空 |
| 障碍移走、空点云、点云断流 | 清除行为符合约定；感知失效时的继续行驶或停车策略明确 |
| 障碍突然进入已有路径 | 旧轨迹被及时复验、拒绝或制动，不能只等下次全局规划 |
| 墙点持续抖动和窄走廊 | 唯一占用格、可通行带宽及净空可量化，窗口并集不持续堵路 |
| 点云延迟、TF 缺失或定位修正 | 不使用错时、错位障碍继续生成有效轨迹 |
| 地图原点、尺寸或分辨率变化 | 缓存不会被当作新地图中的同一行列继续使用 |
| 速度与制动 | 按实测延迟和制动距离确认可用速度，而非固定将 1.0 m/s 视为保障 |

上述内容为待实施与待验收项，本轮未将其标记为已完成。
