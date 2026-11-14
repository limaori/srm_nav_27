# 把 Z-Localization（zlio）剩余部分整体迁入本仓库的问题分析

> 本文是**分析**，不是方案，没有改动任何代码。
> 输入：`/home/srm/z-localization`（上游，GPL-2.0-only）与本仓库 `/home/srm/srm_nav_27` 的现状。
> 结论先行：见 §6「一页结论」。

---

## 0. 先把"剩余部分"钉死（事实基线）

上游 `z-localization/src/` 共 5 个包。本仓库**已经**迁入 2 个（`frontend`、`localization_interfaces`），
所以"剩余部分"只有 3 个包：

| 上游包 | 规模 | 本仓库现状 | 迁入后新增什么 |
| --- | --- | --- | --- |
| `src/backend` | 122 文件 / 22 927 行（含 `include` `src` `apps` `tools`）+ 3 260 行测试，1.9 MB | **无** | 7 个可执行：`backend_node` + 6 个地图/标定工具；1 个核心库 `backend_core` |
| `src/bringup` | 4 文件 | **无** | 一个"一键起前后端"的 launch，只包含 `frontend`/`backend`/`livox_ros_driver2` 三个 include |
| `src/livox_ros_driver2` | 1.1 MB | **已有同名包**：`src/srm27_navigation/livox_ros_driver2` | 无（迁不进来，见 §1 的 A1） |

已迁部分的迁法可以参考：[frontend/CMakeLists.txt:4–8](../../src/frontend/CMakeLists.txt#L4-L8) 记着当时的改造——
"警告规则、算法配置与授权文件都在仓库根目录（`../../cmake`、`../../config`、`../../LICENSE`），
并入本工作空间后改为**包内自持**"。backend 有一模一样的问题（且更严重，见 §1 的 A3）。

同时要认清一件已经写进本仓库文档的事实：
[仿真LIO前端替换Z-LIO实施方案(ai).md:627](仿真LIO前端替换Z-LIO实施方案%28ai%29.md#L627)
与 `:639` 已经把"不用 `src/backend/`、`src/bringup/`、`config/backend.yaml`"列为**边界**，
实施记录 `:63` 也重复了这条。也就是说，这次要做的是**主动撤销一个已经做过的取舍**，
而不是补一个漏掉的步骤。这条会决定下面所有成本判断的基准。

---

## 1. A 类：不改就进不来（构建 / 安装硬失败）

### A1 `livox_ros_driver2` 同名包已存在，迁入即重名

本仓库已经有 `src/srm27_navigation/livox_ros_driver2`（v1.1.0），且 `frontend`、`point_lio`、
`srm27_nav_bringup` 三个包都 `depend` 它。colcon **不支持两个同名包**，上游那份必须整份跳过。

这不是新发现——实施记录里已经写明"本仓库 v1.1.0 已满足，`msg/` 两边 `diff -r` 为空，
同名包无法共存"（[仿真LIO前端替换Z-LIO实施记录(ai).md:63](../records/仿真LIO前端替换Z-LIO实施记录%28ai%29.md#L63)）。
`msg/` 那份 `diff -r` 我也复核过了：**确实完全一致**。

但"跳过上游那份"不等于"没有差异"。实际 diff 出来的两处差异，正好都是 `bringup` 会踩到的：

| 项 | 上游 `src/livox_ros_driver2` | 本仓库 `src/srm27_navigation/livox_ros_driver2` |
| --- | --- | --- |
| launch 目录名 | `launch_ROS2/` | `launch/` |
| `msg_MID360_launch.py` 的 `xfer_format` | `1`（只发 `livox/lidar` CustomMsg） | `4`（`kAllMsg`：`livox/lidar` 发 CustomMsg，**另加** `livox/lidar/pointcloud` 发 PointCloud2） |

后果很具体：上游 `bringup.launch.py` 里写死的是

```python
PathJoinSubstitution([FindPackageShare('livox_ros_driver2'), 'launch_ROS2', 'msg_MID360_launch.py'])
```

在**本仓库里 `launch_ROS2/` 不存在**，于是 `driver:=true` 这条分支是**运行期 launch 报错**
（找不到 launch 文件），不是构建错误。而本仓库那份 `xfer_format = 4` 是上游的超集，
所以只要把路径改成 `launch/`，`input_type:=livox` 仍然成立。
**一句话：`bringup` 必须改一行路径，否则 `driver:=true` 永远起不来；这个错误不会在 `colcon build` 暴露。**


### A2 GTSAM 4.2.x：当前机器上没有，且版本窗是硬的

- `find_package(GTSAM 4.2 CONFIG REQUIRED)` + `if(GTSAM_VERSION VERSION_GREATER_EQUAL 4.3) FATAL_ERROR`
  （`src/backend/CMakeLists.txt`）；`package.xml:36` 又写了 `version_gte="4.2.0" version_lt="4.3.0"`。
- **本机未安装**（`find / -name GTSAMConfig.cmake` 无结果）。在装之前 `colcon build` 100% 失败。
- 好消息：apt 源里有 `ros-humble-gtsam 4.2.0-3jammy`（已验证 `apt-cache policy` 可见），
  所以不必源码编译 30~60 分钟。**但这仍是给整机加了一个新的大依赖**，且它是 ABI 敏感的 C++ 库：
  一旦以后要升级/降级，`backend_core` 全部重编。
- `CMakeLists.txt` 还特意加了 `--disable-new-dtags` + `BUILD_RPATH/INSTALL_RPATH` 指向
  `$<TARGET_FILE_DIR:gtsam>`，说明上游踩过"同 SONAME 多版本互相覆盖"的坑。本机若同时存在
  系统装的 GTSAM 和 apt 装的 `ros-humble-gtsam`，这个 RPATH 技巧未必仍然够用。

### A3 仓库根相对路径，`install()` 会直接失败

`src/backend/CMakeLists.txt` 有三处假定自己住在 `z-localization/src/<pkg>`：

| 行 | 内容 | 迁到 `src/xxx/backend` 后 |
| --- | --- | --- |
| `:7` | `include(.../../../cmake/Warnings.cmake)` | 找不到 `localization_enable_warnings()` → **configure 阶段就报错** |
| `:170` | `install(FILES ../../config/backend.yaml ...)` | `../../config/` 不存在 → **install 阶段报错** |
| `:176` | `install(FILES README.md ../../LICENSE ...)` | 同上 |

这三处必须像 `frontend` 那样改成包内自持（把 `Warnings.cmake` 拷进包内、`backend.yaml` 挪进
`config/`、`LICENSE` 拷进包内）。**这是可控的**，但它是"抄过来直接编"一定会撞的第一堵墙，
而且失败点在 configure/install 而不是编译，容易被误判成环境问题。

### A4 包名过于通用，且 `bringup` 与现有 `srm27_bringup` 语义重叠

`backend` 和 `bringup` 都是没有前缀的通用名。虽然与 `/opt/ros/humble` 和本工作空间现有 48 个包
**都不重名**（已核对），但：

- 本仓库既有 `srm27_bringup/launch/bringup.launch.py`，再加一个 `bringup/launch/bringup.launch.py`，
  `ros2 launch bringup bringup.launch.py` 与 `ros2 launch srm27_bringup bringup.launch.py`
  只差一个前缀，是典型的现场踩坑点（现场改命令行时少打一个前缀就起了另一条链）。
- `backend` 这种名字在长期演进里几乎必然与将来的包重名。本仓库其余 48 个包要么带
  `srm27_` 前缀，要么来自 `dependencies.repos` 里固定的上游仓库，只有这次的
  `backend`/`bringup` 两个名字是无前缀的裸词。

### A5 许可证：GPL-2.0-only

上游整仓 GPL-2.0-only，`backend`/`bringup` 的 `package.xml` 和每个源文件都是。本仓库**根目录没有 LICENSE**，
已迁的 `frontend` 是把上游 `LICENSE` 拷进包内处理的。

迁 backend 意味着：又 2.3 万行 GPL 代码进入本仓库，并且（如果真按上游设计让它接管 `map→odom`）
它就成了运行在产品上的核心组件。**这一条不是技术阻塞，是需要队内明确决策的事**——
尤其是这份代码将来是否要对外发布、是否要闭源分发。前端那次的体量（692 KB）与这次（1.9 MB，
且是导航定位主链路）不是一个量级。

---

## 2. B 类：能编过、能起，但语义是错的（最危险的部分）

这一节是本文的重点。A 类的错误都会"当场报错"，而 B 类**不会报错**，只会让车出现在错的位置上，
或者让某个上游节点悄悄不干活。本仓库文档里反复出现的教训就是这个形态
（[仿真LIO前端替换Z-LIO实施方案(ai).md:103–121](仿真LIO前端替换Z-LIO实施方案%28ai%29.md#L103-L121)
的"C1/C2/C3 三条契约"就是为此写的）。

### B1 `map→odom` 从"1 个发布者"变成"2 个发布者抢同一条 TF"

这是最直接、最必然发生的一条。

本仓库 `map→odom` 的发布者由运行模式决定，**任一时刻只有一个**：

| 模式 | `map→odom` 发布者 | 出处 |
| --- | --- | --- |
| `--lio` / 实车默认 | `tf2_ros static_transform_publisher`（2D 静态恒等或 `--init-pose`） | [localization_launch.py:340–352](../../src/srm27_navigation/srm27_nav_bringup/launch/localization_launch.py#L340-L352) |
| `--gicp` | `small_gicp_relocalization`（20 Hz，平面化） | `src/small_gicp_relocalization.cpp:724` |
| `--slam` | `slam_toolbox` | `start_real_slam.sh` 文件头第 6 步 |

而 `backend` 一启动就会自己发 `map→odom`：

- `apps/publisher_state.cpp:151` —— `tf_broadcaster_->sendTransform(map_frame → odom_frame)`
- 而且默认还会**另外**发一条 `map→body`：`apps/publisher_legacy.cpp:75`，由
  `config/backend.yaml` 的 `output.legacy.publish_tf: true` 打开。`body` 是本仓库 TF 树里
  **不存在**的帧名。

后果：`map` 有 `odom` 和 `body` 两个直接子节点、`odom→base_link` 仍由 `sensor_scan_generation` 发，
RViz 与 Nav2 会看到两个互不相关的全局位姿源。**必须显式关掉静态 `map→odom` 与 `small_gicp`**，
否则就是 `start_real_slam.sh` 文件头警告的那个场景（"两个节点抢同一条 TF，地图会来回跳"）。

### B2 `odom` 同名不同义：后端算出的 `map→odom` 会差一个 `T_off`（★ 最隐蔽的一条）

这是本次迁移**最容易漏、且一定会在实车/仿真上表现为"起步姿态整体歪掉"**的一条。

两个 `odom` 不是同一个坐标系：

1. **ZLIO 自己的世界系**。`src/frontend/src/initialization.cpp` 的 `alignGravity()` 会把世界系旋转到
   重力朝 `-Z`，于是首帧位姿 `R_0` **不是单位阵**。仿真里雷达装 30° roll，`R_0` 就带 30°。
   前端 `world_frame` 参数被本仓库填成 `"odom"`（[localization_launch.py:309](../../src/srm27_navigation/srm27_nav_bringup/launch/localization_launch.py#L309)），
   **但只是贴了个标签**，几何上它不是 `odom`。
2. **本仓库的 `odom`**，由 `zlio_bridge` 定义：`T_odom←lidar(k) = T_off · R_k`，
   `T_off = M · R_0⁻¹`（`M = TF(base_link ← lidar)`，首帧查询），所以 `T_odom←lidar(0) = M`，
   即 **`odom` 的原点/轴向 = 出生时的 `base_link`**
   （[zlio_bridge.py:45](../../src/srm27_navigation/srm27_nav_bringup/scripts/zlio_bridge.py#L45)、`:534`、`:603`）。

两者相差 `T_off`：`T_odom←zlodom = T_off`。

`backend` 的 `map→odom` 是**相对前端自己的 odom 求的**：
`src/pose_output.cpp:53` —— `out.T_map_odom = map_from_sensor_ * T_odom_sensor.inverse();`，
而 `T_odom_sensor` 就是 ZLIO 的 `R_k`。也就是说：**它输出的是 `map→zlodom`，却写在 `odom` 这个名字上。**

于是 TF 链变成 `map --(T_off 的错误重复)--> odom --(bridge 的 T_off)--> base_link`，
`T_off` 被乘了两次。仿真里这正是
[仿真LIO前端替换Z-LIO实施方案(ai).md](仿真LIO前端替换Z-LIO实施方案%28ai%29.md) §4.2 记录过的
"odom 系整体被绕一根水平轴转了约 30°"，只是换了一种触发方式——
而且**这一次没有任何节点会报错**，`backend` 会认为定位 `NORMAL`。

**修法不是改一个参数**：要么让前端把 `world_frame` 改成 `zlodom` 之类（但 backend 只有一个
`odom_frame` 参数，会把 `map→zlodom` 写到 `odom_frame` 上，问题原样保留），要么新增一个
"后端专用前端适配层"，把 `R_k` 左乘 `T_off` 之后再喂给 backend。**这是一层新代码，不是拷贝能解决的。**

### B3 云台：`body_from_sensor` 在 backend 里是常量，在本车上必须是时变量

`backend` 把"传感器位姿 → 车体位姿"建模成**固定外参**
（`config/backend.yaml` 的 `common.body.rotation.*_deg` / `common.body.offset.*`，
读入处 `src/backend/apps/parameters.cpp:65-73`），并且提供 `body_calibration_node` 来标定这组常量。

本仓库的车**雷达挂在 yaw 云台上**，`sensor_scan_generation` 的做法是**每帧查实时 TF**
`base_link ← livox_imu`：

- [nav2_params_srm_minco_fakeyaw.yaml:158–164](../../src/srm27_navigation/srm27_nav_bringup/config/real/nav2_params_srm_minco_fakeyaw.yaml#L158-L164)
  的 `sensor_scan_generation.lidar_frame: "livox_imu"` / `base_frame: "base_link"`，
- 实车入口还专门起 `gimbal_joint_state_relay` 把关节角回传进 TF
  （`nav_srm_real_launch.py:371-382`）。
- 实施方案 §2.2 已经把这条写成核心设计："`ZLIO` 只能替第 ① 层，替不了第 ③ 层，
  而 ③ 恰恰是云台场景唯一正确的地方"。

所以 backend 输出的**车体位姿类话题全是错的**（错在云台角这个量级上，不是噪声量级）：

- `~/localization_pose`（`PoseWithCovarianceStamped`）
- `/tdt_lio_pose` / `/tdt_lio_twist` / `/tdt_lio_path`
- `map→body` TF
- 协方差的杠杆臂传播（`apps/publisher_state.cpp:172` 的 `propagateLeverArmCovariance`）

**注意边界**：`map→odom` 本身**不受**这条影响（它是 `map_from_sensor · T_odom_sensor⁻¹`，
与 body 外参无关）。这是本文给出的少数"可以救"的点之一。
但只要有人拿 backend 的车体位姿去替换 `sensor_scan_generation`，就是上面那种错法。

### B4 重置：bridge 的"换代锚定"与后端的"reset 事务"会做两次补偿

现在这条链上只有**一个**补偿机制：

- `zlio_bridge` 的 `anchor_on_epoch_change`（默认 `True`）：换代时
  `T_off ← T_off · R_last_old · R_first_new⁻¹`，保证 `odom` 连续
  （[`zlio_bridge.py:292`](../../src/srm27_navigation/srm27_nav_bringup/scripts/zlio_bridge.py) 与 `:603`）。

而 `backend` 自带一套完整的换代/重置协议：`FrontendSession` 的 `instance/epoch/reset_required/failed`、
`/localization_reset`（`config/backend.yaml` 的 `services.reset`）、`/zlio/reset_internal`
（`localization_interfaces/srv/ResetFrontend`，带 `transaction_id` / `target_epoch`），
以及"暖重置前后位姿由后端衔接"的假设（`backend` README 的服务表：

> 手动调用 `/localization_reset` 时，若前端无法提供暖重置前后的坐标变换，后端会按机器人保持静止来衔接位姿。

两套机制同时开着，就是**同一个跳变被补偿两次**：`odom→base_link` 因为 bridge 锚定而连续，
而 `map→odom` 被后端又挪了一次，结果是**全局位姿凭空跳一个锚定增量**。
`backend` 还会按"重置期间车静止"的假设记错相对约束，因子图里留下一个错误边。

至少要做到：二选一（关 `anchor_on_epoch_change`，或者不启用 backend 的重置协调），
并明确"谁拥有换代锚定"这件所有权问题。**上游两者都认为自己拥有。**

### B5 `coordinated_reset` 必须从 `false` 翻成 `true`，翻了之后的连锁反应

现在写的是（[`localization_launch.py:311-314`](../../src/srm27_navigation/srm27_nav_bringup/launch/localization_launch.py)）：

```python
# warm 重置保留速度/倾角/零偏；coordinated_reset 必须 false —— 本链路没有后端，
# 前端需要自己在输入中断后换代（bridge 的锚定正是为此准备）。
{"reset_mode": "warm"},
{"coordinated_reset": False},
```

`coordinated_reset` 的语义（`src/frontend/apps/node.cpp:331-350`）是：
输入中断后，`true` → 只把 `reset_required` 置起来等后端来协调（`:346-348`）；
`false` → **前端自己给自己发一次 `ResetFrontend` 并直接换代**（`:349-353`）。

也就是说，一旦把 backend 接进来，这个开关**必须翻成 `true`**，否则：

- 前端自己换代 → 后端看到一个**未经过事务的 epoch 变更**。
- 后端的 `FrontendSession::observe()`（`apps/frontend_session.cpp:29-46`）把它记为新的观测，
  而 `needsConfirmation()`（`:171-175`，判据是"观测到的 epoch ≠ active 的 epoch"）随之变真，
  `pendingAction()`（`:19-25`）于是返回 `ResetRequired`——
  也就是说**后端会再发起一次 `ResetFrontend` 事务**，把前端重置到它自己的 `target_epoch` 上。
  结果是"一次断流 = 两次换代"，而每一次换代都会被 `zlio_bridge` 锚定一次（见 B4），
  且事务挂起期间后端的全局输出是被挂住的。
- 反向的失败也在：README 明说"普通前端处理失败会使后端**冻结输出**，等待显式重置或新前端进程接入，
  不会自动反复重启"。所以只要前端有一次没按后端的预期换代，就得人工 `/localization_reset`。
- 一句话：**掉几帧雷达就可能在"两次锚定"和"冻结等人工"之间二选一。**

而翻了 `true` 又牵出 B4（两套补偿）和一条新的现场流程要求：**每次自动恢复期间车必须真正静止**
（backend 的衔接假设），这与"哨兵在场上一直动"的使用方式直接冲突——
上游 `docs/design.md:114` 自己也承认"比赛中定位飞掉重启时，云台可能还在跟随自瞄转动，几乎不能保证静止"。

### B6 话题 / QoS / 帧名契约与现网对不上（都是"静默不工作"）

| backend 期望（默认值） | 本仓库现状 | 后果 |
| --- | --- | --- |
| `common.frame_topic: /frontend/frame`（**绝对名**） | 前端被 remap 成 `zlio/frame`（`localization_launch.py:319`） | 后端一帧都收不到，静默无输出 |
| `common.prediction_topic: /frontend/odometry` | remap 成 `zlio/frontend_odometry`（`:320`） | 同上 |
| `common.session_topic: /frontend/session` | remap 成 `zlio/session`（`:321`） | 后端拿不到 `instance/epoch`，`FrontendSession` 无法闭合 |
| `common.sensor_frame: imu` | 仿真 `front_mid360`，实车 `livox_imu`（`zlio_body_frame`） | `node_input.cpp:54-57` 会逐条校验 `odom.child_frame_id == sensor_frame` 且 `cloud.header.frame_id == child_frame_id`，**不匹配直接丢帧并刷错误日志** |
| `common.odom_frame: odom` | 见 B2 | 语义错位 |
| `common.body.frame: body` | TF 树里没有 `body` | 见 B1 |

另外 `zlio_bridge` 也在订阅同一个 `zlio/frame`——迁入 backend 后**两个消费者抢同一个观测流**，
这不是冲突，但意味着"bridge 和 backend 都从同一份 `FrontendFrame` 各自推出一个世界位姿"，
B2 的 `T_off` 差异会一直存在，且很难在现象上区分是谁错了。

### B7 3D 的 `map→odom` 撞上 2D 的 Nav2 与地形链路

backend 的 `map→odom` 是完整 3D（`pose_output.cpp` 里 `T_map_odom` 是 4×4）。本仓库整条消费侧
都是**平面假设**：

- `sensor_scan_generation` 对 `odom→base_link` 做平面化 + 抖动抑制（`publish_tf` 输出 2D）；
- `small_gicp_relocalization` 的 GICP 结果"会压平为 `z=roll=pitch=0`"
  （[GICP初始重定位仿真验证方案.md](GICP初始重定位仿真验证方案.md) §1）；
- 规划/代价地图/MPC 全部在 2D 上。

后果不算致命（Nav2 会把 3D 变换投影掉），但会带来两个真问题：

1. `base_link` 在 `map` 里带上 roll/pitch 之后，`terrain_analysis(_ext)` 的地面判据、
   `fake_vel_transform` 的速度分解都会吃到额外的俯仰/横滚分量。
2. `z` 方向不再恒为 0，而本仓库的地图（PGM）与 `pcd_to_mapframe.py` 的整条对账流程
   都只有 `(yaw, dx, dy)` 三个自由度。

### B8 输出面大面积空转

backend 默认打开了一大批没有消费方的输出（`config/backend.yaml`）：

| 输出 | 默认 | 本仓库有消费方吗 |
| --- | --- | --- |
| `/localization/odometry` | 恒发 | 否（已 `grep` 确认） |
| `/localization/lio_world_cloud` | `enabled: true`，`publish_period: 0.0`（**每帧**） | 否 |
| `/tdt_lio_pose` `/tdt_lio_twist` `/LAM_danger_situation` | `output.legacy.enabled: true` | 否（已 `grep` 确认无 `LAM_danger`/`tdt_lio`） |
| `~/odom_path` `~/optimized_path` `~/loop_edges` `~/map_axes` `~/prior_map` `~/runtime_map` | 关 | — |
| `~/localization_state` | 恒发（transient local） | 否 |

最值得注意 `world_cloud`：`enabled: true` + `publish_period: 0.0` = **每一帧都把整帧点云变换到 map 系再发一遍**，
在本仓库这条本就吃紧的 10~20 Hz 链路上是纯增加的 CPU 与 DDS 带宽。

---

## 3. C 类：能跑对，但要付钱（地图 / 运维 / 工程成本）

### C1 地图体系分裂：一套 PGM 一套 PCD bundle，`map` 的原点定义不同

本仓库的地图是 **slam_toolbox 产出的 2D PGM + posegraph**
（`maps/227_1011/` 下有 `.pgm/.yaml/.posegraph/.data`；`scripts/bringup/start_real_slam.sh` 的"存图"一段写明
`/slam_toolbox/save_map` 与 `serialize_map` **各写一半**，两个合起来才是完整地图），
点云是 point_lio 的 `camera_init` 系 `scans.pcd`，再由 `scripts/mapping/pcd_to_mapframe.py`
求一个平面变换烘到地图系。

`backend` 要的是**关键帧地图包**：`keyframes.txt` + `clouds/` + 描述子数据文件
（`backend` README 的"地图与标定工具"）。两者是**不同的东西**：

| | 本仓库现网 | backend 需要 |
| --- | --- | --- |
| 规划用图 | `*.pgm` + `*.yaml`（2D 栅格） | 不消费 |
| 定位用图 | `*_mapframe.pcd`（small_gicp 用） | `map_bundle/`（关键帧 + 扫描上下文描述子） |
| `map` 原点 | PGM 的 `origin` | PCD bundle 的任意原点 |

**这一条比它看上去轻**：`build_pcd_map_bundle --input <pcd> --out-bundle <dir> --x-min ... --sensor-z ...`
是**从一张全局 PCD 反推关键帧与描述子**的（`tools/build.cpp:41-53` 的参数表），
所以理论上可以拿现成的 `maps/pcd/227_1011_mapframe.pcd` 直接生成 bundle，只要
`--sc-lidar-height` / `--sensor-z` / 描述子配置与定位时一致。
**前提是 `map` 原点两边一致**——而 `pcd_to_mapframe.py` 已经保证了
"PGM 与 PCD 同源同系"，所以这条路是通的。但反过来说：
**bundle 的 `map` 必须由 `_mapframe.pcd` 生成，不能用 `camera_init` 系的原始 `scans.pcd`**
（差 63°/239°，`pcd_to_mapframe.py` 文件头与 `nav2_params_srm_minco.yaml:341` 都记着这个坑）。

### C2 建图流水线要重做，而且其中一步是纯手工

上游 README 的完整流水线是 6 步，其中第 4 步无法自动化：

```
建图(mapping) → save_map → clean_map_bundle(自动清残影)
→ CloudCompare 手工收尾（去残影 / 调平 / 对齐世界原点）
→ build_pcd_map_bundle(网格生成 bundle + 描述子) → localization
```

而 `clean_map_bundle` 需要的是**建图时落盘的 runtime bundle**（README："保留完整地图包，
后续清理需要其中的关键帧和局部点云"），不是一张 `map.pcd`。
也就是说想用"自动清残影"就必须**用 ZLIO 前端重新建一次图**，不能拿现成的 session。

对一场 RMUC 的适应性训练来说，这意味着"改地图"的成本从
`start_real_slam.sh --save <name>` 一条命令，变成"重新跑一遍建图 + 跑工具 + 开 CloudCompare 手修 +
再跑工具 + 再对账 PGM"。**这是本次迁移里最大的、且最容易被低估的运营成本。**

### C3 依赖装机

| 依赖 | 现状 | 备注 |
| --- | --- | --- |
| GTSAM 4.2.0 | **缺** | apt 可装，见 A2 |
| nanoflann / TBB / OpenMP / PCL / yaml-cpp | 齐 | 已核对 |
| `message_filters` `tf2_eigen` `pcl_conversions` `launch_testing_ament_cmake` | 齐 | 已核对 |
| `livox_ros_driver2` | 已有同名包 | 见 A1 |

所以"装依赖"这一项的绝对成本不高，主要是 GTSAM 的**版本锁定**会在未来持续施压。

### C4 相对路径以进程 CWD 为基准，运行产物会落到奇怪的地方

`config/backend.yaml`：

- `prior_map.directory: pcd/rmuc`（注释明写"相对路径以进程工作目录为基准"）
- `trajectory.save_directory: pcd/runtime_zlio`

如果照抄 default 起节点，`prior_map` 会相对**当前 shell 的 cwd** 解析。`backend.launch.py`
还会在 `prior_map.enabled` 且目录不存在时**直接抛异常**。本仓库的所有入口都是
`scripts/*.sh` + launch，cwd 是仓库根；迁入后必须全部改成绝对路径，否则就是
"在 A 目录能起、在 B 目录起不来"。

### C5 测试体量与 CI 时间

`src/backend/test/CMakeLists.txt` 里 12 个 gtest + 2 个 pytest + 2 个 launch_test，
其中 `backend_localization_test` / `backend_relocalization_test` 的 `TIMEOUT` 是 120 s。
本仓库现在的测试是 `srm27_nav_bringup` 下几个纯 Python 的 launch/接线测试（秒级）。
迁入后 `colcon test` 的量级会明显变化，而且这些测试**只覆盖上游自己的契约**——
对本仓库"三层 TF 分层"的契约一条都不覆盖（现状那几条由
`test_lio_frontend_wiring.py` / `test_cli_parity.py` 守着）。

### C6 上游已停更，2.3 万行外来代码的维护成本

上游 README 自述"大概率不会再更新了，除非有重大 bug"。迁入 2.3 万行 + GTSAM 因子图代码，
意味着**这部分由本队自己维护**。而它内部有明确的设计耦合（epoch 事务、地图缓存预算、
ScanContext/G-PROBE 双描述子、UWB 在线标定、Schur 条件数退化检测），
一旦需要为"云台"或"小陀螺"改动，改动面会落在因子图和配准里，不是外围。

---

## 4. D 类：与现有设计哲学直接冲突（要不要迁的根本问题）

### D1 两套架构对"谁拥有 `map→odom`"的回答是相反的

| | 本仓库（四层分层） | Z-Localization（前后端） |
| --- | --- | --- |
| `odom→base_link` | `sensor_scan_generation`，**每帧查实时 TF** 过云台关节、平面化 | 前端高频传播（传感器系） |
| `map→odom` | 静态恒等 / small_gicp / slam_toolbox，**接近恒等** | 后端因子图**持续优化** |
| `map` 原点约定 | 与 `odom` 同源（= 出生点的 `base_link`），见实施方案 §1.1 的 C3 | PCD 地图原点（CloudCompare 定） |
| 云台 | 设计核心（第 ③ 层） | 不存在这个概念 |

实施方案 §1.1 那条"`--lio` 分支里 `map→odom` 必须保持恒等"的约定
（`scripts/bringup/start_sim_nav.sh` 里就是这么注释的），与 backend"map→odom 是估计量"是
**互斥的两套世界观**。迁 backend 不是"多一个节点"，是**换一层的主人**。

### D2 上游对"刚性安装"的隐含假设贯穿始终

`body_from_sensor` 常量（B3）、ScanContext 的 `lidar_height`（`common.scan_context.lidar_height: 0.35`）、
`build_pcd_map_bundle` 的"固定高度 + 水平姿态采样"（README 第 5 步：
"本例按水平姿态、固定高度生成"）、`level_with_odom_roll_pitch`——
这些都在假设"雷达相对车体是固定外参"。
`docs/design.md:126` 提到"云台转动"时指的是**整机在动**，不是"雷达相对底盘在转"。

换句话说：**上游不是"没适配云台"，而是它的模型里没有这个自由度。**
本车要接，必须自己补一层"用实时 TF 把传感器位姿换成车体位姿"，而不是改参数。

---

## 5. 如果还是要迁：三档路线

> 这一节只给分档和最小改动集，不写实施步骤（那是下一份文档的事）。
> 排序原则：**先拿零风险的部分，再考虑高风险的部分**。

### 档 0（推荐先做的第一件事）：只迁工具，不迁节点 —— 运行期零风险

只迁 `tools/` 的 6 个可执行（`build_pcd_map_bundle`、`clean_map_bundle`、`transform_map_bundle`、
`filter_map_bundle`、`rebuild_scan_context_descriptors`、`body_calibration_node`）+
`backend_core`，**不迁 `apps/`**（也就没有 `backend_node`、没有 TF 发布、没有话题冲突）。

- 收益：拿到"从 PCD 生成地图包 / 变换地图包 / 清残影 / 重建描述子"这套离线能力，
  可以现在就拿 `maps/pcd/227_1011_mapframe.pcd` 试跑，**完全不影响任何现网链路**。
- 成本：仍然要 GTSAM（`backend_core` 本身就链 gtsam）+ A3 的路径改造。
- 判据：能在本仓库 `colcon build` 出这 6 个可执行，并用现有 PCD 生成一个自洽的 bundle
  （`rebuild_scan_context_descriptors` 跑通、描述子能自比对）。

这一档能把 A2/A3/A5 全部验完，而 B1~B8 一个都不会触发。

### 档 1：只迁 `backend`，不迁 `bringup` / `livox_ros_driver2`

`bringup` 的价值只在于"一键起前后端"，而本仓库的入口是 `scripts/*.sh` + 分级 launch，
再加一层通用名的 `bringup`（A4）收益很低、混淆成本很高。
`livox_ros_driver2` 迁不进来（A1）。所以这一档是"迁 backend 时的默认形态"。

**必须做的适配（超出"拷贝"的部分）：**

1. **新增后端专用前端适配层**——解决 B2。把 bridge 的 `T_off` 应用到 `FrontendFrame`
   的位姿与点云上，再发到 `backend` 期望的话题名。**这一层不可避免**，否则 B2 必现。
2. **关掉静态 `map→odom` 与 `small_gicp`**——解决 B1。
3. **关掉 `output.legacy.*`（尤其 `publish_tf`）**——解决 B1 的第二条 TF 与 B8 的空转。
4. **关掉 `output.world_cloud.enabled` 或把 `publish_period` 调到 ≥0.2 s**——解决 B8 的带宽。
5. **`coordinated_reset: true` + 决定 anchor 所有权**——解决 B4/B5，并重写
   `localization_launch.py:311-314` 那段注释（它现在是**反向**的）。
6. **话题/帧名全量对账**——解决 B6，`sensor_frame` 要按仿真/实车分别给 `front_mid360` / `livox_imu`。
7. **明确"车体位姿仍由 `sensor_scan_generation` 出"**——解决 B3。backend 的车体位姿输出
   一律视为诊断量，不允许进导航。

做完这 7 条之后，backend 实际承担的角色缩水成：**"提供 `map→odom` 的全局重定位与回环"**。
那就该回头问一句：这件事现在 `small_gicp_relocalization` 已经在做，
差距是"需要粗略初值（≤1 m / ≤90°）"vs"全自动全局重定位 + 回环 + UWB"。
**这个差距值不值得上面 7 条 + C2 的建图流水线，是这次决策的真正问题。**

### 档 2（若只是想验证算法）：用 `common.frontend: legacy` 做 PoC

`backend` 支持 `common.frontend: legacy`——不依赖 ZLIO 的 `FrontendFrame`，
而是订 `cloud_registered` + `/Odometry_scan_end`（`config/backend.yaml` 的 `common.odom_topic` /
`common.sync_odom_topic` / `common.cloud_topic`，`parameters.cpp:44-48`）。

本仓库的 point_lio 发的正好是前一半名字，但**不是全部**（`point_lio/src/laserMapping.cpp:413,420`）：

| backend legacy 默认 | point_lio 实际 | 处理 |
| --- | --- | --- |
| `cloud_registered` | `cloud_registered` | 同名，直接可用 |
| `/Odometry_scan_end` | `aft_mapped_to_init` | **必须 remap** |

这一档的价值是：可以**在不碰前端、不碰 bridge、不碰 `coordinated_reset` 的前提下**，
先把"后端能不能在这张图上定位"验掉。代价有两条要确认：
① 点云须在 odom 系（`input_cloud_in_local_frame=false`）——point_lio 的 `cloud_registered`
确实在世界系，但那个"世界"是 `camera_init`，与 `odom` 之间正是 `pcd_to_mapframe.py` 在处理的
那个平面变换；② `aft_mapped_to_init` 的时间戳是帧首还是帧尾，与 `sync_odom_topic` 的
"scan_end" 语义是否一致。**换句话说，`legacy` 档省掉的恰恰是 B2/B5，但会把它们以
"坐标差一个 yaw、时间差一帧"的形式搬回来，只是这次更容易在数据上看出来。**

---

## 6. 一页结论

| # | 问题 | 类别 | 能不改就过吗 | 会不会报错 |
| --- | --- | --- | --- | --- |
| A1 | `livox_ros_driver2` 重名 | 构建 | 否（整份跳过） | 会（colcon 重名 + `bringup` 的 `launch_ROS2/` 路径运行期报错） |
| A2 | GTSAM 4.2.x 缺失 + 硬版本窗 | 构建 | 否（apt 装） | 会（configure） |
| A3 | `../../cmake`、`../../config`、`../../LICENSE` | 构建 | 否（改包内自持） | 会（configure/install） |
| A4 | 包名 `backend`/`bringup` 过于通用 | 工程 | 是 | 不会 |
| A5 | GPL-2.0-only | 合规 | 需决策 | 不会 |
| **B1** | **`map→odom` 三/四个发布者** | **架构** | **否** | **不会（TF 打架）** |
| **B2** | **`odom` 同名不同义，`T_off` 被乘两次** | **架构** | **否（要新适配层）** | **不会（姿态整体歪）** |
| B3 | 云台 vs 固定 `body_from_sensor` | 架构 | 否 | 不会（车体位姿错） |
| B4 | 换代锚定被补偿两次 | 架构 | 否（二选一） | 不会（全局位姿跳） |
| B5 | `coordinated_reset` 必须翻转 | 架构 | 否 | 不会（一次断流 = 两次换代 / 冻结等人工） |
| B6 | 话题/帧名/QoS 契约错位 | 集成 | 否 | 不会（静默丢帧） |
| B7 | 3D `map→odom` vs 2D 消费侧 | 集成 | 部分 | 不会 |
| B8 | 输出面空转（`world_cloud` 每帧） | 性能 | 否（关掉） | 不会 |
| C1 | 地图体系分裂 | 运维 | 部分（`_mapframe.pcd` 可复用） | 不会 |
| C2 | 建图流水线重做 + CloudCompare 手工 | 运维 | 否 | 不会 |
| C3 | 其余依赖已齐 | — | 是 | — |
| C4 | CWD 相对路径 | 集成 | 否（改绝对路径） | 会（launch 抛异常） |
| C5 | 测试体量 | 工程 | 是 | — |
| C6 | 上游停更 + 2.3 万行外来代码 | 工程 | — | — |

**一句话**：迁 `bringup`/`livox_ros_driver2` 没有意义（A1/A4）；迁 `backend` 的**构建**问题都是
"一天内能解决"的量级（A2/A3），**真正的问题是 B 类**——尤其是 **B1（TF 所有权）** 和
**B2（`odom` 语义 + `T_off`）** 这两条，它们决定了"拷贝过来的 backend 会不会在没有任何报错的情况下
把车定位到一个歪掉的世界里"。这两条都需要**新写一层适配代码 + 明确的所有权划分**，
不是配置能解决的。

如果目的是"让定位更稳"，档 0（只迁离线工具）先落地、档 2（`legacy` 前端 PoC）验证算法，
再决定档 1，是风险最低的顺序。
