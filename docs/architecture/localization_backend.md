# Z-Localization backend 接入契约（实验模式）

> 状态：代码已迁入，**默认关闭**；只完成 `colcon build` / `colcon test` 级验证，
> 未做仿真与实车验收。实车验收通过前不得作为正式定位入口。
> 依据：[Z-Localization 整体迁移实施方案](../design/Z-Localization整体迁移实施方案.md)、
> [剩余部分整体迁移问题分析](../design/Z-Localization剩余部分整体迁移问题分析%28ai%29.md)。
> 最后核对：2026-11-16

本文件是 backend 模式的**接口与所有权契约**（方案 §3.2 第 4、5 条的"具体方案固定在
接口文档与测试中"）。实现位置：

| 位置 | 内容 |
|---|---|
| `src/localization/backend/srm27_zlocalization_backend/` | 上游 `src/backend` 迁入版（算法、6 个地图/标定工具、测试、文档） |
| `src/localization/backend/srm27_localization_adapter/scripts/zlocalization_backend_adapter.py` | backend 专用输入适配 + reset 服务代理 |
| `src/localization/backend/srm27_localization_adapter/scripts/zlio_bridge.py` | 持有该适配层（`backend_enabled:=true` 时启用） |
| `src/bringup/srm27_nav_bringup/launch/localization_launch.py` | 启动接线 + 冲突检查 |
| `scripts/bringup/start_real_nav.sh` / `start_sim_nav.sh` | `--zloc-backend` + `--zloc-prior` / `--zloc-loop` / `--zloc-uwb` |

## 1. 谁拥有 `map → odom`

backend 模式下 `map → odom` 的**唯一发布者**是 `srm27_zlocalization_backend_node`。
四个候选所有者互斥，任一时刻只能有一个：

| 模式 | `map → odom` 发布者 | 入口 |
|---|---|---|
| `--lio` / `--static` | `static_transform_publisher` | 默认链路，不变 |
| `--reloc` | `small_gicp_relocalization` | 不变 |
| `--slam` | `slam_toolbox` | 不变 |
| `--zloc-backend` | `srm27_zlocalization_backend_node` | **本轮新增，默认关闭** |

冲突检查在启动前失败，而不是"起两个节点抢 TF"：

- `localization_launch.py::_validate_zlocalization_backend`：backend 模式要求
  `use_lio_odometry:=True` + `odometry_frontend:=zlio`；启用先验时另需绝对路径地图包；
  与 `use_pcd_localization`（GICP）互斥；同时把静态 `map→odom` 的
  `IfCondition` 从 `not use_pcd_localization` 改成
  `not (use_pcd_localization or use_zlocalization_backend)`。
- `nav2_stack_launch.py::_validate_odometry_frontend`：backend 与 `slam:=True` 互斥。
- `start_*_nav.sh`：`--zloc-backend` 与 `--reloc/--slam/--static/--gicp` 互斥，并要求
  `--lio` + `--lio-frontend zlio`；`--zloc-prior` 开启时还要求 `--zloc-map <绝对目录>`。

backend 仍然**只出 `map→odom`**：`odom→gimbal_yaw→base_link` 那条链继续由
`sensor_scan_generation` 每帧查实时 TF 产生（云台角在这里进）。backend 的车体位姿类
输出（`~/localization_pose`、legacy `tdt_lio_*`、`map→body` TF）在本工作区**一律视为
诊断量**，配置里已关闭（见 §5）。

## 2. 两个 `odom` 必须同名同义（T_off 唯一所有者）

前端的 `FrontendFrame.odometry` 是"ZLIO 自己的世界系"位姿 `R_k`（`alignGravity()` 之后
首帧不是单位阵）。本仓库的 `odom` 由 `zlio_bridge` 定义：

```text
T_off            = T_ground · T(base_frame←lidar_frame) · R_0⁻¹
T_odom←lidar(k)  = T_off · R_k
```

backend 按 `map_from_sensor · (T_odom_sensor)⁻¹` 求 `map→odom`，所以喂给它的
`T_odom_sensor` 必须**逐位等于** `zlio_bridge` 发布的 `lidar_odometry`。

由此得到一条硬约束：**适配层必须与 `lidar_odometry` 在同一个节点里**。
如果另起一个节点自己算 `T_off`，它抓到的"第一帧"可能与 bridge 不是同一帧
（订阅建立晚几帧，或 TF 精确时刻查询失败退化成最新值），两个 odom 系会差一个常量
刚体变换 `D`，整条 TF 链稳定地错 `D` 而 `localization_state` 仍是 `NORMAL`。
所以本实现把 `BackendInputAdapter` 挂在 `zlio_bridge` 内部（`backend_enabled` 参数），
它拿到的就是当帧的 `self.t_off` 与 `pose`。

## 3. 输入契约（前端 → backend）

backend 逐帧强校验下列字段，任何一条不符就**丢弃整帧**（只有节流 WARN，不会崩）：

| 字段 | 要求 | 适配层怎么做 |
|---|---|---|
| `odometry.header.frame_id` | `common.odom_frame`（`odom`） | 固定写 `odom` |
| `odometry.child_frame_id` | `common.sensor_frame` | 写 `backend_sensor_frame`（= `zlio_bridge.lidar_frame`，实车 `livox_imu` / 仿真 `front_mid360`） |
| `cloud.header.frame_id` | 等于 `child_frame_id` | 已一致则不动；不一致时改写并只告警一次 |
| `cloud.header.stamp` | 等于 `odometry.header.stamp` | 不一致时对齐并告警一次 |
| `begin` | ≤ `odometry.header.stamp` | 原样透传 |
| `instance_id` / `epoch` / `sequence` | 前端会话身份 | 原样透传（backend 用它仲裁会话） |
| `matches` / `lidar_updated` | 诊断量 | 原样透传 |

点云**保持在前端局部（IMU/雷达）系**（`common.input_cloud_in_local_frame: true`），
适配层不做任何点云变换；backend 自己用位姿把它搬到地图系。这既符合上游语义，也避免
20 Hz 下多一次整帧变换。

### 3.1 协方差

前端位姿协方差（`zlio::poseCovariance()`）是**点约定**（point convention）：
`Cov([δp − [p]×δθ; δθ])`，δ 是世界系左扰动。左乘常量 `T_off` 后

```text
δp' = R_off·δp,      p' = t_off + R_off·p
ρ' − [p']×θ' = R_off·(ρ − [p]×θ)
```

因此适配层做 `C' = blkdiag(R_off, R_off) · C · blkdiag(R_off, R_off)ᵀ`
（位置块、旋转块、交叉块一起左乘 `R_off`），与平移 `t_off` 无关。

`zlio_bridge` 的 `pose_body_offset_xyz`（默认 0）是右乘的随体平移，等于把参考点在
odom 系里移动 `d = R_pose·offset`，适配层再用 `A = [[I, −[d]×], [0, I]]` 补这一项耦合。
数值由 `test/test_zlocalization_backend_adapter.py` 钉住。

### 3.2 话题、QoS、时间

| 方向 | 话题 | 类型 | QoS |
|---|---|---|---|
| 适配层 → backend | `frontend/frame` | `localization_interfaces/msg/FrontendFrame` | reliable，keep_last 50 |
| 适配层 → backend | `frontend/odometry` | `FrontendOdometry` | reliable，keep_last 200 |
| 适配层 → backend | `frontend/session` | `FrontendSession` | reliable，keep_last 1，transient local |
| backend → 适配层（reset） | `zlocalization/reset_frontend` | `ResetFrontend` | services 默认 |
| 适配层 → 前端（reset） | `/zlio/reset_internal` | `ResetFrontend` | services 默认 |

约定：

- 话题名是**相对名**（`backend.yaml` 的 `common.frame_topic` 等），随 namespace 解析；
  仿真里 namespace=`red_standard_robot1` 时两边自然对齐。上游默认是绝对名
  `/frontend/frame`，本工作区改成相对名并在 §5 里有测试钉住。
- session 必须 transient local：backend 晚启动也要能拿到当前 `instance/epoch`。
- 时间戳只透传不改写：`FrontendFrame` 的时间语义是**扫描末端**（前端后验时刻），
  backend 的 `sync_odom_topic` 语义与此一致。

## 4. reset / epoch 的唯一所有权

结论：**连续性补偿的唯一所有者是本仓库 `zlio_bridge` 的 `T_off` 锚定**；
backend 侧不再做第二次坐标补偿。

### 4.1 为什么必须二选一

两套机制同时开着就是同一个跳变被补偿两次：

| 机制 | 做什么 |
|---|---|
| `zlio_bridge` 的 `anchor_on_epoch_change` | 换代时 `T_off ← T_off · R_last_old · R_first_new⁻¹`，让**适配后 odom 连续** |
| backend 的 `PoseOutput::reframeOdometry` | 收到 reset 回复里的 `old_from_new` 后 `map_from_odom ← map_from_odom · old_from_new` |

若两者都做：odom 坐标已经连续，backend 再乘一个非单位边界 ⇒ 全局位姿凭空跳一个锚定增量
（问题分析 §B4），还会在因子图里留下错误的相对约束。

### 4.2 本实现的接线

```text
backend_node ──ResetFrontend──► zlocalization/reset_frontend（适配层代理）
                                     │ 转发同一事务
                                     ▼
                              /zlio/reset_internal（前端）
```

1. **前端**：backend 模式强制 `coordinated_reset:=true`（`localization_launch.py`）。
   输入中断时前端只置 `reset_required` 等后端协调，不再自己换代；否则一次断流会触发
   两次换代（问题分析 §B5）。非 backend 模式保持 `false`，行为不变。
2. **backend**：发起重置事务时把 `assume_stationary=true`（上游语义），但本链路会拿到
   有效边界，所以不依赖"重置期间停车"这条假设。
3. **适配层代理**：把前端的 `old_from_new` **改写成恒等边界**（`has_boundary=true`、
   `old_from_new` = 单位变换、`source_epoch`/`boundary_stamp` 透传）。

   恒等不是近似而是精确：bridge 已经让适配后 odom 连续，所以适配后新旧坐标关系就是恒等。
   backend 收到恒等边界后 `reframeOdometry` 是 no-op，同时走"边界有效"分支
   —— 比"假设停车"更强：移动中 reset 也不会累积误差。

   前端返回 `has_boundary=false`（冷重置）时，代理用**适配层最后一帧**的
   `epoch`/时间戳补一条恒等边界；一帧都还没发过时**不冒充**边界，让 backend 退回
   "假设停车"契约。

```text
时序（一次断流恢复）：
  前端 ─ reset_required=true ─► 适配层 ─► backend（FrontendSession 透传）
  backend ─ ResetFrontend(target=N) ─► 代理 ─► 前端 reset（warm）
  前端 ─ 回复（含真实 old_from_new / source_epoch / stamp）─► 代理
  代理 ─ 回复（恒等边界）─► backend：reframe 为 no-op、epoch=N、接受新代次首帧
  bridge ─ 新代次首帧：T_off 锚定 ─► 适配后 odom 在接缝处连续
```

### 4.3 需要注意的边界情况

- `reset_mode` 保持 `warm`；warm 失败退化成冷启动时坐标仍然连续（锚定与 warm/cold 无关）。
- reset 期间**车不需要静止**；但实车操作流程仍建议在安全状态执行 `/localization_reset`。
- 若把 `anchor_on_epoch_change` 关掉，适配层的恒等边界就不再成立：
  `zlio_bridge` 会在启动时打 ERROR（`backend_enabled=true` 需要锚定打开）。
- 多次切换模式后 backend 的 `epoch` 由它自己的 reset 事务推进，与前端 epoch 一一对应；
  不要手工改 `target_epoch`。

## 5. 配置与关闭的输出

`config/backend.yaml`（算法基线，迁入版）在本工作区的关键改动：

| 键 | 值 | 原因 |
|---|---|---|
| `common.frame_topic` / `prediction_topic` / `session_topic` | `frontend/*`（相对名） | namespace 下不能抢同名绝对话题 |
| `prior_map.directory` | `""`（必须由入口给绝对路径） | 不依赖进程 CWD |
| `trajectory.save_directory` | `""`（必须由入口给绝对路径） | 同上 |
| `prior_map.enabled` / `loop.enabled` / `uwb.enabled` | `false` | 三项由 CLI 独立选择，默认全部关闭 |
| `output.legacy.enabled` | `false` | 无消费者；且 `publish_tf` 会多发一条 `map→body` |
| `output.legacy.publish_tf` | `false` | `map` 只能有 `odom` 一个子节点 |
| `output.world_cloud.enabled` | `false` | 每帧整帧变换到 map 系是纯开销 |
| `reset.service` | `zlocalization/reset_frontend` | 走适配层代理（§4.2） |
| `services.reset` | `localization_reset`（相对名） | 随 namespace 解析 |

平台 profile 只覆盖 `common.sensor_frame`，算法参数不分叉：

| 文件 | `sensor_frame` | 用于 |
|---|---|---|
| `config/srm27_localization_real.yaml` | `livox_imu` | 实车 |
| `config/srm27_localization_sim.yaml` | `front_mid360` | 仿真 |

`backend.launch.py` 额外提供三个覆盖参数（迁入时新增）：`sensor_frame`、`odom_frame`、
`trajectory_directory`；后两个/前者为空表示沿用配置。`prior_map` 与
`trajectory_directory` **必须是绝对路径**，相对路径在 configure 阶段直接报错
（`require_absolute()`），不允许"在 A 目录能起、在 B 目录起不来"。

### 三项独立能力（2026-11-19）

`--zloc-prior`、`--zloc-loop`、`--zloc-uwb` 均默认关闭，支持全部 8 种组合。
`--zloc-map` 仅指定路径；`--zloc-config` 可覆盖平台算法配置文件（绝对路径，支持 `extends`）。
脚本总是显式传递三项布尔值，未指定的开关不会被 YAML 或同名环境变量悄悄开启。
全关时仍由 backend 发布 LIO 推算的 `map→odom`，不具备外部绝对地图约束。

先验与运行回环各持有独立的描述子库。启用回环时不再裁剪运行关键帧，并自动关闭 fixed-lag，
保留历史图变量供回环因子引用；代价是内存随轨迹增长。先验+UWB 保持联合创新门控，
回环经过描述子、几何、PCM 和图创新检验后提交到同一个图，TF 所有权不变。
普通协调 reset 保留图和两库；估计器数值故障只清运行轨迹/回环库，保留不可变先验。

UWB 的 `topic/frame`、天线杆臂、时间偏置及固定/在线坐标对齐需按场上配置核对。
当前 UWB 因子假设天线到前端传感器的杆臂固定；若跨越旋转云台关节，需要另外处理动态外参。
无先验时在线坐标/时间标定会关闭，固定对齐配置仍需有效。

## 6. 依赖：GTSAM 4.2.x

backend 的算法库硬依赖 GTSAM 4.2.x（`CMakeLists.txt` 拒绝 ≥4.3，
`package.xml` 写 `version_gte="4.2.0" version_lt="4.3.0"`）。本机没有系统级 GTSAM
且无 root 时，用仓库脚本装到用户目录：

```bash
./scripts/setup/install_gtsam_4_2.sh          # 默认装到 ~/.local/gtsam-4.2.0
# 家目录不可写时装到工作区（本机现有安装就是 .deps/gtsam-4.2.0）：
GTSAM_PREFIX=$PWD/.deps/gtsam-4.2.0 ./scripts/setup/install_gtsam_4_2.sh
source scripts/setup/env_srm27.sh             # 自动探测前缀，导出 GTSAM_DIR / LD_LIBRARY_PATH
```

构建期 `GTSAM_DIR` 指向 `.../lib/cmake/GTSAM`；运行期靠
`--disable-new-dtags` + `BUILD/INSTALL_RPATH=$<TARGET_FILE_DIR:gtsam>` 找到
`libgtsam.so.4` 与 `libmetis-gtsam.so`（同 SONAME 多版本共存时优先用构建选中的那份）。
部署记录见 `docs/records/2026-11-16-zlocalization-backend-migration.md` §5。

## 7. 先验地图包

backend 需要**关键帧地图包**（`keyframes.txt` + `clouds/` + `sc_descriptors.bin`），
不是 GICP 用的单张 PCD。生成方式（必须用地图系的那张，不能用 `camera_init` 系原始点云）：

```bash
# 1) 先按实测地面把 mapframe 点云摆正（只改 3D PCD，不动 PGM；地面落到 map z=0）
python3 scripts/mapping/level_map_pcd.py \
  --pcd maps/pcd/<site>_mapframe.pcd          # 产出 <site>_mapframe_leveled.pcd

# 2) 用摆正后的点云建包；--sensor-z 是**实测**雷达离地高度，不是 CAD 值
./scripts/mapping/build_zloc_map_bundle.sh \
  --name <site> --pcd maps/pcd/<site>_mapframe_leveled.pcd \
  --sensor-z 0.2966 --force
```

要点：

- `--sensor-z` / `--sc-lidar-height` 取**实测**雷达离地高度（本车 `gimbal_yaw` 0.2681 +
  `livox_imu` 0.0285 = 0.2966 m；不要再用 CAD 的 0.346388 → 0.3464），且必须与
  `scan_context.lidar_height` 逐位一致（`storage.cpp` 按 1e-6 比较）；环数/扇区/最大半径/
  降采样/augment 同理。不一致时 backend 启动直接
  `FATAL: failed to load Scan Context descriptors` —— 不会静默降级成低精度匹配。
- 2026-11-24 起，`219_1121` 与 `227_1113pro` 两个实车 bundle 都已按上述口径重建；
  实车 profile 的 `common.scan_context.lidar_height` 为 `0.2966`。裸 PCD 仍保留原始件，
  摆正后的是 `*_mapframe_leveled.pcd`。
- 检索参数（`exclude_recent` / `candidates` / `distance_threshold`）同样沿用
  `common.scan_context.*`，改了一个就要同步另一个。
- `clean_map_bundle` 只对**建图阶段落盘的 runtime bundle** 有意义；对
  `build_pcd_map_bundle` 生成的 bundle 不会报错，但因为没有可见性记录会 0 点被清理。
- 2D 栅格图（PGM）继续给 Nav2 用，两套地图并行维护；backend 接入**不改变**地图格式。

## 8. 回退

- backend 模式默认关闭；`--zloc-backend` 之外的所有入口行为与迁移前逐位一致。
- 现场回退：去掉 `--zloc-backend`（等价于回到 `--lio` + 静态 `map→odom`），或
  `--reloc` 用 GICP。
- 代码层回退：`srm27_zlocalization_backend` 是一个独立包，删掉/不构建它不会影响
  默认链路（`localization_launch.py` 对"包缺失"做了降级：默认链路照常启动，
  只有显式 `use_zlocalization_backend:=True` 才报错）。
- 地图回退：bundle 目录是运行时参数，与 PGM/PCD 并行；切换不需要改代码。

## 9. 已知限制（未验收项）

- 未做仿真导航链（验收 E）与实车（验收 F）；精度/CPU/内存指标尚未测。
- 先验+回环+UWB 共存已实现并有合成场景回归；现场三融合精度、UWB 标定和长时资源占用仍待验收。
- backend 输出的 `map→odom` 默认仍是 3D；2D Nav2 栈可开
  `output.planar_map_to_odom: true`，对外 TF 只保留 x/y/yaw（内部估计仍是 6-DoF）。
  2026-11-24 实车 profile 已打开：先验地图按地面摆正后，`z` 从 0.063 m 降到 -0.006 m，
  但 odom（出生时 `gimbal_yaw`）自身的 `roll/pitch≈1.4°/2.4°` 仍在；4-DoF 投影把这部分
  清掉，global/local costmap 才共面。3D 消费者（需要完整 z/roll/pitch）应保持 false。
  换新地图仍建议先跑 `level_map_pcd.py`，否则 3D 地图的 z/tilt 会进入配准。
- 云台这类"传感器相对车体运动会变"的自由度上游模型里不存在：本实现只借用
  `map→odom`（与 body 外参无关），车体位姿仍由 `sensor_scan_generation` 产生。
