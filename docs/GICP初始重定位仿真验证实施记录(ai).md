# GICP 初始重定位仿真验证 实施记录（ai）

对应方案：[`docs/GICP初始重定位仿真验证方案.md`](./GICP初始重定位仿真验证方案.md)。
本文记录「按方案实施」的产出、实测数据、当前阻塞与下一步，不重复方案本身的内容。

> 结论先行：**工具链已按方案 §9 落地并自检通过；主验收矩阵尚未取得有效结果。**
> 当前阻塞是「先验点云地图与 `registered_scan` 的配准残差过大（实测 0.26–0.59 m 中位
> 最近邻距离，GICP 报 RMSE 0.47 m）」，导致方案 §8.2 的接受门限（RMSE ≤ 0.3 m）
> 无法通过，快速筛查 7 组全部 `accepted=0`。方案里最关键的问题（当前
> `max_rotation_step=0.5 rad` 是否阻塞 ±90° 恢复）**因此还没有得到实测答案**。

---

## 1. 本轮交付物

### 1.1 被测节点：逐次诊断输出（方案 §9）

`src/srm27_navigation/small_gicp_relocalization/`

* `src/small_gicp_relocalization.cpp` / `include/.../small_gicp_relocalization.hpp`
  * 每次配准尝试输出一条 **JSON 逐次诊断**，覆盖全部分支：
    `no_scan` / `too_few_points` / `not_converged` / `non_finite` / `rejected` /
    `accepted` / `accepted_init_step` / `hypothesis_ambiguous` / `initial_pose_applied`。
    字段含源扫描时间、序号、点数、converged、iterations、inliers、inlier_ratio、rmse、
    dpos/dangle、`init_dpos`/`init_dangle`（相对**本轮实际生效初值**）、平面化位姿、
    拒绝原因、单次耗时、accepted/initialized 状态，以及多航向候选的逐候选评估。
  * 发布到 `gicp_diagnostics`（`std_msgs/String`，JSON），同时以 `[gicp_diag]` 前缀
    进日志；**accepted 日志不再节流**，方案 §2「不能从日志行数推算成功次数」得以满足。
  * `publishTransform()` 增加 `max_scan_age`（默认 3 s）过期保护：输入停发后停止
    刷新 `map → odom`，对应方案 §2/§7 的 S6。
* 初始化策略参数（**默认全部关闭，基线行为不变**，方案 §10）：
  `init_acceptance_enable` / `init_max_translation_step` / `init_max_rotation_step`
  / `init_accept_count` / `init_tracking_hold_count`、
  `init_hypothesis_enable` / `init_hypothesis_angles` / `init_hypothesis_quality_gap`
  / `init_hypothesis_rmse_margin`。
  > 这部分只实现、未经过验证：基线本来就没通过，按方案 §10 的顺序不应先放宽门限
  > （会在错误地图上放大假接受风险）。

### 1.2 验证工具包：`src/srm27_navigation/srm27_gicp_validation/`

| 工具 | 方案条目 | 说明 |
| --- | --- | --- |
| `gicp_initialization_runner` | §9 | 从 YAML 读矩阵；逐轮重启被测 GICP、发布 `/initialpose`、超时归档、写 `cases.jsonl`/`manifest.yaml` |
| `gicp_truth_evaluator` | §8 | 只读真值 + TF，按扫描时间戳对齐；算 `e_xy/e_yaw/e_z`、3 s 稳定窗、跳变、可疑 accepted、失效原因；输出 `iterations_*.csv`/`summary.json`/`summary.csv` |
| `gicp_map_recorder` + `tools/rebuild_prior_map.py` | §4.2 | 两段式建图：采集只落盘 npy，离线按真值拼接 PCD/栅格/清单（含 SHA-256、`T_M_W`、坐标契约） |
| `gicp_session_check` | §5 | 会话就绪/TF 检查（真值、`registered_scan` 有数据、`odom→base_link` 存在） |
| `gicp_eval_selftest` | §9 | 评估器离线自检（合成真值 + TF + 诊断） |
| `gicp_registered_scan_provider` | §3.2 | 本环境替代 `loam_interface`/`sensor_scan_generation`（见 §2.2） |
| `gicp_static_tf_republisher` | §5 | 周期重发 `base_link→front_mid360` 外参（10 Hz），不依赖 `/tf_static` latch |
| `gicp_cmd_vel_script` | §7 | 建图/S9 用的速度脚本（`map_slow`、`s9_motion` 等路线） |
| `tools/check_scan_map_alignment.py` | §4.2 | 地标核对工具：在 (dx,dy,dyaw) 与候选外参上评估扫描↔地图最近邻距离 |

### 1.3 会话编排脚本：`script/gicp_validation/`

* `run_session.sh` —— 一次完成「起栈 → 建图 → 试验矩阵 → 归档」。
  **必须在单个后台作业里运行**（见 §2.1）。
* `start_validation_stack.sh` —— 起 Gazebo + 点云转换 + Point-LIO + TF 扫描链。
* `run_prior_map_capture.sh` —— 采集 + 离线重建。
* `selftest_evaluator.sh` —— 评估器自检（`good`/`bad`/`fake`/`noisy`）。
* `run_alignment_check.sh` —— 扫描↔地图对齐核对。
* `diag_*.sh`、`probe_scan.sh` —— 排查过程留下的诊断脚本，可复现本文 §2 的每个结论。

### 1.4 已验证的事实（有实测日志支撑）

* **评估器自检通过**（方案 §8.2 的判据确实按预期工作）：

  | 场景 | 期望 | 实测 |
  | --- | --- | --- |
  | `good`（误差 0.05 m / 1°） | 通过 | `passed=true`，`e_xy_max=0.05`，稳定完成 t0→0.25 s |
  | `bad`（误差 1.0 m） | 失败 | `passed=false`，`observation_timeout`，`suspect_count=282` |
  | `fake`（误差 5 m / 30°，RMSE 0.02） | 失败 | `passed=false`，`suspect_count=243`，逐例 `可疑 accepted` 记录 |

* **GICP 逐次诊断可用**：每次尝试都有完整 JSON，能直接区分
  「优化未收敛」「收敛但被步长拒绝」「收敛但被 RMSE/内点率拒绝」。
* **先验地图产物可用**：`tmp/gicp_validation/promap.pcd`（0.05 m 体素，2 144 403 点）、
  `promap.pgm/.yaml`、`promap_manifest.yaml`（`T_M_W`、哈希、坐标契约、轨迹范围）。
  轨迹跨度 x=6.46 m、y=0.15 m，与 `map_slow` 路线一致。

---

## 2. 环境层阻塞与已确认的根因

这些都不是工程代码的缺陷，但每一条都会让「按方案的默认启动方式」直接失败。

### 2.1 sandbox 的执行模型（最关键）

本环境每次 bash 调用都是**独立的 PID/挂载命名空间**（`/proc`、`/tmp`、`/dev/shm` 各自独立）：

* 一次调用结束后，其中启动的子进程**全部结束**——所以「Gazebo 常驻 + 多次调用跑测试」
  的常规做法不可行，必须把整场试验放进**同一个**后台作业（`run_session.sh` 就是这么做的）。
* `/dev/shm` 各自独立 ⇒ Fast DDS 的**共享内存传输跨调用完全不通**：同一调用内一切正常，
  跨调用一个消息都收不到（实测：跨调用订阅 `registered_scan`/`chassis_odometry_gt` 均为 0 条）。
  因此曾改用 UDP-only profile —— 但那又让同调用内的可靠交付变差（runner 收不到数据），
  最终选择的方案是：**默认传输 + 所有节点在同一调用内**。

### 2.2 上游感知节点在本环境不可用，已用替代实现

| 上游节点 | 症状 | 根因 | 本工程的处理 |
| --- | --- | --- | --- |
| `loam_interface` | `registered_scan` 一条都不发；`TF lookup failed: "base_link" ... does not exist` 刷屏 | 它在收到首帧里程计时用**带时间戳**的 `lookupTransform(base_link, front_mid360, 扫描时间)` 做一次性初始化；本环境下该查询持续失败（`/tf_static` 的 latch 送不到晚加入的 tf2 监听者） | 用 `gicp_registered_scan_provider` 自己完成「雷达系点云 → odom 系」；外参 TF 由 `gicp_static_tf_republisher` 以 10 Hz **周期**发布，不再依赖 latch |
| `sensor_scan_generation` | `odom→base_link` 一条都不发 | 它用 `message_filters` 的 **ExactTime** 同步 `lidar_odometry` 与 `registered_scan`，本环境两者时间戳相差约 1 s，回调永不触发 | 由同一个 `gicp_registered_scan_provider` 直接发 `odom→base_link` / `odom→front_mid360` |

> 这两处替代都在文件头注释里写明了「与方案 §3.2 的差异」。它们的几何用的是
> `srm27_sentry_geometry.yaml` 里已确认的外参（`xyz=[0.15,-0.15,0.22]`、
> `rpy=[-0.0698,0,-π/2]`），因此不引入新的几何假设；但**正式报告必须标注：本轮
> `registered_scan` 与 `odom→base_link` 不是由方案指定的上游节点提供的**。

### 2.3 其它必须知道的环境事实

* **Gazebo 需要软件渲染**：`Sensors` 系统默认用 Ogre1，在没有 GPU 的环境里
  `RenderUtil::Init()` 段错误。改用 `<render_engine>ogre2</render_engine>` 的**世界副本**
  （`tmp/gicp_validation/worlds/rmuc_2025_ogre2.sdf`，由脚本生成，不改仓库文件）后正常，
  实测实时率 RTF=1.00、雷达约 19 Hz。
* **Gazebo 必须显式停**：`kill` 掉 `ros2 launch` 后 `ign gazebo` 会变成孤儿，下一次会话会
  出现重复的时钟/物理世界与重复 TF 发布者。预检里用
  `ign service -s /server_control --reqtype ignition.msgs.ServerControl --req 'stop: true'` 收尾。
* **`/tf` 话题只能有一种消息类型**：往 `/tf` 直接发单条 `TransformStamped` 会让话题出现
  `[TransformStamped, TFMessage]` 两种类型，所有 tf2 订阅者静默收不到（实测报
  `contains more than one type`）。本工程一律发 `TFMessage`。
* **`ros2 launch` 找不到本包的控制台脚本**：本机 setuptools 把 `ament_python` 的脚本装到
  `<prefix>/bin` 而不是 `lib/<pkg>/`，因此 `validation_stack.launch.py` 里用
  `ExecuteProcess` + 绝对路径启动。
* 用户 shell 里的 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` 曾经让 `/tf_static` 的 latch 送不到
  `loam_interface`；改用周期重发外参后该约束不再关键，但 `env.sh` 里保留了说明。

---

## 3. 当前唯一的功能性阻塞：扫描↔地图配准残差过大

### 3.1 现象

快速筛查（结构丰富区、位置无偏差、yaw = 0/±30/±60/±90° 各 1 次）**7 组全部
`accepted=0`，`failure_reason=stopped_without_success`**。逐次诊断显示：

```json
{"seq":18,"source_points":8535,"converged":true,"iterations":37,"accepted":false,
 "branch":"rejected","reason":"rmse","inlier_ratio":0.373638,"rmse":0.474256,
 "dpos":0.590482,"dangle":0.001208,"result_x":-0.589489,"result_y":-0.03422,
 "result_yaw":-0.001208,"duration_ms":194.5}
```

即：**优化是收敛的、算子几乎不动（yaw 误差 0.001 rad ≈ 0.07°），但 RMSE 0.474 m
超过门限 0.3 m，内点率 0.374 刚过 0.3**。yaw=0 时就是如此，说明这不是 ±90° 的问题，
而是**基线本身就没对齐**。

### 3.2 独立的对齐核对（`tools/check_scan_map_alignment.py`）

* 用**同一份采集数据**（记录帧按真值变换到地图系）与地图做最近邻：
  * 外参候选全部试过：`I / R_off / t_off / T_L_B / T_B_L / R_off⁻¹`，
    最好的是 **`I`（0.26–0.59 m 中位最近邻，取决于地图抽稀粒度）**，
    其余候选 0.38–3.25 m。**结论：GICP 加载地图时施加的外参方向是对的（等价于恒等），
    不是外参把地图搞偏了。**
* `(dx,dy,dyaw)` 粗搜索最优也只有 0.65 m 中位最近邻（1.0 m 抽稀地图），
  yaw 扫描从 −180° 到 +180° 呈单调无峰形态 —— 说明两张图**不是简单的刚体错位**，
  而是「同源但残差偏大」。

### 3.3 尚存的可能原因（下一步按此顺序排查）

1. **地图与扫描的采集时刻不一致导致的地图模糊**：地图是机器人**运动**中 1175 帧
   按真值拼接的，`max_odom_gap=0.05 s` 下地面/立面的多次观测仍有约 0.1–0.3 m 的重影
   （0.05 m 体素下 2.14 M 点，说明同一表面被多帧反复填格）。
   → 试验：整场静止（不发布速度）单点采集重建成图，再看残差；或按「每格取最近观测」重建。
2. **`registered_scan` 的稀疏化**：`gicp_registered_scan_provider` 目前
   `velodyne_points` 只抽 1/4（`point_stride=4`）后再截到 12000 点，`source_points≈8530`。
   GICP 的源点云过稀会让内点率与 RMSE 同时变差。
   → 试验：`point_stride=1`、`max_points` 提到 60000，观察 RMSE 与耗时。
3. **地图 z 覆盖范围**：当前地图包含 z 到 20 m 的高结构（场地高墙），而方案 §1 本轮
   只评价平面 `x/y/yaw`，GICP 是三维配准，高结构的少量错位会主导 RMSE。
   → 试验：把地图裁剪到 z ∈ [−0.5, 3] 再评估残差。
4. **Gazebo 雷达噪声**：`noise_stddev=0.01 m`，量级不足以解释 0.47 m，可最后排除。

---

## 4. 复现方式

```bash
# 一次性环境（每次会话独立 ROS_DOMAIN_ID）
cd /home/srm/srm_nav_27

# 评估器自检（不需要 Gazebo，先验证判据本身）
bash script/gicp_validation/selftest_evaluator.sh good
bash script/gicp_validation/selftest_evaluator.sh bad
bash script/gicp_validation/selftest_evaluator.sh fake

# 整场会话：建图（若缺）+ 快速筛查 + 归档
#   —— 必须在一个后台作业里跑完，进程不能跨调用存活
bash script/gicp_validation/run_session.sh screen

# 只看对齐残差（不跑 GICP）
bash script/gicp_validation/run_alignment_check.sh
```

产物：

* 逐轮诊断 `/home/srm/srm_nav_27/tmp/gicp_validation/results/logs/gicp.log`
* 逐次记录 + 判定 `tmp/gicp_validation/results/<run_id>/iterations_*.csv`、`summary.json`、`cases.jsonl`
* 地图清单 `tmp/gicp_validation/promap_manifest.yaml`
* 会话日志 `tmp/gicp_validation/logs/session_screen.log`

---

## 5. 与方案的差异与未完成项（必须写进最终报告）

### 5.1 已实施但与方案有差异

| 方案条目 | 本轮做法 | 影响 |
| --- | --- | --- |
| §3.2 `registered_scan` 由 `loam_interface`、`odom→base_link` 由 `sensor_scan_generation` | 由 `gicp_registered_scan_provider` 统一提供 | 被测链路的上游实现与方案不同；GICP 本身未改 |
| §5 用 `nav_srm_simulation_launch.py use_pcd_localization:=True` 起链路 | 用验证专用 `validation_stack.launch.py`（只含点云转换/LIO/TF） | 少启动 Nav2/地形分析，避免无关负载；参数文件仍是 `nav2_params_srm.yaml` 的副本 |
| §6.8 每轮重启 GICP 并清空评估器缓存 | 每轮重启 GICP；**Gazebo 与 LIO 保持常驻** | 只能覆盖「运行中重置初值」口径；方案要求的**完整冷启动**（从进程启动计时）尚未做 |
| §7 S1 共 945 次 | **未执行** | 基线未通过，按方案 §7「快速筛查失败时先定位原因，不立即启动全量批测」 |
| §4.1 地图原点 | 取出生点即地图原点，`T_M_W = inverse(T_W_B(spawn))` | 与方案一致，已写入清单 |

### 5.2 未完成

* S0 回归、S1 全量矩阵、1 m 对角线补测、完整冷启动（S8）
* S3–S7、S9 扰动与失效测试
* 参数实验（§10）与参数定版
* 方案 §11 要求的最终报告（成功率热力图、耗时分布、误差曲线、低 RMSE 错误匹配案例）
  —— 这些都要等基线对齐问题解决后才有意义

---

## 5.3 一键启动入口：`script/start_sim_nav.sh --gicp`

方案 §2 记录过「`start_sim_nav.sh` 固定 `use_pcd_localization:=False`，不能用它证明
GICP 生效」。本轮给该脚本加了 `--gicp` 开关，把这件事补上：

```bash
# 用 GICP 初始重定位（先验点云 + 自动推导 map->odom 初值）
./script/start_sim_nav.sh --gicp \
    --prior-pcd /abs/prior.pcd \
    -m /abs/prior.yaml \
    --no-rotation

# 手动指定 map->odom 初值（x,y,z,roll,pitch,yaw），用于人为注入初值误差
./script/start_sim_nav.sh --gicp --prior-pcd /abs/prior.pcd \
    --init-pose "0.5,0,0,0,0,0.5236" --no-rotation
```

行为：

* `--gicp` → `use_pcd_localization:=True use_lio_odometry:=False`，并把
  `prior_pcd_file` 传给 launch；不再启动 `simulation_ground_truth_odometry`，
  `map→odom` 只由 `small_gicp_relocalization` 发布（静态 TF 不再发）。
* 先验点云清单（`*_manifest.yaml`）存在时，按
  **`T_M_O_init = T_M_W · T_W_B(spawn)`** 自动推出 `init_pose`，并把它与 `run_id`
  写进一份临时参数副本（`init_pose` 是 GICP 节点参数，launch 入口没有为它开
  launch 参数，直接改副本最省事且可追溯）。建图起点与出生点一致时结果就是恒等，
  不一致时正好给出让机器人出现在正确位置所需的初值。
* 清单里记录的建图出生点与 `srm_sim.yaml` 的 `robot.x/y/yaw` 不一致时会**告警**——
  这正是「两张图不同原点」的典型症状。
* `--init-pose` 显式给定后不再从清单推导。
* 新增环境变量：`USE_GICP PRIOR_PCD_FILE PRIOR_PCD_MANIFEST INIT_POSE GICP_RUN_ID
  PRIOR_PCD_SEARCH_DIRS`。

已验证：`DRY_RUN=1 ./script/start_sim_nav.sh --gicp --prior-pcd <promap.pcd>` 能正确
推出 `init_pose=0,0,0,0,0,0` 并把参数写进副本；不加 `--gicp` 时行为与改前一致。
**注意：只验证了命令拼装（dry-run），没有在 Gazebo 里跑通整条 GICP 链路**——
因为 §3 的配准残差问题还没解决。

## 6. 下一步建议（按优先级）

1. **先修对齐残差**（§3.3 的 1–3 条），把静止单点的扫描↔地图中位最近邻压到
   ≲0.05 m 量级，并让一次配准的 RMSE 稳定 ≲0.1 m。这是所有验收的前置条件。
2. 基线通过后，**立刻做航向快速筛查**（`quick_screen.yaml`，7 组 × 3 次），
   用逐次诊断区分「优化没找到正确解」与「找到但被 `max_rotation_step` 拒绝」——
   这正是方案 §2 结尾要求实测的那个问题。
3. 再按方案 §11 的顺序推进 S0 → S1 → S3–S9，并补完整冷启动。
4. 报告里必须标注 §5.1 的链路差异与 §2.1/§2.2 的环境替代，否则结论不可复现。
