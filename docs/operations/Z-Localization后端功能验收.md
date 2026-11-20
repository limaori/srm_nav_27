# Z-Localization Backend 功能验收

> 适用范围：Z-Localization backend 已迁入，输入数据链路（ZLIO → adapter → backend）默认正常。本手册只验 backend 功能和它对导航的定位输出，不重复检查雷达驱动、IMU 单位、逐点时间、话题 QoS 等接线项。
>
> 建议先在仿真验收，再做实车低速验证。backend 模式目前是实验模式；未通过本手册前，不作为正式比赛定位链路。

## 1. 验收目标

本轮要证明 backend 能完成以下工作：

1. 加载与运行环境匹配的先验地图包，并从 `INITIALIZING` 建立可信全局位姿。
2. 移动过程中保持定位，接受合理的地图校正，不产生错误的大幅跳变。
3. 对错误地图或不支持的几何观测拒绝建立/维持可信位姿。
4. 执行前后端协调 reset 后，能完成新 epoch 初始化并恢复可信输出。
5. backend 是唯一的 `map → odom` 发布者；Nav2 消费的定位结果与 backend 一致。

§2–§8 保留单先验定位的验收流程。场上三融合使用 §9：先验、运行回环、UWB 可同时启用；单先验测试不替代三融合验收。

## 2. 验收准备

先选一个地图包，要求它与仿真世界或实车场地一致。不能只因为 PCD 文件存在就用它：地图 bundle 必须从对应地图系的 `*_mapframe.pcd` 生成，地图原点要与 Nav2 栅格图一致。

本项目已验证的地图输入是 `maps/pcd/227_1113pro_mapframe.pcd`。若当前仿真世界不是该场地，请改用与仿真世界一致的 PCD 和对应地图；不要用错误场地地图做正常定位测试。

首次生成 bundle 时执行：

```bash
cd ~/srm_nav_27
source scripts/setup/env_srm27.sh     # ROS + install/ + GTSAM（自动探测前缀）

export SITE=227_1113pro
export PCD="$PWD/maps/pcd/${SITE}_mapframe.pcd"
export BUNDLE="$HOME/srm_nav_27_data/zlocalization/map_bundles/${SITE}"
mkdir -p "$(dirname "$BUNDLE")"

ros2 run srm27_zlocalization_backend srm27_zlocalization_backend_build_pcd_map_bundle \
  --input "$PCD" --out-bundle "$BUNDLE" \
  --x-min -8 --x-max 4.5 --y-min -7.6 --y-max 10.1 \
  --sensor-z 0.3464 --query-step 2.0 --bundle-step 1.5 \
  --map-voxel 0.15 --scan-voxel 0.15 --sc-lidar-height 0.3464
```

检查地图包基本文件并保存校验值：

```bash
test -s "$BUNDLE/keyframes.txt"
test -d "$BUNDLE/clouds"
find "$BUNDLE" -maxdepth 2 -type f -printf '%P %s bytes\n' | sort
sha256sum "$PCD" "$BUNDLE/keyframes.txt"
```

`keyframes.txt`、`clouds/` 和 Scan Context 描述子必须存在且非空。迁入记录中的基准地图生成了 108 个关键帧；地图变化后数量可以不同，不把 108 当通用门槛。

## 3. 启动仿真 backend 模式

使用和 bundle 同场地的仿真世界、PGM 地图与先验地图包。以下以 `rmuc_2025` 为例；仅当 `$BUNDLE` 确实由该场地对应的地图生成时才用这组参数：

```bash
cd ~/srm_nav_27
source scripts/setup/env_srm27.sh     # ROS + install/ + GTSAM（自动探测前缀）

./scripts/bringup/start_sim_nav.sh \
  -w rmuc_2025 -m rmuc_2025 \
  --lio --lio-frontend zlio \
  --zloc-backend --zloc-prior --zloc-map "$BUNDLE"
```

启动后确认：

```bash
ros2 node list | grep -E 'backend_node|zlio|zlio_bridge|small_gicp|slam_toolbox'
ros2 topic list | grep -E 'localization_state|localization_pose|localization/odometry'
ros2 service list | grep -E 'localization_reset|relocalize|reset_frontend'
```

预期 backend 节点在仿真 namespace 下，通常为 `/red_standard_robot1/backend_node`；不应有 `small_gicp_relocalization` 或 `slam_toolbox`。记录实际 namespace，后续示例设为：

```bash
export NS=/red_standard_robot1
```

## 4. 功能验收步骤

### 4.1 先验地图加载与全局初始化

```bash
ros2 topic echo "$NS/backend_node/localization_state" --once
ros2 topic echo /tf --once
ros2 topic hz /localization/odometry
```

启动日志应包含 `loaded validated map` 和实际关键帧数。状态应先为 `INITIALIZING;pose_trusted=false`，收到有效观测并完成地图几何验证后进入 `NORMAL;pose_trusted=true`。保存初始化耗时与日志中的配准诊断。

在 RViz 中将 Fixed Frame 设为 `map`，显示 Nav2 栅格地图、backend 的先验地图和机器人 TF。先验点云应与栅格图中的墙体/场地边界重合；机器人应出现在实际出生位置附近，朝向一致。若 backend 进入 `NORMAL` 但地图叠加错位，不通过：检查 map bundle 来源和坐标原点，不能通过改 Nav2 初始位姿把错误掩盖掉。

**通过判据：**进入 `NORMAL` 且 `pose_trusted=true`；`/localization/odometry` 持续发布；地图叠加和出生位姿正确；初始化没有跳到另一处对称区域。

### 4.2 正常跟踪与地图校正

用仿真遥控或 Nav2 让机器人在有墙体/结构的区域内完成：直行约 2–3 m、停止、原地转向、再沿不同方向移动。每项动作重复 3 次。查看：

```bash
ros2 topic echo "$NS/backend_node/localization_state"
ros2 topic echo /localization/odometry
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom gimbal_yaw
```

记录每次动作前后的状态、`map → odom` 和 `odom → gimbal_yaw`。若控制器不允许原地转向，可用短弧线替代。注意 `map → odom` 在没有明显漂移时可能变化很小；验收重点是输出稳定、在有全局观测时能合理校正，而非要求它每帧都变化。

要分辨 `map → odom` 的变化是**先验配准**还是**回环**造成的，另开终端运行（不要求打开 backend 的 visualization）：

```bash
./debug_tools/plot_backend_correction.py --duration 120
# 或实车/仿真带命名空间：
./debug_tools/plot_backend_correction.py --ros-args -r __ns:="$NS"
```

面板③给出每次修正的日志报告值，面板④给出同一事件在 `/tf` 上实测的前→后箭头；详细判读见
[debug_tools/README.md](../../debug_tools/README.md) §4。跑完后看各来源（先验/UWB/回环/恢复）
的净贡献与总效果：

```bash
./debug_tools/backend_contribution_report.py --stats-only
```

本节命令只指定 `--zloc-prior`，
回环和 UWB 因未指定开关而关闭；要同开请添加 `--zloc-loop --zloc-uwb`。
没有回环事件不能反推回环已关闭；检查启动能力日志、轨迹重访与几何门控。
联合日志 `sources=prior_scan_to_map+uwb` 的修正量属于联合事务，不可拆成两个独立贡献。
若③一直为空，先查 `/rosout` 是否有 `global correction committed`。

**通过判据：**状态持续可信；无瞬时大幅跳变；机器人在 RViz 中与地图结构保持一致；停止时位姿不持续漂移；backend 不因普通运动反复进入 `SUSPECT`/`LOST`。

### 4.3 错误地图拒绝

先停止当前仿真，再使用另一个场地的 bundle 启动同一场景。不要在 Nav2 已经运行时热换 backend 地图。

```bash
./scripts/bringup/start_sim_nav.sh \
  -w rmuc_2025 -m rmuc_2025 \
  --lio --lio-frontend zlio \
  --zloc-backend --zloc-prior --zloc-map /absolute/path/to/a/different_site_bundle
```

观察至少 30 秒的 `localization_state` 和 `map → odom`：

```bash
ros2 topic echo "$NS/backend_node/localization_state"
ros2 run tf2_ros tf2_echo map odom
```

**通过判据：**backend 不得把错误地图匹配标成可信 `NORMAL`；应停留在未初始化/恢复失败状态，或冻结最后可信输出（若测试从已定位状态注入错误观测）。不能因为服务/节点仍活着就判定通过。测完后必须用正确 bundle 重启。

### 4.4 协调 reset 与连续性恢复

用正确地图启动并等到 `NORMAL`。先让机器人停稳，再调用协调 reset：

```bash
ros2 service call "$NS/localization_reset" std_srvs/srv/Trigger "{}"
```

同时观察 backend 状态、session epoch 和两段 TF：

```bash
ros2 topic echo "$NS/backend_node/localization_state"
ros2 topic echo /zlio/session
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom gimbal_yaw
```

记录 reset 前后的 epoch、可信状态、服务响应和位姿。等待 backend 重新通过先验地图验证并回到 `NORMAL`。再重复调用一次 reset，覆盖连续两次事务；每次都等上一次恢复完成后再调用。

**通过判据：**服务请求被接受；epoch 只推进一次；reset 期间不发布不可信的新全局位姿；`odom → gimbal_yaw` 不因换代跳变；重新匹配后回到 `NORMAL`；`map → odom` 连续或仅发生可由真实地图重定位解释的修正，不发生固定重复补偿量。

若要验证断流自动恢复，可另做一次：在仿真中暂停传感器输入 2 秒后恢复（只暂停 LiDAR/IMU 输入，不结束 Gazebo），确认前端报告 reset 请求、backend 事务协调、epoch 推进及重新定位。此项不替代手动 reset 测试。

### 4.5 定位丢失后的手动重定位（可选）

`~/relocalize` 仅在 backend 状态为 `LOST` 时接受请求。正常 `NORMAL` 状态调用得到拒绝或无动作属于预期，不要为了触发接口而误判为失败。先通过受控故障让 backend 进入 `LOST`，再调用：

```bash
ros2 service call "$NS/backend_node/relocalize" std_srvs/srv/Trigger "{}"
```

跟踪状态是否从 `LOST` 恢复到 `NORMAL`，确认恢复使用新观测完成，而不是只因 service 返回 `success` 就判成功。若现场/仿真无法可重复地注入 LOST，应将此项标记“未测”，不要通过错误地图启动代替 LOST 后恢复测试。

### 4.6 唯一 TF 所有者与导航接管

在 backend 模式下运行：

```bash
ros2 node list | grep -E 'static_transform_publisher|small_gicp|backend_node|slam_toolbox'
ros2 run tf2_tools view_frames
```

检查 `/tf` 中 `map → odom` 只有 backend 发布；`odom → gimbal_yaw` 仍由本项目的 `sensor_scan_generation` 发布。然后发一个短距离 Nav2 目标，观察机器人移动期间定位状态、地图匹配和代价地图；结束后再做一次停止保持。

**通过判据：**无双 TF 发布者；Nav2 能正常到达短距离目标；运动期间 costmap/路径不整体跳动；到点停止后 backend 继续保持可信状态。导航任务通过但定位状态错误，或定位状态正常但 map/PGM 错位，都不通过。

## 5. 结果记录

每个测试场景至少保存：启动命令、bundle 路径与 SHA256、世界/PGM 名称、backend 配置、完整 backend 日志、定位状态 rosbag、动作前后 `map → odom` 和 `odom → gimbal_yaw`、结论。

建议录制：

```bash
mkdir -p "$HOME/srm_nav_27_data/zlocalization/acceptance"
ros2 bag record -o "$HOME/srm_nav_27_data/zlocalization/acceptance/run_$(date +%Y%m%d_%H%M%S)" \
  /tf /tf_static \
  "$NS/backend_node/localization_state" \
  "$NS/backend_node/localization_pose" \
  /localization/odometry \
  /zlio/session /zlio/frame
```

用表格汇总结论：

| 项目 | 结果 | 证据/备注 |
|---|---|---|
| 地图包加载 | 通过 / 失败 | keyframe 数、bundle SHA256 |
| 全局初始化 | 通过 / 失败 | 初始化耗时、最终状态、RViz 截图 |
| 正常跟踪 | 通过 / 失败 | 运动路线、状态变化、TF 轨迹 |
| 错误地图拒绝 | 通过 / 失败 | 错误 bundle、状态和可信标志 |
| 协调 reset | 通过 / 失败 | epoch、TF 连续性、恢复耗时 |
| LOST 手动重定位 | 通过 / 失败 / 未测 | LOST 注入方式、服务和恢复证据 |
| 唯一 TF owner / Nav2 | 通过 / 失败 | `view_frames`、导航目标结果 |
| 资源与延迟 | 通过 / 失败 | CPU、峰值内存、恢复耗时 |

## 6. 实车复验顺序

仿真通过后，将同一套功能项移到静态、结构明显的实车场地低速复验：正确地图初始化 → 直行/转向跟踪 → 停车 reset → 错误地图拒绝（可只做短时定位节点测试，不接导航）→ 短距离 Nav2 目标。保持急停可用，首次实车验收不要边移动边测试 reset。

实车上另记录 CPU/内存、初始化耗时、恢复耗时和连续运行时长。任何 `NORMAL` 状态下的地图错位、TF 双发布、未解释的位置跳变或错误地图被接受，都判失败并移除 `--zloc-backend` 回到原定位模式。

## 7. 当前实现参考

- [Backend 接入契约](../architecture/localization_backend.md)：TF owner、frame 与 reset 语义。
- [迁入实施记录](../records/2026-11-16-zlocalization-backend-migration.md)：已有构建、测试和地图包验证证据。
- [Backend 包 README](../../src/localization/backend/srm27_zlocalization_backend/README.md)：话题、服务、状态和地图工具说明。
- [综合实车验收手册](实车分段验收-ZLIO与backend.md)：包含本链前端与底盘检查；执行本手册时可跳过其前端数据链章节。

## 8. 227 实地首轮 reset 故障与复验（2026-11-19）

本轮 reset **未通过恢复验收**。服务返回 `coordinated front-end reset transaction started`
只表示请求已接受；随后 backend 为 `LOST;epoch=0`，TF 时间戳停止更新。
录包 `~/srm_nav_27_data/zlocalization/acceptance/reset_20261119_181136` 显示：

- `/zlio/session` 已从 epoch 0 切到 1，且 `reset_required=false;failed=false`。
- backend 最后一条状态为 `manual coordinated front-end reset`，定位输出随之停止；录包继续约 21 秒仍无恢复。
- 本次启动日志记录 `zlio_bridge.py` 在 reset 后退出，exit code 1；backend 随后反复报告 `zlocalization/reset_frontend` 服务不可用。

代码检查发现 `zlocalization_backend_adapter.py::_on_reset` 只填充响应，未返回响应对象。
rclpy 使用回调返回值发送服务响应，因此 `None` 会触发 `TypeError`，导致 bridge 退出。
已补齐成功、服务不可用、超时、传输异常四条路径的 `return response`；新增回归在修复前失败，
修复后适配层 15 项及桥接数学 7 项测试通过。实机恢复效果仍待复验，不能用单元测试代替。

复验时保持车和云台静止，先用 `start_real_nav.sh --stop` 停止旧链，再按本场地参数重新启动
`--lio --lio-frontend zlio --zloc-backend --zloc-prior --zloc-map <227 bundle>`，继续保留 `--no-chassis --no-joy`。
等待 NORMAL 后按 §4.4 重新测试，记录 bridge 是否仍存活、前后端 epoch 是否完成切换、
TF 时间戳是否重新推进，以及恢复耗时和位姿连续性。每次恢复完成后才测试下一次 reset。


## 9. 场上独立开关与三融合验收

三项默认全关；按需要选择 `--zloc-prior`、`--zloc-loop`、`--zloc-uwb`。
仅指定 `--zloc-map` 不会启用先验，旧命令需要补 `--zloc-prior`。

```bash
./scripts/bringup/start_real_nav.sh \
  -m "$PWD/maps/227_1113pro/227_1113pro.yaml" \
  --lio --lio-frontend zlio --zloc-backend \
  --zloc-prior --zloc-loop --zloc-uwb \
  --zloc-map "$HOME/srm_nav_27_data/zlocalization/map_bundles/227_1113pro"
```

可附加 `--zloc-config /absolute/path/to/site.yaml` 指定现场配置（支持 `extends`）。
先核对 UWB PoseStamped 话题、frame、坐标对齐、天线杆臂与时间偏置；迁入基线不是本车标定值。
当前 UWB 固定杆臂模型不自动补偿“天线在底盘、传感器在旋转云台”的动态外参。
量级可以估算：天线与传感器水平距离 r，云台相对标定姿态转 θ 时杆臂误差约 `2·r·sin(θ/2)`；
本机 r≈0.245 m，θ=10°/45°/90° 分别约 4/19/35 cm，已经超过 `position_std_xy` 的 0.15 m。
因此验收期间保持云台固定；若天线也随云台转，杆臂要按云台系测量后再填 `antenna_position_body`。

1. 启动日志应为 `unified backend ready: prior_map=enabled, loop=enabled, uwb=enabled`。
2. 在同一次运行中确认 `global correction committed ... sources=prior_scan_to_map+uwb`
   （或单来源通过门控的提交），以及 `UWB factor accepted in system transaction`。
3. 重访已走过的位置，确认 `loop scan-to-map accepted`；只打开开关不保证每次重访都验收回环。
4. 完成 §4.4 的 reset，确认 epoch 推进、重新 NORMAL，先验与后续回环仍可用。
5. 对照单先验记录定位误差和 CPU/内存。回环启用会自动关闭 fixed-lag 并保留运行历史，
   图与关键帧内存会随运行增长。

长时观测常驻内存（另开终端，每 10 分钟记一行，同时记累计里程与日志里的关键帧数）：

```bash
while sleep 600; do
  pid=$(pgrep -f srm27_zlocalization_backend_node | head -1)
  printf '%s ' "$(date +%H:%M)"; ps -o rss=,pcpu=,etime= -p "$pid"   # RSS 单位 KiB
done
```

回环开启时 RSS 单调增长是预期行为（`graph.fixed_lag_seconds` 被置 0、历史关键帧不再裁剪）；
要看的是“每米轨迹/每个关键帧增长多少”，而不是单看绝对值。

合成场景回归覆盖八种开关配置、先验+回环（有/无 UWB）的实际因子提交、历史节点保留、
前端换代后的双库隔离；这些不代表已通过现场三融合精度和长时运行验收。
