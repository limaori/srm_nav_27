# Z-Localization 后端继续验收方案

> 适用范围：**prior + loop**（UWB 暂不纳入：当前没有物理接入，后端也收不到有效 UWB 观测）。
>
> 本方案是 [Z-Localization后端功能验收.md](Z-Localization后端功能验收.md) 的续篇，不重复检查雷达驱动、IMU 单位、逐点时间、QoS、地图包生成等基础接线；只针对当前已经暴露的**输出稳定性、复位/恢复、回环收益、导航联动、长跑开销**做继续验收。
>
> 当前基线（2026-11-19 20:48 `all_*`）：prior 99 接受 / 130 冲突，loop 23 接受 / 114 拒绝，OBS_DEGRADED 56.5%，输出位姿整体稳定，但存在一个 **0.154 m / 9.8 ms** 的输出尖峰；没有 reset，没有 LOST。基线不是“通过”，而是本轮测试的起点。

---

## 1. 本轮要回答的问题

1. **输出位姿**（`TF map→base_link` / body pose）在正常、自转、急加速/急转下是否稳定，是否有 >5 cm / >10 cm 的跳变？
2. **绝对精度**：静止已知点、重复到同一点、正常路径的 map 误差是多少？稳定不等于准确。
3. **复位/恢复**：`/localization_reset`、前端 reset、SUSPECT/LOST recovery 是否可靠？epoch、TF、输出连续性是否正确？
4. **状态机**：OBS 56.5% 却没有 SUSPECT，是真稳健还是冲突计数被 accepted 重置？注入故障时能不能升级？
5. **回环**：prior-only vs prior+loop 在同一条非自转重访路径上，loop 是否真的改善精度？误闭合率和 full graph 内存代价多大？
6. **导航联动**：0.15 m 级定位台阶和 0.3 m 净空同时存在时，Nav2 的脱困/碰撞是否增加？是否需要定位跳变保护？
7. **长跑开销**：prior 很便宜，loop full graph 的 CPU/RSS 增长是否可接受？

UWB 的验收另开一轮，等硬件接入后再做；本方案不把 UWB 的 0 当成“效果为 0”。

---

## 2. 验收原则：先分清“纠正面”和“输出面”

这是上一轮最容易误判的地方，本轮必须固定口径：

- **输出面**：`TF map→base_link`（或 `${NS}/backend_node/localization_pose`）。判断“定位稳不稳、准不准”只看这个。
- **纠正面**：`map→odom`。它随前端 odom 反向变化；前端漂 0.1 m，map→odom 就要改 0.1 m。**map→odom 台阶大不代表输出不稳。**
- **因子贡献**：`Σ|Δp|` 是累计绝对修正量，是“补偿活动量”，不是误差；net 是纠正面净变化，也不是输出误差。
- 所有时间序列统计必须用消息 `stamp_ns`，不要用墙钟到达时间；消息突发会把 0.6 s 的变化压成 0.05 s，制造假跳变。

验收报告里每个图/表必须标注它属于输出面还是纠正面。

---

## 3. 指标与初始门槛

门槛是首轮目标，允许在第一次基线后调整，但调整必须写进报告，不能事后放宽。

### 3.1 输出位姿（主指标）

从 `/tf` 取 `map→base_link`，至少 10 Hz（不足则用 backend body pose 话题），统计：

| 指标 | 正常运动目标 | 自转/激烈目标 | 失败条件 |
|---|---:|---:|---|
| 单步平移最大值（按 stamp 相邻样本） | ≤ 3 cm | ≤ 10 cm | 出现 > 10 cm 且 body 也跳 |
| 平移速度 p99 | ≤ 1.5 m/s | ≤ 2.5 m/s | 出现 > 5 m/s 的不可解释尖峰 |
| yaw rate p99 | ≤ 120°/s | ≤ 200°/s | 出现 > 300°/s 的不可解释跳变 |
| 跳变计数 > 5 cm | 0 | ≤ 2 次/5 min | 自转段 > 5 次/5 min |
| OBS_DEGRADED 时间占比 | ≤ 10% | ≤ 30% | > 50% 或持续 > 5 s |
| SUSPECT / LOST | 0 / 0 | 0 / 0 | 出现 LOST；或 SUSPECT 后恢复失败 |
| reset | 按 T5 单独验收 | 同左 | epoch/TF 不连续 |

> 当前 20:48 数据在自转段出现 1 次 0.154 m / 9.8 ms 的输出尖峰；T3 要专门复现/排除。若 body pose 不跳，只算 sensor/gimbal 采样点，不算失败；若 body 也跳，按失败处理。

### 3.2 先验/回环健康度（辅助指标）

| 指标 | 目标 |
|---|---|
| prior accepted / conflicted | 正常段 accepted ≥ 90%，conflict ≤ 10%；自转段记录实际值，不强制 |
| prior accepted NIS | 中位 < 3；最大不应长期贴门限 |
| prior scan-to-map RMSE | 中位 ≤ 0.20 m，支持率 ≥ 60% |
| loop accepted / rejected | 清晰重访路径 accepted ≥ 5 次；拒绝率记录；不能 0 接受 |
| loop 误闭合 | 不允许 target 明显错误且被接受；NIS 贴门限的接受要单独列出 |
| 单次最大修正 | 正常 ≤ 5 cm；自转允许 10–15 cm，但必须逐条解释 |

### 3.3 绝对精度（T1/T2）

| 指标 | 目标 |
|---|---:|
| 静止已知点位置误差 | ≤ 5 cm |
| 静止 yaw 误差 | ≤ 0.5° |
| 同一点重复 3 次的位置重复性 | ≤ 2 cm |
| 正常路径回到同一点的闭合误差 | ≤ 10 cm |
| 正常路径已知点最大误差 | ≤ 15 cm |

没有真值时，至少用场地已知点/标定物/全站仪；不能用 `map→odom` 代替精度。

### 3.4 导航与开销

| 指标 | 目标 |
|---|---|
| Nav2 goal 成功率 | ≥ 90%（同一条路线，与基线对比） |
| 脱困次数 | 不高于基线；每次都要有时间、定位跳变、障碍物记录 |
| 碰撞/接近碰撞 | 0 |
| backend CPU | 单核占用 < 100%（记录 p95） |
| backend RSS | 30 min 增长 ≤ 100 MB，且不单调泄漏 |
| prior 配准 p95 | ≤ 20 ms |
| loop 候选/验证 p95 | 首轮先补日志，目标 ≤ 50 ms/周期（按实测调整） |

---

## 4. 测试矩阵

| ID | 场景 | 配置 | 时长 | 优先级 | 目的 |
|---|---|---|---:|---|---|
| T0 | 预检/静态基线 | prior only | 2 min | P0 | 确认地图、TF、服务、唯一 map→odom 发布者；建立输出基线 |
| T1 | 静止已知点/重复性 | prior only | 10 min | P0 | 绝对精度、静止漂移、yaw 精度 |
| T2 | 正常运动（无自转） | prior only | 10 min | P0 | 正常输出稳定性、prior 健康度 |
| T3 | 分级自转 | prior only | 4 × 60 s | P0 | 2/4/6/8.5 rad/s 下的输出尖峰、OBS、时间同步 |
| T4 | 自转 + 导航 | prior only；再 prior+loop | 5 min × 2 | P0 | 导航联动、脱困/碰撞、回环是否有帮助 |
| T5 | reset / 前端 reset | prior only | 5 次 | P0 | epoch、TF 连续性、恢复时间、双发布者 |
| T6 | 故障注入/状态机 | prior only | 3 组 | P0 | OBS→SUSPECT→recovery 是否按预期升级 |
| T7 | 回环 A/B | prior only vs prior+loop | 10 min × 2 | P1 | 同一非自转重访路径的精度、误闭合、内存 |
| T8 | 长跑/开销 | prior+loop | 30–60 min | P1 | CPU/RSS/full graph/关键帧增长 |
| T9 | UWB | 暂不做 | — | P2 | 硬件接入后另开 |

建议顺序：**T0 → T1 → T2 → T5 → T3 → T6 → T4 → T7 → T8**。先做静止/复位/状态机，再做自转和回环；不要在 reset/状态机没验完时直接跑高速自转导航。

---

## 5. 通用流程（每个测试都按这个做）

### 5.1 预检

```bash
cd ~/srm_nav_27
source scripts/setup/env_srm27.sh

# 实际 namespace；无 namespace 时留空
export NS=/red_standard_robot1

# 地图包、校验值
test -s "$BUNDLE/keyframes.txt"
find "$BUNDLE" -maxdepth 2 -type f -printf '%P %s bytes\n' | sort | head
sha256sum "$BUNDLE/keyframes.txt"

# 只允许 backend 发布 map→odom
ros2 node list | grep -E 'backend_node|small_gicp|slam_toolbox'
ros2 run tf2_ros tf2_monitor map odom | head -40   # 只应看到 backend_node 一个 authority
ros2 topic list | grep -E "${NS}/backend_node/localization_state|${NS}/backend_node/localization_pose|/localization/odometry"
ros2 service list | grep -E "localization_reset|relocalize|reset_frontend"
```

### 5.2 启动

```bash
# prior only
./scripts/bringup/start_real_nav.sh --lio --lio-frontend zlio \
  --zloc-backend --zloc-prior --zloc-map "$BUNDLE"

# prior + loop（仅 T4/T7/T8 使用）
./scripts/bringup/start_real_nav.sh --lio --lio-frontend zlio \
  --zloc-backend --zloc-prior --zloc-loop --zloc-map "$BUNDLE"
```

等 `localization state -> NORMAL`，确认唯一 `map→odom` 发布者，再开始。

### 5.3 录包

```bash
mkdir -p user_log/bags
ros2 bag record -o "user_log/bags/accept_${RUN_ID}" \
  /tf /tf_static \
  /localization/odometry \
  ${NS}/backend_node/localization_pose \
  ${NS}/backend_node/localization_state \
  /rosout /cmd_vel /Odometry \
  ${NS}/frontend/frame ${NS}/frontend/odometry \
  /livox/imu /gimbal_joint_angle
```

`/tf`、`/rosout`、`/localization/odometry`、`/cmd_vel`、`/Odometry`、`/livox/imu`、`/gimbal_joint_angle` 是绝对名；`localization_pose/state`、`frontend/*` 是 backend/frontend 的相对名，必须带 `$NS`。UWB 接入后另加 `/uwb_pose`。

### 5.4 录纠正数据

```bash
./debug_tools/plot_backend_correction.py --duration 300 --save "accept_${RUN_ID}.png"
LATEST_LOG=$(ls -t "$HOME"/.ros/log/srm27_zlocalization_backend_node_*.log | head -1)
./debug_tools/backend_contribution_report.py \
  --samples "user_log/accept_${RUN_ID}_samples.csv" \
  --events  "user_log/accept_${RUN_ID}_events.csv" \
  --backend-log "$LATEST_LOG"
```

`plot_backend_correction` 画的是纠正面；输出面必须从 bag/TF 另算。

### 5.5 输出面统计

从 bag 或实时 TF 取 `map→base_link`（实际 body frame 以 TF 树为准），按 `stamp` 计算：

- 单步平移、速度、加速度；
- yaw rate；
- >5 cm / >10 cm 跳变次数；
- OBS/SUSPECT/LOST 时间；
- reset/epoch 次数。

当前工具还没有 body 面板；首轮可以用 bag 回放 + `tf2_echo`/Python 脚本，建议补一个 `debug_tools/plot_localization_output.py`，作为本轮交付物之一。

### 5.6 判定

每个测试填一张表：

```text
RUN_ID:
场景:
配置:
时长:
bag:
backend log:
输出面 max step / p99 speed / yaw rate:
>5cm / >10cm 次数:
OBS% / SUSPECT / LOST / reset:
prior accepted/conflict:
loop accepted/rejected:
精度（如有）:
导航（如有）:
失败点:
结论: PASS / FAIL / CONDITIONAL
```

---

## 6. 专项步骤

### T0 预检/静态基线

1. 车静止，启动 prior only。
2. 确认 `NORMAL`、唯一 `map→odom`、无 LOST。
3. 静止 120 s，录输出面 + 纠正面。
4. 检查输出面位置漂移、yaw 漂移、跳变。

**通过**：静止 120 s 输出位置漂移 ≤ 2 cm，yaw ≤ 0.2°，无 >2 cm 台阶；无 SUSPECT/LOST。

### T1 静止已知点/重复性

1. 在场地选 3 个已知点（全站仪/卷尺/地图 landmark），车分别停放。
2. 每个点静止 60 s，记录 `map→base_link` 与已知坐标差。
3. 同一点重复 3 次（熄火/重新启动 backend 或 reset 后重新定位）。
4. 计算位置误差、yaw 误差、重复性。

**通过**：位置误差 ≤ 5 cm，yaw ≤ 0.5°，重复性 ≤ 2 cm；记录每个点的误差向量，不能只报均值。

### T2 正常运动（无自转）

1. prior only，沿固定路线正常行驶 10 min：直线、慢转、停车再起步，**不要持续自转**。
2. 中途在 2 个已知点停车 30 s，记录误差。
3. 终点回到起点，记录闭合误差。
4. 统计输出面 max step、p99 speed、OBS%、prior accepted/conflict。

**通过**：输出 max step ≤ 3 cm；OBS ≤ 10%；无 SUSPECT/LOST；闭合误差 ≤ 10 cm；prior conflict ≤ 10%。

### T3 分级自转（核心）

1. 空旷区域，车静止起步，prior only，loop off。
2. 依次做 **2 / 4 / 6 / 8.5 rad/s** 自转，每档 60 s；两档之间停 10 s。
3. 同步录 `/livox/imu`、`${NS}/frontend/frame`、`/Odometry`、`/tf`、`/cmd_vel`。
4. 统计每档输出面 max step、>5/>10 cm 次数、OBS%、prior accepted/conflict、map→odom 修正量。
5. 专门检查 20:48 数据里 2493.0 附近的 0.154 m / 9.8 ms 尖峰能否复现；如果复现，查 body、IMU 时间、杆臂、deskew。
6. 用 IMU 陀螺积分对比 ZLIO yaw；比较原地自转时平移是否漂。

**通过**：输出面 body max step ≤ 10 cm，>10 cm 次数为 0；无 LOST；OBS ≤ 30%；8.5 rad/s 下允许更大的 correction，但必须可解释。若 0.154 m 尖峰在 body 上复现，判 FAIL，先修时间同步/杆臂/源点云，再继续。

### T4 自转 + 导航

1. 同一条短路线，prior only 跑 5 min；再做 prior+loop 5 min。
2. 记录 goal 成功率、脱困次数、碰撞/接近碰撞、输出跳变、OBS。
3. 加一个**输出跳变保护**的试验：body 单步 >10 cm 或 OBS >2 s 时暂停/清 costmap；对比有无保护。

**通过**：无碰撞；goal 成功率 ≥ 90%；脱困不高于基线；body 跳变 ≤ 10 cm；保护触发时能安全停车/恢复。

### T5 reset / 前端 reset

1. 车静止在已知点，确认 `NORMAL`，记录 epoch、`map→base_link`、`map→odom`。
2. 调用：

```bash
ros2 service call /localization_reset std_srvs/srv/Trigger
# 带 namespace: ros2 service call /$NS/localization_reset std_srvs/srv/Trigger
```

3. 观察：
   - service 返回 success；
   - front-end reset 完成、epoch +1；
   - backend 重新进入 `INITIALIZING` → `NORMAL`；
   - `map→odom`/输出是否连续，是否有双发布者；
   - 恢复时间。
4. 重复 5 次；再做一次“前端单独重启”与“backend 单独重启”的对照。
5. 只有 `LOST` 才测 `~/relocalize`；不要在 NORMAL 下测。

**通过**：5/5 成功；epoch 只 +1；恢复 ≤ 2 s（按实际调整）；输出跳变 ≤ 10 cm；无重复 `map→odom`；无 stale TF。

### T6 故障注入/状态机

在仿真或安全场地做，不要直接实车高速注入：

1. **点云中断**：暂停/断开 lidar 2–3 s，观察 OBS→SUSPECT→LOST 的时间。
2. **odom 跳变**：注入 0.5 m / 10° 的前端跳变，观察是否冲突、是否恢复。
3. **先验不匹配**：用错误地图/遮挡，观察是否降级而不是继续输出。
4. 每次记录状态转换时间、恢复结果、输出跳变、是否误恢复。

**通过**：OBS 持续 >2 s 应进入 SUSPECT（或明确的降级）；恢复后 2–3 帧低 NIS 才回 NORMAL；错误恢复能回滚；不得长时间“OBS 但继续输出大修正”。

### T7 回环 A/B

1. 选一条有 **3 个以上清晰重访** 的路线，正常行驶，不持续自转。
2. 先 prior only 跑 10 min；再 prior+loop 跑同一条路线 10 min（同一司机/速度/起点）。
3. 记录 loop accepted/rejected、target、NIS、单次修正、误闭合；输出面精度（已知点/闭合误差）；RSS/关键帧。
4. 对比：loop 是否改善输出误差？是否引入 >3 cm 的恶化？是否出现错误 target？

**通过**：loop accepted ≥ 5；无误闭合；输出精度不差于 prior-only 超过 3 cm；RSS 增长 ≤ 100 MB/30 min；若 loop 无收益，记录“关闭 loop”的决策，不要为了开而开。

### T8 长跑/开销

1. prior+loop 连续跑 30–60 min，包含正常、自转、停车。
2. 记录：

```bash
top -b -d 1 -p $(pgrep -f srm27_zlocalization_backend_node)
ps -o pid,rss,%cpu,etime -p $(pgrep -f srm27_zlocalization_backend_node)
```

3. 记录关键帧数、loop 候选/接受、prior 配准耗时、RSS 曲线。
4. 若 loop 打开 full graph，观察内存是否单调增长。

**通过**：CPU 单核 < 100%；RSS 30 min 增长 ≤ 100 MB 且无持续泄漏；无 LOST；输出面指标不随运行时间退化。

### T9 UWB（暂不做）

硬件接入后单独验收：话题/QoS/frame、时间偏置、天线杆臂、UWB-only、prior+UWB、三融合；注意 `sources=prior_scan_to_map+uwb` 是联合事务，不能从一条日志拆成两项。

---

## 7. 通过判定

本轮验收按 Gate 汇总，**全部 Gate 通过才算整体通过**：

| Gate | 内容 | 通过条件 |
|---|---|---|
| G1 | 输出稳定性 | T1/T2/T3 的输出面指标达标；无 LOST；15 cm 尖峰有结论 |
| G2 | 精度 | T1/T2 已知点/闭合误差达标；有真值或 landmark 证据 |
| G3 | reset/恢复/状态机 | T5 5/5；T6 能升级/恢复/回滚；无重复 map→odom |
| G4 | 回环 | T7 有收益且无误闭合/内存可接受；否则明确关闭 |
| G5 | 导航联动 | T4 无碰撞、成功率达标、保护有效 |
| G6 | 长跑开销 | T8 CPU/RSS 达标，无随时间退化 |

如果某个 Gate 失败，记录为 blocker，不要用“整体看起来稳定”覆盖；先修 blocker，再跑下一轮。

---

## 8. 交付物与命名

每个测试产出：

```text
user_log/accept_<date>_<test>/
  README.md                # 场景、配置、结果、PASS/FAIL
  output_pose.csv          # map→base_link 统计（若工具有）
  *_samples.csv            # plot_backend_correction 输出
  *_events.csv
  *.png
  backend.log
  zlio_bridge.log
  controller.log
```

bag 放 `user_log/bags/`（已在 `.gitignore` 内）。

最终交付一份 `docs/records/<date>-zlocalization-acceptance.md`，包含：

- 每个 Gate 的 PASS/FAIL；
- 关键指标表；
- 失败复现步骤；
- 是否允许进入比赛/导航的结论；
- 未完成项（UWB、长跑等）。

---

## 9. 安全与停止条件

- 自转测试在空旷场地，急停可用，人员远离；8.5 rad/s 时不要站车旁。
- reset 只在车静止时做；reset 期间不要发 Nav2 目标。
- 出现以下任一情况立即停止并保存日志：
  - LOST；
  - 输出 body 单步 > 20 cm；
  - 双 `map→odom` 发布者；
  - TF 断 > 1 s；
  - 碰撞/接近碰撞；
  - reset 后 epoch/TF 不一致。
- 故障注入优先在仿真做；实车只做低风险的点云中断/odom 跳变。

---

## 10. 建议排期

| 时间 | 内容 |
|---|---|
| 第 1 天上午 | T0/T1：预检、静止已知点、输出基线 |
| 第 1 天下午 | T2/T5：正常运动、reset/前端 reset |
| 第 2 天上午 | T3/T6：分级自转、故障注入/状态机 |
| 第 2 天下午 | T4/T7：自转导航、回环 A/B |
| 第 3 天 | T8：长跑/开销；汇总报告；UWB 另开 |

如果 T3 的 15 cm 尖峰在 body 上复现，暂停 T4/T7，先修时间同步/杆臂/源点云；如果 T5 reset 不通过，暂停所有导航验收。

---

## 11. 本轮最关键的三个判断

1. **输出面是否真的稳**：不是看 `map→odom`，是看 `TF map→base_link`；15 cm 尖峰必须有结论。
2. **reset/状态机是否能接住故障**：OBS 56.5% 却没有 SUSPECT，这是当前最大的验收缺口。
3. **loop 是否值得开**：23/114 的接受率和 full graph 内存，必须用同路径 A/B 证明收益，否则自转/长跑时关闭。
