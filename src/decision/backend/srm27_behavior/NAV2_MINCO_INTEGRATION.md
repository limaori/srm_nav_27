# 2027 决策与 YAstar / MinimumSnap / MINCO 联调

本轮以 `origin/main` 的 `45dd22a595a5f0eba00a61ef0a74f75f9d9dfb99` 为基线，用合并提交 `349843c` 保留 `feat/决策模块` 的原有历史。

## 合并和接口

| 冲突范围 | 处理结果 |
| --- | --- |
| 决策目录迁移、重名插件和测试 | 采用主线的 `backend/srm27_behavior`、`frontend/srm27_behavior_plugins`、`common/srm27_behavior_core`。核对迁移后的 UC/UL XML、航点、许可和取消逻辑，并保留主线 `dev_rm.xml` 中的 `test_spin_speed`。 |
| 实车与仿真脚本、Nav2 参数 | 保留 YAstar / MinimumSnap 的新插件 ID、主线的限速与恢复参数。自动决策继续要求 mux 的运动许可，测试树按显式开关启动。 |
| MINCO 控制器与脱困 | 保留主线的实测位姿判停、制动净空、首选脱困方向及 pipeline 包名，合并 `BackUpFreeSpace` 的一秒代价地图服务等待上限，服务缺失时停车并返回失败。 |
| 串口功率测试 | `e15aebd` 已改用六个限幅参数，`test_speed_profile` 仍在检查旧功率档表。将该断言改为通过伪终端读取真实节点发出的 CRC 校验帧，检查合速度、自转上限、加速过程和看门狗归零。 |

决策继续调用 `NavigateToPose` 和 `NavigateThroughPoses`。这两种 Action 在 Nav2 树里使用 `GridBased`，启用 MinimumSnap 后，将平滑路径送到 `FollowPath`。决策根据 Action 结果和地图坐标系下的实际位置判断到点，超时或失败清除当前运动许可状态，并在退避后重试。普通导航失败在 `decision/status.nav_fault` 中可见，`test_spin_speed` 通过 Action 状态话题的持久 QoS 读取已有状态。

| 层级 | 默认 `trajectory_source: internal` | 规划端 `trajectory_source: topic` |
| --- | --- | --- |
| Nav2 全局与平滑 | YAstar 生成 `plan`。启用 MinimumSnap 后，Nav2 树调用 `SmoothPath` 并产生 `plan_smoothed`。 | `GridBased` 加载 `srm27_minco_planner::MincoPlanner`，内部复用 YAstar。相同的 Nav2 树和 Action 继续运行。 |
| 局部规划 | `MincoMpcController` 通过 `srm27_minco_pipeline` 的内部工作线程生成局部轨迹。 | `FollowPath` 通过 `planning/follow_path` 刷新实际执行路径和有效速度上限。规划端输出 `planning/minco_trajectory`，包含 odom 系的原始多项式、有效期和对应请求版本。 |
| MPC 与速度 | MPC 在本地 ESDF 上复核轨迹，输出云台 fakeyaw 系的平移。 | MPC 只接收当前任务及路径版本的轨迹，仍在本地 ESDF 上复核，检查动态限速和有效期。无可用轨迹时输出零平移，超出起轨时限后交给 Nav2 恢复。 |
| 速度落地 | `fake_vel_transform` 转换一次平移，mux 合成 `rotation_cmd` 并应用许可与限幅，最终输出 `cmd_vel_sim` 或 `cmd_vel_chassis`。 | 同左。 |

规划端模式从有效的 `FollowPath` 配置读取优化、净空及限速参数。全局重新规划时，规划器只发布仍匹配当前请求的结果。收到同一目标的新路径时，控制器可在有效期内继续执行已校验的轨迹。切换任务后作废旧轨迹。当前激活请求停止刷新后，规划端停止继续为该路径求解。修改过轨迹消息定义的节点需要一起重新构建。

## 构建

在 Ubuntu 22.04 / ROS 2 Humble 使用 BehaviorTree.CPP 4.6.2。MINCO 和 YAstar 使用仓库内的依赖，MinimumSnap 需要 `ros-humble-osqp-vendor` 及 [OsqpEigen v0.8.0 安装步骤](../../../../docs/archive/2026-11-24-tdt-nav-kit-README.md)，bringup 的定位可选分支还依赖 GTSAM 4.2。自定义安装前缀需加入 `CMAKE_PREFIX_PATH` 和 `LD_LIBRARY_PATH`。

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --parallel-workers 4 \
  --packages-up-to srm27_behavior srm27_nav_bringup srm27_nav_protocol \
    srm27_nav_plugins fake_vel_transform \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
```

## 复现 219_1121 Gazebo 验证

在仓库根目录用两终端执行。仿真用独立的 ROS 域和 Gazebo 分区。测试航点来自该地图的 `goals.json` 和出生点，场景清单提供 `map → odom` 平移。

终端 1 生成本轮参数并启动 Gazebo、Nav2 和决策。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=92 ROS_LOCALHOST_ONLY=1 IGN_PARTITION=srm27-gazebo-219
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/tests/recovery/fastdds.xml"
run_dir="$PWD/.cache/decision-gazebo-219"
mkdir -p "$run_dir"
python3 - "$run_dir" <<'PY'
from pathlib import Path
import sys
import yaml

out = Path(sys.argv[1])
behavior = Path("src/decision/backend/srm27_behavior")
config = yaml.safe_load((behavior / "params/sentry_behavior.yaml").read_text())
config["srm27_behavior_server"]["ros__parameters"].update(
    waypoints_file=str((behavior / "test/gazebo_219_waypoints.yaml").resolve()),
    status_topic="mock/sefdefined",
    groot2_port=31692,
)
(out / "behavior.yaml").write_text(yaml.safe_dump(config))
nav = Path("src/bringup/srm27_nav_bringup/config/simulation/current/nav2_params_srm_fakeyaw.yaml")
config = yaml.safe_load(nav.read_text())
config["controller_server"]["ros__parameters"]["FollowPath"]["trajectory_source"] = "topic"
(out / "nav2.yaml").write_text(yaml.safe_dump(config))
PY
PREKILL=0 OPEN_MODE=direct TERMINAL=bash ./scripts/bringup/start_sim_nav.sh \
  --no-gui --no-rviz -m 219_1121 \
  --sim-config "$PWD/maps/219_1121/simulation/sim.yaml" \
  --params "$run_dir/nav2.yaml" --yastar-planner --minisnap-smoother \
  --behavior rmul_2027 --behavior-params "$run_dir/behavior.yaml"
```

终端 2 发布模拟裁判输入并记录运行结果。验收通过后，在终端 1 按 Ctrl+C 结束。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=92 ROS_LOCALHOST_ONLY=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/tests/recovery/fastdds.xml"
python3 src/decision/backend/srm27_behavior/test/verify_gazebo_minco.py \
  --output "$PWD/.cache/decision-gazebo-219/gazebo.json"
```

`219_1121` 的实车决策入口同样需要 `--behavior-params`，应使用按该地图和己方颜色标定的实车参数文件。

## 验证结果

- Humble 容器内定向构建通过，覆盖 43 个相关包及其工作区依赖。49 个 CTest 条目及 32 个决策/平面闭环场景通过，另对本轮修改的决策失败、运动门控和串口用例重跑 5 项。

- Gazebo 使用 Fortress、`219_1121` 场景、点云代价地图及真值里程计。两次 `NavigateToPose` 均返回成功。观察到 YAstar、MinimumSnap、规划端多项式轨迹、MPC、fakeyaw 和 mux 的实际输出。
- 决策从等待比赛切换到去中心。到点后保持驻点自转。HP 400 → 350 触发短时变速，HP 110 切换回补。比赛结束时 mux 平移和自转归零，规划端随后停止输出轨迹。
- 一键脚本跑通两次 Action。中心与回补的停车点距目标分别约 0.309 m 和 0.313 m，均在 0.40 m 的仿真判停容差内。收到两份全局路径、两份平滑路径和 23 帧规划端轨迹，两次任务的版本均匹配。
- 伪终端验证实际节点输出 19 字节 CRC 帧，平移合速度限到 2.60 m/s，底盘转速限到 8.50 rad/s。断开请求后实际发送零速帧。
- 一次复跑中回补的首轮局部路径被净空检查判为阻塞，Nav2 清理代价地图并重试，随后到点。验证范围为本地图代表性单点往返，长路线、多点实际 Gazebo 路径和实车待独立验收。
- 决策及平面闭环回归可用 `src/decision/backend/srm27_behavior/scripts/validate_decision.sh --docker --nav2`。规划、控制、地图、协议和 bringup 使用各包构建目录的 `ctest -R '^test_' --output-on-failure`。实车脚本用 `--check` 验证命令生成，仿真脚本用 `DRY_RUN=1` 验证场景、地图变换、插件和决策参数的传递。
