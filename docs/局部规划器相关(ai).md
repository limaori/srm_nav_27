# 局部规划器相关(ai)

本项目的局部控制器跟踪全局规划器给出的路径。当前结构是：

```text
Theta* 全局路径规划 → OmniPidPursuitController 路径跟踪 → Velocity Smoother → cmd_vel
```

项目 README 写的是默认 Global Planner，但当前仿真和实车配置实际明确使用了 ThetaStarPlanner（见 `src/srm27_navigation/srm27_nav_bringup/config/simulation/nav2_params.yaml`）。

## 当前局部控制流程

1. ControllerServer 以 20 Hz 调用自定义控制器（见 `src/srm27_navigation/srm27_nav_bringup/config/simulation/nav2_params.yaml`）。

2. 将全局路径变换到 gimbal_yaw_fake 坐标系，只保留局部代价地图范围内的部分，并删除已经走过的路径点（见 `src/srm27_navigation/srm27_omni_pid_controller/src/omni_pid_pursuit_controller.cpp`）。

3. 根据速度计算前视距离：

   ```text
   lookahead = clamp(|v_xy| × lookahead_time, min, max)
   ```

   当前配置为 0.5～1.0 m、前视时间 1 s。

4. 找到前视点 carrot point，计算它相对机器人原点的距离和方向。

5. 用 PID 根据距离生成一个标量线速度，再投影成全向底盘速度：

   ```text
   vx = v × cos(theta)
   vy = v × sin(theta)
   ```

   因此它支持横移，适合全向底盘（见 `src/srm27_navigation/srm27_omni_pid_controller/src/omni_pid_pursuit_controller.cpp`）。

6. 根据三点拟合曲率，在弯道减速；接近目标时进一步减速。

7. 从局部路径中抽取 10 个点进行代价地图碰撞检查；若碰撞则抛出异常，交给 Nav2 的恢复行为或重新规划（见 `src/srm27_navigation/srm27_omni_pid_controller/src/omni_pid_pursuit_controller.cpp`）。

所以它本质上是：

> 纯追踪 / 前视跟踪 + PID + 曲率限速 + 路径点碰撞检查

而不是“在局部窗口里搜索一条新路径”。

当前配置中：

- `enable_rotation: false`
- `use_rotate_to_heading: false`
- `yaw_goal_tolerance: 6.28`

因此实际运行时基本只控制平移，不控制最终朝向（见 `src/srm27_navigation/srm27_nav_bringup/config/simulation/nav2_params.yaml`）。

## 优点

- 计算量小，确定性强，20 Hz 运行容易。
- 参数少，调试直观。
- 对全向底盘可以直接输出 vx/vy。
- 路径跟踪平滑，适合静态地图和开阔赛场。
- 与当前 Nav2 插件体系集成简单。

## 主要局限

- 不会主动绕开动态障碍物；遇到障碍通常是停下、恢复或触发全局重规划。
- 碰撞检查的是全局路径上的 10 个采样点，不是根据实际 cmd_vel 前向仿真的连续轨迹。
- 如果路径点之间很稀疏，或者机器人跟踪时切弯，安全性可能不足。
- `worldToMap()` 失败时直接返回“无碰撞”，存在越界安全风险。
- 控制器自身没有显式加速度约束，主要依赖后面的 `velocity_smoother`。
- `v_linear_min` 虽允许负值，但当前距离误差始终为正，实际通常不会主动倒车。
- `min_max_sum_error` 参数在代码中没有真正用于 PID；PID 积分限幅实际写死为 ±1。
- 如果以后开启旋转控制，最好补充 yaw 误差归一化到 `[-pi, pi]`。

## 和其他方案的区别

| 方案 | 基本思想 | 障碍绕行 | 全向支持 | 平滑性 | 计算量/调参 |
| --- | --- | --- | --- | --- | --- |
| 当前 Omni PID Pursuit | 跟踪已有路径，选前视点输出速度 | 弱 | 好 | 较好 | 低/低 |
| DWB | 采样一组 (vx, vy, omega)，前向模拟并评分 | 中 | 好 | 中等 | 中/较高 |
| MPPI | 大量采样控制序列，按代价选择最优控制 | 强 | 好 | 很好 | 高/中高 |
| RPP | 几何纯追踪加曲率、障碍和接近目标限速 | 弱 | 通常偏差速底盘 | 好 | 很低/低 |
| MPC | 显式建立动力学模型并在线优化 | 强 | 好 | 很好 | 很高/很高 |

### DWB

Nav2 原生、工程风险最低。它会在动态窗口内采样速度并模拟短轨迹，再按障碍物、路径距离、目标距离、速度等评分。适合作为第一个局部避障基线。

缺点是速度采样离散，可能出现振荡、局部最优，且全向底盘需要合理配置 vx/vy/omega 采样数量。

### MPPI

Nav2 有原生 nav2_mppi_controller。它会对未来一段时间的控制序列进行大量随机采样，用代价函数选择当前最优控制。相比 DWB，它能更自然地处理全向运动、非线性动力学、速度/加速度约束和复杂代价。

critic，并不是自动拥有完美的目标预测。

## 对本项目的建议

- 如果希望低风险、低算力、先建立稳定基线：先试 DWB。
- 如果需要严格满足底盘动力学和高性能控制，再考虑自定义 MPC。

对这个全向哨兵底盘，实际优先级通常是：

> MPPI > DWB

其中 MPPI 更适合最终方案，DWB 更适合作为容易落地的对照组。
