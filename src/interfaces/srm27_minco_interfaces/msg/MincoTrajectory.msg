# 规划器（MincoPlanner）一次 MINCO 求解的完整结果。
#
# 为什么用"采样点 + 时间 + 速度"而不是多项式系数：
#   * 控制器只需要按时间取 (位置, 速度) 作为 MPC 参考，不需要系数；
#   * 系数矩阵跨 MINCO 实现版本不稳定，而采样点对发布方/订阅方都是黑盒；
#   * 采样步长由发布方决定（默认 0.05 s），订阅方线性插值即可。
#
# 时间基准：header.stamp 是生成时刻，t[] 是**相对** header.stamp 的偏移（从 0 递增）。

std_msgs/Header header          # header.frame_id = 轨迹所在坐标系（odom），stamp = 生成时刻

uint64 path_version             # 对应的全局路径版本；换路径就变
uint64 goal_epoch              # 对应目标会话；换目标就变

float64 duration                # 轨迹总时长（s），必须等于 t[] 的末元素
bool terminal_is_global_goal    # 末端是否就是全局目标（是则末端速度必须为 0）
float64 terminal_speed          # 末端速度（m/s）
string terminal_reason          # 末端类型：goal / horizon / boundary / blocked / stop

# 采样点（等长数组，按 t 递增）
float64[] t                     # 时刻（s，相对 header.stamp）
float64[] x                     # 位置 x（m）
float64[] y                     # 位置 y（m）
float64[] yaw                   # 航向（rad，未做 unwrap 时可能跳变；控制器按需插值）
float64[] vx                    # 速度 x（m/s，坐标系同 header.frame_id）
float64[] vy                    # 速度 y（m/s）

# Exact executable trajectory. Each segment uses local time and six coefficients
# per axis in ascending order. The samples above remain available for diagnostics.
float64[] piece_durations
float64[] x_coefficients
float64[] y_coefficients
builtin_interfaces/Time valid_until
bool terminal_requires_stop
