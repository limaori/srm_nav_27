# EventData.msg

# --- 补给区状态 ---
uint8 supply_zone_non_overlap      # 己方与资源区不重叠的补给区
uint8 supply_zone_overlap          # 己方与资源区重叠的补给区
uint8 supply_zone_rmul             # 己方补给区占领状态 (仅 RMUL)

# --- 能量机关状态 ---
uint8 small_energy_status          # 己方小能量机关状态
uint8 big_energy_status            # 己方大能量机关状态

# --- 高地占领状态 ---
uint8 central_highland_status      # 环形/中央高地占领状态
uint8 trapezoidal_highland_status  # 己方梯形高地占领状态

# --- 飞镖相关 ---
uint16 dart_last_hit_time          # 对方飞镖最后一次击中时间
uint8 dart_last_hit_target         # 对方飞镖最后一次击中目标

# --- 增益点状态 ---
uint8 center_gain_point_status     # 中心增益点 (仅 RMUL)
uint8 fortress_gain_point_status   # 己方堡垒增益点
uint8 outpost_gain_point_status    # 己方前哨站增益点
uint8 base_gain_point_status       # 己方基地增益点

# 2027 V1.0.0 表 1-9。数组顺序固定为 [R, B]；状态均相对于接收方队伍。
# 仅 8 字节原生场地事件有效；旧 4 字节转发包不得推断阵地控制关系。
uint8 FULL_CONTROL=0
uint8 TEMPORARY_CONTROL=1
uint8 NO_CONTROL=2
bool battlefield_valid
uint8[2] outpost_control
uint8[2] base_control
uint8[2] highland_activation
uint8[2] outpost_activation
uint8[2] fortress_activation
uint8[2] base_activation
