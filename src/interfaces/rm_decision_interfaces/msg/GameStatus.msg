# 比赛阶段常量；数据字段沿用 SRM 串口协议。
uint8 NOT_START=0
uint8 PREPARATION=1
uint8 SELF_CHECKING=2
uint8 COUNT_DOWN=3
uint8 RUNNING=4
uint8 GAME_OVER=5

uint8 game_type         # 新增：比赛类型
uint8 game_progress     # 现有：比赛阶段
uint16 stage_remain_time # 现有：剩余时间
uint64 sync_time_stamp  # 新增：UNIX 时间戳 (注意使用 uint64)
