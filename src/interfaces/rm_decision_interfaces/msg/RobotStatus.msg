# SRM 机器人状态；由身份/血量、热量和弹丸余量报文聚合。
uint8 robot_id
uint16 current_hp
uint16 shooter_heat
bool team_color                 # false=红，true=蓝，仅在 status_valid 时有效
bool is_attacked                 # 仅在 damage_info_valid 时表示装甲被击中

uint8 robot_level
uint16 maximum_hp
uint16 shooter_barrel_cooling_value
uint16 shooter_barrel_heat_limit
uint16 chassis_power_limit
uint16 projectile_allowance_17mm
uint16 remaining_gold_coin

# 未接收的报文不应被当成零值数据用于决策。
bool status_valid
bool heat_valid
bool ammo_valid

# 装甲伤害信息。当前 SRM 串口未解析伤害报文，保持无效。
uint8 ARMOR_HIT=0
uint8 SYSTEM_OFFLINE=1
uint8 OVER_SHOOT_SPEED=2
uint8 OVER_HEAT=3
uint8 OVER_POWER=4
uint8 ARMOR_COLLISION=5
bool damage_info_valid
uint8 armor_id
uint8 hp_deduction_reason
