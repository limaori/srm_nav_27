# 对应下位机的 ReceiveAllRobotHpData 结构体中的 data 部分
uint16 ally_1_robot_hp
uint16 ally_2_robot_hp
uint16 ally_3_robot_hp
uint16 ally_4_robot_hp
# 5 reserved
uint16 ally_7_robot_hp  # 哨兵通常是 7 号
uint16 ally_outpost_hp
uint16 ally_base_hp

# 2027 V1.0.0 表 1-8；旧 16 字节下位机报文没有这些字段。
bool control_score_valid
uint16 red_outpost_hp
uint16 blue_outpost_hp
uint16 enemy_base_hp
uint16 ally_control_degree
uint16 enemy_control_degree
