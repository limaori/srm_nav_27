# rm_decision_interfaces

本工作区统一的 SRM 消息接口包，位于 `src/interfaces/rm_decision_interfaces`。串口驱动、行为树和键盘云台工具都使用此包。原 `pb_rm_interfaces` 已合并；重新构建后需 `source install/setup.bash`。消息变更要求所有发布/订阅节点一起重建，旧工作区二进制和旧 rosbag 不会自动转换类型。

## 消息与使用位置

| 消息 | 话题 | 使用方式 |
|---|---|---|
| `GameStatus` | `referee/game_status` | 串口发布，行为树订阅 |
| `EventData` | `referee/event_data` | 串口发布，行为树订阅 |
| `AllyRobotHP` | `referee/ally_robot_hp` | 串口发布，行为树订阅；当前协议只有己方血量 |
| `GroundRobotPosition` | `referee/ground_robot_position` | 串口发布，行为树订阅 |
| `RfidStatus` | `referee/rfid_status` | 串口发布，行为树订阅 |
| `RobotStatus` | `referee/robot_status` | 串口汇总发布，行为树订阅 |
| `ProjectileAllowance` | `referee/projectile_allowance` | 串口发布 |
| `SwitchPosition` | `referee/switch_position` | 串口发布 |
| `Sefdefined` | `/srm/sefdefined` | 原始身份、性能和电源状态 |
| `RobotControl` | `robot_control` | 串口订阅；当前只发送 `is_recovering` 字段 |
| `GimbalCmd` / `Gimbal` | `cmd_gimbal` 等 | 行为树和键盘工具发布 |
| `Buff` | `referee/buff` | 行为树可订阅；当前 SRM 串口未发布 |

`GameRobotHP` 保留完整红蓝血量格式，原相同字段的 `AllRobotHP` 合并为此类型。它与只含己方血量的 `AllyRobotHP` 含义不同。`FriendLocation`、`RFID`、`DecisionNum`、`Models`、`RobotStateInfo` 保留供其他功能使用。

## 字段统一规则

`EventData`、`GameStatus`、`GroundRobotPosition` 和 `RfidStatus` 采用 SRM 字段与类型。行为树 RFID 输入端口名称保持原样，内部改用 `friendly_fortress_gain`、`friendly_supply_zone_non_overlap`、`friendly_supply_zone_overlap`、`center_gain_point_rmul`。

`RobotStatus` 保留 SRM 的 `shooter_heat`、`team_color`、`is_attacked`，增加行为树需要的性能、弹药和有效标志。串口收到身份/性能（`0x000B` 或 `0x0201`）、热量（`0x0202`）和弹药（`0x000E`）报文时更新相应字段，其他已知字段保持不变。`status_valid`、`heat_valid`、`ammo_valid` 表示本次运行已收到对应报文，不代表数据永不过期。

`IsStatusOK` 要求身份/血量和热量有效；弹药阈值大于零时还要求弹药有效。当前串口没有装甲伤害数据来源，`damage_info_valid` 默认无效，`IsAttacked` 不会将默认装甲编号误判为攻击方向。

合并不改变下位机数据包结构、CRC 和速度指令格式。合入的 PB 消息版权许可见 `LICENSE.pb_rm_interfaces`。
