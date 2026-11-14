#ifndef SRM27_NAV_PROTOCOL__PACKET_TYPEDEF_HPP_
#define SRM27_NAV_PROTOCOL__PACKET_TYPEDEF_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>
#include <iostream>
#include <algorithm>

namespace srm27_nav_protocol
{
// V1.0.0 协议规定 SOF 为 0xA5
const uint8_t SOF_RECEIVE = 0xA5;
const uint8_t SOF_SEND = 0xA5;

/********************************************************/
/* ID define (New Protocol 2-byte IDs)                  */
/********************************************************/
// Receive IDs
const uint16_t ID_GAME_STATUS = 0x0001;            // 比赛状态数据
const uint16_t ID_ALL_ROBOT_HP = 0x0003;           // 机器人血量数据 (文档表1-4定义0x0003为机器人血量)
const uint16_t ID_EVENT_DATA = 0x0101;             // 场地事件数据
const uint16_t ID_ROBOT_STATUS = 0x0202;           // 实时底盘缓冲能量和射击热量数据
const uint16_t ID_PROJECTILE_ALLOWANCE = 0x000E;   // 允许发弹量数据
const uint16_t ID_RFID_STATUS = 0x0209;            // 机器人RFID状态
const uint16_t ID_GROUND_ROBOT_POSITION = 0x020B;  // 地面机器人位置数据
const uint16_t ID_SWITCH_POSITION = 0x000D;        // 打前哨战与否标志位
const uint16_t ID_SEFDEFINED = 0x000B;             // 新定义的0x0B数据包
const uint16_t ID_ROBOT_STATUS_V1 = 0x0201;        // 机器人状态数据 (兼容 ID 0x0201)
const uint16_t ID_CHASSIS_SPEED = 0x0010;          // 底盘速度反馈（融合结果 speed->fused）

// Send ID
// 0x0302: 自定义控制器与机器人交互数据
const uint16_t ID_ROBOT_CMD = 0x0302;

/********************************************************/
/* HeaderFrame                                           */
/********************************************************/

// 表1-2 frame_header 格式
// SOF(1) + data_length(2) + seq(1) + CRC8(1) = 5 bytes
struct HeaderFrame
{
  uint8_t sof;
  uint16_t data_length;
  uint8_t seq;
  uint8_t crc;
} __attribute__((packed));

/********************************************************/
/* Receive data Structures                              */
/********************************************************/

// 0x0001 比赛状态数据
struct ReceiveGameStatusData
{
  HeaderFrame frame_header;
  uint16_t cmd_id;

  struct
  {
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t sync_time_stamp;
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

// 0x0003 机器人血量数据
struct ReceiveAllRobotHpData
{
  HeaderFrame frame_header;
  uint16_t cmd_id;

  struct
  {
    uint16_t ally_1_robot_hp;
    uint16_t ally_2_robot_hp;
    uint16_t ally_3_robot_hp;
    uint16_t ally_4_robot_hp;
    uint16_t reserved;
    uint16_t ally_7_robot_hp;
    uint16_t ally_outpost_hp;
    uint16_t ally_base_hp;
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

// 0x0101 场地事件数据
struct ReceiveAllRobotHp2027
{
  HeaderFrame frame_header;
  uint16_t cmd_id;
  struct
  {
    uint16_t ally_1_robot_hp, reserved, ally_3_robot_hp, ally_4_robot_hp;
    int16_t damage_difference;
    uint16_t ally_7_robot_hp, red_outpost_hp, ally_base_hp, blue_outpost_hp;
    uint16_t enemy_base_hp, ally_control_degree, enemy_control_degree;
  } __attribute__((packed)) data;
  uint16_t crc;
} __attribute__((packed));

struct ReceiveEventData2027
{
  HeaderFrame frame_header;
  uint16_t cmd_id;
  struct { uint32_t event_data, event_data_2; } __attribute__((packed)) data;
  uint16_t crc;
} __attribute__((packed));
static_assert(sizeof(ReceiveAllRobotHp2027) == 33);
static_assert(sizeof(ReceiveEventData2027) == 17);

struct ReceiveEventData
{
  HeaderFrame frame_header;
  uint16_t cmd_id;

  struct
  {
    uint32_t event_data;
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

// 0x0202 实时底盘缓冲能量和射击热量数据
// 替代旧的 0x0002/RobotStatus
struct ReceiveRobotStatus
{
  HeaderFrame frame_header;
  uint16_t cmd_id;

  struct
  {
    uint16_t reserved_1;
    uint16_t reserved_2;
    float reserved_3;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_1_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

// 0x000E 允许发弹量数据
struct ReceiveProjectileAllowance
{
  HeaderFrame frame_header;
  uint16_t cmd_id;
  struct
  {
    uint16_t projectile_allowance_17mm;  // 机器人自身拥有的17mm弹丸允许发弹量
    uint16_t projectile_allowance_42mm;  // 42mm弹丸允许发弹量
    uint16_t remaining_gold_coin;        // 剩余金币数量
    uint16_t projectile_allowance_fortress;  // 堡垒增益点提供的储备17mm弹丸允许发弹量
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

// 0x000D 打前哨战与否标志位
struct ReceiveSwitchPosition
{
  HeaderFrame frame_header;
  uint16_t cmd_id;
  struct
  {
    uint8_t switch_position;  // 0=未打完, 1=已打完
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

// 0x0209 机器人 RFID 模块状态
struct ReceiveRfidStatus
{
  HeaderFrame frame_header;
  uint16_t cmd_id;

  struct
  {
    uint32_t rfid_status;
    uint8_t rfid_status_2;
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

// 0x020B 地面机器人位置数据
struct ReceiveGroundRobotPosition
{
  HeaderFrame frame_header;
  uint16_t cmd_id;

  struct
  {
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float reserved_1;
    float reserved_2;
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

// 0x000B 自定义数据包
struct ReceiveSefdefinedData
{
  HeaderFrame frame_header;
  uint16_t cmd_id;

  struct {
    uint8_t robot_id;  // 本机器人ID
    uint8_t robot_level;	// 机器人等级
    uint16_t current_HP;	// 机器人当前血量
    uint16_t maximum_HP;	// 机器人血量上限
    uint16_t shooter_barrel_cooling_value;  // 机器人射击热量每秒冷却值
    uint16_t shooter_barrel_heat_limit;     // 机器人射击热量上限
    uint16_t chassis_power_limit;           // 机器人底盘功率上限
    uint8_t power_management_gimbal_output : 1;  // bit 0：gimbal口输出，0为无输出，1为 24V输出
    uint8_t power_management_chassis_output : 1;  // bit 1：chassis口输出，0为无输出，1为24V输出
    uint8_t power_management_shooter_output : 1;  // bit 2：shooter口输出，0为无输出，1为24V输出
  } __attribute__((__packed__))  data;

  uint16_t crc;
} __attribute__((packed));

// 0x0010 底盘/云台速度反馈（下位机速度观测器的**融合结果** speed->fused）
//
// 语义（下位机口径，别自己重新解释）:
//   * **vx/vy 是云台系**（+x = 云台前、+y = 云台左）：2026-11-08 起下位机按云台的
//     "前"执行 vx/vy，回传也用同一个系（方案 §4.6）。
//   * **wz 是底盘转速**（俯视逆时针为正）：下发的 wz 转的是底盘，不是云台。
//   * 单位：vx / vy 为 m/s，wz 为 rad/s，chassis_to_gimbal_yaw 为**度**，两路云台转速
//     为 rad/s。
//   * 第 4 个字段是 **chassis_to_gimbal_yaw**（单位**度**，不是弧度），定义见字段注释：
//     它描述的是"底盘车头相对云台"的偏角，**不是**底盘在世界系/上电零位系里的 yaw，
//     与 IMU、上电零位都无关，由云台 yaw 电机编码器解算。
//   * 这是**观测值**，不是"指令已执行"的回显：指令 → 底盘 → 观测存在延迟与尺度差，
//     拿它当"下位机收到了我的速度"的证据会误判。它的用途是反馈/对账/标定。
//   * 该包与 0x0001 / 0x000B 按 10 ms 槽位轮转（各约 33.3 Hz），按 cmd_id 分派，
//     不能按"第几个包"解析；槽位被跳过的轮次该包直接丢失（不是重发）。
//
// 2026-11-08（方案 §4.4.1）：数据段 16 -> **20 字节**，末尾追加**一路**云台 yaw 转速
//   （gimbal_yaw_rate_imu，陀螺仪、相对地面）。上下位机要同时更新；
//   接收侧按 `data_length` 区分新旧帧：16 = 旧帧（转速不可用，不报错），20 = 新帧。
//
// ⚠ 只加**陀螺仪**转速，不加编码器转速：编码器转速就是第 4 个字段
//   `chassis_to_gimbal_yaw` 的变化率，上位机对它求导即可，多发一个 float 只是把
//   同样的信息再搬一遍（2026-11-08 决定）。
constexpr uint16_t DATA_LENGTH_CHASSIS_SPEED = 20;
/// rief 旧帧（2026-11-08 之前）的数据段长度：只有 vx/vy/wz/chassis_to_gimbal_yaw。
constexpr uint16_t DATA_LENGTH_CHASSIS_SPEED_V1 = 16;

struct ReceiveChassisSpeedData
{
  HeaderFrame frame_header;
  uint16_t cmd_id;

  struct
  {
    float vx;  // m/s, 云台系 +x 向前（云台的"前"）
    float vy;  // m/s, 云台系 +y 向左
    float wz;  // rad/s, **底盘**自转，俯视逆时针为正
    // 底盘相对云台的偏角，单位**度**，范围 (-180, 180]，俯视逆时针为正。
    // 定义：以**云台指向**为 0，底盘车头相对云台偏转了多少。
    //   · 云台不动、底盘被推着逆时针转 45° → 从云台看车头在左边 → +45；
    //   · 与 IMU、上电零位、世界系都无关，只来自云台 yaw 电机编码器；
    //   · URDF 的 gimbal_yaw_joint 是反过来的"云台相对底盘"且用弧度，
    //     所以 joint_rad = -chassis_to_gimbal_yaw_deg * π / 180（换算在 cpp 里做）。
    float chassis_to_gimbal_yaw_deg;
    // 云台陀螺仪 yaw 转速（rad/s，**相对地面**，俯视逆时针为正）。
    // 用途：fake_vel_transform 的 yaw 外推（方案 §4.4.2 的 gimbal_rate 源）。
    // ⚠ 正方向定义电控也不确定，必须先按方案 §8.1 上车实测，不要靠约定。
    // 编码器转速（云台相对底盘）不单独回传：它 = 上面 chassis_to_gimbal_yaw 的变化率。
    float gimbal_yaw_rate_imu;
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

// 旧帧：只有前 4 个字段。单独一个结构体是**必须**的 ——
// `fromVector<T>()` 在数据短于结构体时会 stderr 警告并补零，拿新结构体去读 25 字节旧帧
// 会每帧刷一行警告（33 Hz），而且分不清"字段真的是 0"还是"帧里没有这个字段"。
struct ReceiveChassisSpeedDataV1
{
  HeaderFrame frame_header;
  uint16_t cmd_id;

  struct
  {
    float vx;
    float vy;
    float wz;
    float chassis_to_gimbal_yaw_deg;
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

static_assert(
  sizeof(ReceiveChassisSpeedData) == 9 + DATA_LENGTH_CHASSIS_SPEED,
  "ReceiveChassisSpeedData 必须严格 29 字节: 帧头 5 + cmd_id 2 + 数据 20 + CRC16 2。");
static_assert(
  sizeof(ReceiveChassisSpeedDataV1) == 9 + DATA_LENGTH_CHASSIS_SPEED_V1,
  "ReceiveChassisSpeedDataV1 必须严格 25 字节: 帧头 5 + cmd_id 2 + 数据 16 + CRC16 2。");
static_assert(
  offsetof(ReceiveChassisSpeedData, data) == 7,
  "0x0010 的数据段必须紧跟在 cmd_id 之后（协议 §4.1: 帧头 5 + cmd_id 2）。");
static_assert(
  offsetof(ReceiveChassisSpeedData, data.gimbal_yaw_rate_imu) == 7 + 16,
  "云台转速必须紧跟在旧的 4 个 float32 之后（数据段偏移 16）。");

/********************************************************/
/* Send data                                            */
/********************************************************/

// [协议 §2.1] 上→下 数据段长度固定 12 字节, 整帧固定 19 字节。
// 这个方向【没有】cmd_id —— 帧头 5 字节之后直接就是数据段。
constexpr uint16_t DATA_LENGTH_CMD = 12;
constexpr std::size_t FRAME_LENGTH_CMD = 5 + DATA_LENGTH_CMD + 2;   // 19

struct SendRobotCmdData
{
  HeaderFrame frame_header;

  // 数据段: 只有 vx / vy / wz 三个 float32
  struct
  {
    float vx;
    float vy;
    float wz;
  } __attribute__((packed)) speed_vector;

  uint16_t checksum;
} __attribute__((packed));

// [协议 §2.1] 用静态断言把帧长钉死。
// 原实现里数据段多了一个 `uint8_t is_recovering;` 决策状态字节, 使
//   sizeof(SendRobotCmdData) = 20 -> data_length = 20-5-2 = 13, 整帧 20 字节,
// 而下位机只接受 data_length == 12, 于是整帧被静默丢弃(CRC 全对也没用),
// 表现就是"软件链路全对但车不动"。
// 有了这个断言, 以后谁再往数据段里加字段会在编译期直接失败,
// 而不是发出一个下位机不认的帧。
static_assert(
  sizeof(SendRobotCmdData) == FRAME_LENGTH_CMD,
  "SendRobotCmdData 必须严格 19 字节: 数据段只能是 vx/vy/wz 三个 float32。"
  "加字段会让 data_length != 12, 下位机会整帧丢弃 (协议 §2.1)。");

// [协议 §4.1] 下→上 整帧 = 9 + data_length, 所以某个包的数据段长度
// 应当等于 sizeof(T) - 9。用于接收侧长度自检, 避免把包认错。
template <typename T>
constexpr bool receive_length_matches(uint16_t data_length)
{
  return data_length == sizeof(T) - 9;
}

/********************************************************/
/* template                                             */
/********************************************************/

template <typename T>
inline T fromVector(const std::vector<uint8_t>& data)
{
  // 1. 初始化结构体：使用 {} 进行零初始化，防止部分拷贝导致的垃圾值
  T packet = {};

  // 或者使用 memset 强制清零（适用于纯 C 风格结构体）
  // std::memset(&packet, 0, sizeof(T));

  // 2. 计算拷贝长度：保护不越界
  size_t copy_len = std::min(data.size(), sizeof(T));

  // 3. 执行拷贝
  // 提示：如果 data.size() < sizeof(T)，这只是部分数据有效，最好加个日志警告
  if (data.size() < sizeof(T))
  {
    std::cerr << "Warning: Data size less than struct size!" << std::endl;
  }

  std::copy(data.begin(), data.begin() + copy_len, reinterpret_cast<uint8_t*>(&packet));

  return packet;
}

template <typename T>
inline std::vector<uint8_t> toVector(const T& data)
{
  std::vector<uint8_t> packet(sizeof(T));

  // 使用 const_cast 并非必须，但 reinterpret_cast<const uint8_t*> 是最准确的
  const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&data);

  std::copy(ptr, ptr + sizeof(T), packet.begin());

  return packet;
}

}  // namespace srm27_nav_protocol

#endif  // SRM27_NAV_PROTOCOL__PACKET_TYPEDEF_HPP_
