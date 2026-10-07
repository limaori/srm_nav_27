#include "srm27_nav_protocol.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "crc_func.h"
#include "packet_typedef.hpp"

// 发送线程与 cmd_vel 回调之间共享发送状态的互斥量
std::mutex send_mutex_;

#define USB_NOT_OK_SLEEP_TIME 1000   // (ms)
#define USB_PROTECT_SLEEP_TIME 1000  // (ms)

namespace srm27_nav_protocol
{

Srm27NavProtocolNode::Srm27NavProtocolNode(const rclcpp::NodeOptions & options)
: Node("Srm27NavProtocolNode", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start Srm27NavProtocolNode!");

  getParams();
  createPublisher();
  createSubscription();

  serial_port_protect_thread_ = std::thread(&Srm27NavProtocolNode::serialPortProtect, this);
  receive_thread_ = std::thread(&Srm27NavProtocolNode::receiveData, this);
  send_thread_ = std::thread(&Srm27NavProtocolNode::sendData, this);
}

Srm27NavProtocolNode::~Srm27NavProtocolNode()
{
  // 注意顺序: 必须先 join 发送线程 —— 它退出前会补发零速帧 (协议 §6.3),
  // 而这需要串口还开着。端口在下面所有线程 join 完之后才 close。
  if (send_thread_.joinable()) send_thread_.join();
  if (receive_thread_.joinable()) receive_thread_.join();
  if (serial_port_protect_thread_.joinable()) serial_port_protect_thread_.join();
  if (serial_driver_->port()->is_open()) serial_driver_->port()->close();
  if (owned_ctx_) owned_ctx_->waitForExit();
}

void Srm27NavProtocolNode::createPublisher()
{
  event_data_pub_ =
    this->create_publisher<rm_decision_interfaces::msg::EventData>("referee/event_data", 10);
  sefdefined_pub_ =
    this->create_publisher<rm_decision_interfaces::msg::Sefdefined>("/srm/sefdefined", 10);
  // all_robot_hp_pub_ = this->create_publisher<rm_decision_interfaces::msg::AllyRobotHP>("referee/ally_robot_hp", 10);
  all_robot_hp_pub_ =
    this->create_publisher<rm_decision_interfaces::msg::AllyRobotHP>("referee/ally_robot_hp", 10);
  game_status_pub_ =
    this->create_publisher<rm_decision_interfaces::msg::GameStatus>("referee/game_status", 10);
  ground_robot_position_pub_ = this->create_publisher<rm_decision_interfaces::msg::GroundRobotPosition>(
    "referee/ground_robot_position", 10);
  rfid_status_pub_ =
    this->create_publisher<rm_decision_interfaces::msg::RfidStatus>("referee/rfid_status", 10);
  robot_status_pub_ =
    this->create_publisher<rm_decision_interfaces::msg::RobotStatus>("referee/robot_status", 10);
  projectile_allowance_pub_ =
    this->create_publisher<rm_decision_interfaces::msg::ProjectileAllowance>("referee/projectile_allowance", 10);
  switch_position_pub_ =
    this->create_publisher<rm_decision_interfaces::msg::SwitchPosition>("referee/switch_position", 10);
}

void Srm27NavProtocolNode::createSubscription()
{
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_chassis", 10,
    std::bind(&Srm27NavProtocolNode::CmdVelCallback, this, std::placeholders::_1));
  // 移除了 robot_control 订阅 (is_recovering 在标准 19 字节帧里没有位置, 见 hpp 注释)
}

void Srm27NavProtocolNode::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");
    if (fc_string == "none")
      fc = FlowControl::NONE;
    else if (fc_string == "hardware")
      fc = FlowControl::HARDWARE;
    else if (fc_string == "software")
      fc = FlowControl::SOFTWARE;
    else
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");
    if (pt_string == "none")
      pt = Parity::NONE;
    else if (pt_string == "odd")
      pt = Parity::ODD;
    else if (pt_string == "even")
      pt = Parity::EVEN;
    else
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");
    if (sb_string == "1" || sb_string == "1.0")
      sb = StopBits::ONE;
    else if (sb_string == "1.5")
      sb = StopBits::ONE_POINT_FIVE;
    else if (sb_string == "2" || sb_string == "2.0")
      sb = StopBits::TWO;
    else
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);

  try {
    debug_print_hex_ = declare_parameter<bool>("debug_print_hex", false);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The debug_print_hex provided was invalid");
    throw ex;
  }

  // ---- 协议 §3.3 / §6 相关参数 ----
  send_rate_hz_ = declare_parameter<double>("send_rate_hz", 100.0);
  cmd_timeout_sec_ = declare_parameter<double>("cmd_timeout_sec", 0.5);
  max_vx_ = declare_parameter<double>("max_vx", 0.5);
  max_vy_ = declare_parameter<double>("max_vy", 0.5);
  max_wz_ = declare_parameter<double>("max_wz", 1.0);

  if (!(send_rate_hz_ > 0.0)) {
    throw std::invalid_argument{"send_rate_hz 必须 > 0"};
  }
  // 19 字节在 115200 baud 下约占 1.65 ms, 超过 200 Hz 就没有帧间空闲,
  // 会退化成背靠背连发, 违反协议 §6.2。推荐 100 Hz (§6.1)。
  if (send_rate_hz_ > 200.0) {
    throw std::invalid_argument{"send_rate_hz 不能超过 200 (协议 §6.1/§6.2: 推荐 100 Hz 且帧间需空闲)"};
  }
  if (!(cmd_timeout_sec_ > 0.0)) {
    throw std::invalid_argument{"cmd_timeout_sec 必须 > 0"};
  }
  if (!(max_vx_ > 0.0) || !(max_vy_ > 0.0) || !(max_wz_ > 0.0)) {
    throw std::invalid_argument{"max_vx / max_vy / max_wz 必须 > 0"};
  }

  RCLCPP_INFO(
    get_logger(),
    "协议参数: 上→下 %zu 字节 (data_length=%u), 发送 %.1f Hz, 看门狗 %.0f ms, "
    "限幅 vx<=%.3f vy<=%.3f wz<=%.3f",
    FRAME_LENGTH_CMD, static_cast<unsigned>(DATA_LENGTH_CMD), send_rate_hz_,
    cmd_timeout_sec_ * 1000.0, max_vx_, max_vy_, max_wz_);
}

void Srm27NavProtocolNode::serialPortProtect()
{
  RCLCPP_INFO(get_logger(), "Start serialPortProtect!");
  serial_driver_->init_port(device_name_, *device_config_);
  try {
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      RCLCPP_INFO(get_logger(), "Serial port opened!");
      is_usb_ok_ = true;
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Open serial port failed : %s", ex.what());
    is_usb_ok_ = false;
  }
  // [修复] 原来这里有一行无条件 `is_usb_ok_ = true;`, 会把上面 catch 里的 false 覆盖掉。
  // 后果: 端口一旦打开失败(或之后失效), 下面的重试分支 `if (!is_usb_ok_)` 永远进不去,
  // 于是【永久静默失败】—— 不再重试、不再报错、也不再自愈(拔插 USB 后必须重启节点)。
  // 删除该行, 让打开失败时能每秒重试并打印错误。
  std::this_thread::sleep_for(std::chrono::milliseconds(USB_PROTECT_SLEEP_TIME));

  while (rclcpp::ok()) {
    if (!is_usb_ok_) {
      try {
        if (serial_driver_->port()->is_open()) serial_driver_->port()->close();
        serial_driver_->port()->open();
        if (serial_driver_->port()->is_open()) {
          RCLCPP_INFO(get_logger(), "Serial port opened!");
          is_usb_ok_ = true;
        }
      } catch (const std::exception & ex) {
        is_usb_ok_ = false;
        RCLCPP_ERROR(get_logger(), "Open serial port failed : %s", ex.what());
      }
    };
    std::this_thread::sleep_for(std::chrono::milliseconds(USB_PROTECT_SLEEP_TIME));
  }
}

/********************************************************/
/* Receive data                                         */
/********************************************************/

void Srm27NavProtocolNode::receiveData()
{
  RCLCPP_INFO(get_logger(), "Start receiveData with Sliding Window!");

  int retry_count = 0;
  std::deque<uint8_t> rx_buffer;
  std::vector<uint8_t> read_buf;
  read_buf.reserve(1024);

  while (rclcpp::ok()) {
    if (!is_usb_ok_) {
      RCLCPP_WARN(get_logger(), "receive: usb is not ok! Retry count: %d", retry_count++);
      std::this_thread::sleep_for(std::chrono::milliseconds(USB_NOT_OK_SLEEP_TIME));
      rx_buffer.clear();
      continue;
    }

    try {
      // 1. 从串口读取所有可用数据
      read_buf.resize(1024);
      int received_len = 0;
      try {
        received_len = serial_driver_->port()->receive(read_buf);
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(get_logger(), "Error reading from serial port: %s", ex.what());
        is_usb_ok_ = false;
        continue;
      }

      if (received_len > 0) {
        // 高效插入 deque
        rx_buffer.insert(rx_buffer.end(), read_buf.begin(), read_buf.begin() + received_len);
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }

      // 2. 滑动窗口解析
      while (rx_buffer.size() >= 5) {
        // 查找帧头 SOF
        if (rx_buffer.front() != SOF_RECEIVE) {
          rx_buffer.pop_front();
          continue;
        }

        // 提取前5字节 Header 用于 CRC8 校验
        std::vector<uint8_t> header_bytes(rx_buffer.begin(), rx_buffer.begin() + 5);
        if (!verify_CRC8_check_sum(header_bytes.data(), 5)) {
          // 伪造包头，丢弃1个字节，继续滑动
          RCLCPP_WARN(get_logger(), "Receive Header CRC8 FAIL! Dropping 1 byte.");
          rx_buffer.pop_front();
          continue;
        }

        HeaderFrame header_frame = fromVector<HeaderFrame>(header_bytes);
        
        // 检查 data_length 是否合法，防止恶意数据耗尽内存
        if (header_frame.data_length > 2048) {
          RCLCPP_WARN(get_logger(), "Invalid data length: %d", header_frame.data_length);
          rx_buffer.pop_front();
          continue;
        }

        // 完整包所需长度：Header(5) + CmdID(2) + Data(data_length) + CRC16(2)
        size_t total_packet_len = 5 + 2 + header_frame.data_length + 2;

        // 断帧处理：长度不够，跳出循环等待更多数据
        if (rx_buffer.size() < total_packet_len) {
          break;
        }

        // 提取全包用于 CRC16 校验
        std::vector<uint8_t> full_packet(rx_buffer.begin(), rx_buffer.begin() + total_packet_len);
        if (!verify_CRC16_check_sum(full_packet.data(), total_packet_len)) {
          // 校验失败，可能是错位或者丢包，丢弃头部 1 个字节，继续滑动查找下一个 SOF
          RCLCPP_WARN(get_logger(), "Receive CRC16 FAIL! Dropping 1 byte and sliding.");
          rx_buffer.pop_front();
          continue;
        }

        // --- 成功解析出完整帧 ---
        // 从 rx_buffer 中移除已被解析的完整帧
        rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + total_packet_len);

        // 获取 CmdID，小端序解析
        uint16_t cmd_id = static_cast<uint16_t>(full_packet[5]) | (static_cast<uint16_t>(full_packet[6]) << 8);

        // 打印调试信息
        if (debug_print_hex_) {
          printHex("RECV", cmd_id, full_packet);
        }

        // 8. 解析数据
        switch (cmd_id) {
          case ID_EVENT_DATA: {
            ReceiveEventData event_data = fromVector<ReceiveEventData>(full_packet);
            publishEventData(event_data);
          } break;
          case ID_ALL_ROBOT_HP: {
            ReceiveAllRobotHpData all_robot_hp_data = fromVector<ReceiveAllRobotHpData>(full_packet);
            publishAllRobotHp(all_robot_hp_data);
          } break;
          case ID_GAME_STATUS: {
            ReceiveGameStatusData game_status_data = fromVector<ReceiveGameStatusData>(full_packet);
            publishGameStatus(game_status_data);
          } break;
          case ID_GROUND_ROBOT_POSITION: {
            ReceiveGroundRobotPosition ground_robot_position_data =
              fromVector<ReceiveGroundRobotPosition>(full_packet);
            publishGroundRobotPosition(ground_robot_position_data);
          } break;
          case ID_RFID_STATUS: {
            ReceiveRfidStatus rfid_status_data = fromVector<ReceiveRfidStatus>(full_packet);
            publishRfidStatus(rfid_status_data);
          } break;
          case ID_ROBOT_STATUS: {
            ReceiveRobotStatus robot_status_data = fromVector<ReceiveRobotStatus>(full_packet);
            publishRobotStatus(robot_status_data);
          } break;
          case ID_PROJECTILE_ALLOWANCE: {
            ReceiveProjectileAllowance projectile_allowance_data =
              fromVector<ReceiveProjectileAllowance>(full_packet);
            publishProjectileAllowance(projectile_allowance_data);
          } break;
          case ID_SWITCH_POSITION: {
            ReceiveSwitchPosition switch_position_data = fromVector<ReceiveSwitchPosition>(full_packet);
            publishSwitchPosition(switch_position_data);
          } break;
          case ID_SEFDEFINED:
          case ID_ROBOT_STATUS_V1: {
            ReceiveSefdefinedData sefdefined_data = fromVector<ReceiveSefdefinedData>(full_packet);
            publishSefdefined(sefdefined_data);
          } break;
          default: {
            RCLCPP_DEBUG(get_logger(), "Unknown CmdID: 0x%04X", cmd_id);
          } break;
        }
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error receiving data: %s", ex.what());
      is_usb_ok_ = false;
    }
  }
}

/********************************************************/
/* Publish data                                         */
/********************************************************/
void Srm27NavProtocolNode::publishEventData(ReceiveEventData & event_data)
{
  rm_decision_interfaces::msg::EventData msg;
  // 获取 32位 事件数据
  uint32_t bits = event_data.data.event_data;

  // --- 补给区状态 (Bit 0-2) ---
  msg.supply_zone_non_overlap = (bits >> 0) & 1;  // bit 0: 己方与资源区不重叠的补给区 (1为已占领)
  msg.supply_zone_overlap = (bits >> 1) & 1;  // bit 1: 己方与资源区重叠的补给区 (1为已占领)
  msg.supply_zone_rmul = (bits >> 2) & 1;  // bit 2: 己方补给区占领状态 (仅 RMUL 适用)

  // --- 能量机关状态 (Bit 3-6) ---
  // 注意：此处占 2 个 bit，掩码应为 3 (二进制 11)
  // 状态：0为未激活，1为已激活，2为正在激活
  msg.small_energy_status = (bits >> 3) & 3;  // bit 3-4: 己方小能量机关状态
  msg.big_energy_status = (bits >> 5) & 3;    // bit 5-6: 己方大能量机关状态

  // --- 高地占领状态 (Bit 7-10) ---
  // bit 7-8: 1为被己方占领，2为被对方占领
  msg.central_highland_status = (bits >> 7) & 3;
  // bit 9-10: 己方梯形高地占领状态 (1为已占领)
  msg.trapezoidal_highland_status = (bits >> 9) & 3;

  // --- 飞镖相关 (Bit 11-22) ---
  // bit 11-19: 对方飞镖最后一次击中时间 (0-420)，共 9 bit，掩码 0x1FF (511)
  msg.dart_last_hit_time = (bits >> 11) & 0x1FF;

  // bit 20-22: 对方飞镖最后一次击中目标，共 3 bit，掩码 7 (二进制 111)
  // 0:无, 1:前哨站, 2:基地固定, 3:基地随机固定, 4:基地随机移动, 5:基地末端移动
  msg.dart_last_hit_target = (bits >> 20) & 7;

  // --- 增益点状态 (Bit 23-29) ---
  // bit 23-24: 中心增益点 (仅 RMUL) - 0:未占, 1:己方, 2:对方, 3:双方
  msg.center_gain_point_status = (bits >> 23) & 3;

  // bit 25-26: 己方堡垒增益点 - 0:未占, 1:己方, 2:对方, 3:双方
  msg.fortress_gain_point_status = (bits >> 25) & 3;

  // bit 27-28: 己方前哨站增益点 - 0:未占, 1:己方, 2:对方
  msg.outpost_gain_point_status = (bits >> 27) & 3;

  // bit 29: 己方基地增益点 - 1为已占领
  msg.base_gain_point_status = (bits >> 29) & 1;

  // bit 30-31: 保留位，无需读取

  event_data_pub_->publish(msg);
}

void Srm27NavProtocolNode::publishAllRobotHp(ReceiveAllRobotHpData & all_robot_hp)
{
  // rm_decision_interfaces::msg::AllyRobotHP msg;  // 使用新消息类型
  rm_decision_interfaces::msg::AllyRobotHP msg;

  msg.ally_1_robot_hp = all_robot_hp.data.ally_1_robot_hp;
  msg.ally_2_robot_hp = all_robot_hp.data.ally_2_robot_hp;
  msg.ally_3_robot_hp = all_robot_hp.data.ally_3_robot_hp;
  msg.ally_4_robot_hp = all_robot_hp.data.ally_4_robot_hp;
  msg.ally_7_robot_hp = all_robot_hp.data.ally_7_robot_hp;  // 重点关注
  msg.ally_outpost_hp = all_robot_hp.data.ally_outpost_hp;
  msg.ally_base_hp = all_robot_hp.data.ally_base_hp;

  all_robot_hp_pub_->publish(msg);
}

void Srm27NavProtocolNode::publishGameStatus(ReceiveGameStatusData & game_status)
{
  rm_decision_interfaces::msg::GameStatus msg;

  // 1. 比赛类型 (Byte 0, Bit 0-3)
  // 1: RMUC 超级对抗赛, 2: RMUL 高校单项赛, 3: ICRA, 4: RMUL 3V3, 5: RMUL 步兵对抗
  msg.game_type = game_status.data.game_type;

  // 2. 当前比赛阶段 (Byte 0, Bit 4-7)
  // 0: 未开始, 1: 准备阶段, 2: 自检阶段, 3: 5s倒计时, 4: 比赛中, 5: 结算中
  msg.game_progress = game_status.data.game_progress;

  // 3. 当前阶段剩余时间 (Byte 1-2)
  // 单位：秒
  msg.stage_remain_time = game_status.data.stage_remain_time;

  // 4. UNIX 时间戳 (Byte 3-10)
  // 当机器人正确连接到裁判系统的 NTP 服务器后生效
  msg.sync_time_stamp = game_status.data.sync_time_stamp;

  game_status_pub_->publish(msg);
}

void Srm27NavProtocolNode::publishGroundRobotPosition(
  ReceiveGroundRobotPosition & ground_robot_position)
{
  rm_decision_interfaces::msg::GroundRobotPosition msg;
  msg.hero_x = ground_robot_position.data.hero_x;
  msg.hero_y = ground_robot_position.data.hero_y;
  msg.engineer_x = ground_robot_position.data.engineer_x;
  msg.engineer_y = ground_robot_position.data.engineer_y;
  msg.standard_3_x = ground_robot_position.data.standard_3_x;
  msg.standard_3_y = ground_robot_position.data.standard_3_y;
  msg.standard_4_x = ground_robot_position.data.standard_4_x;
  msg.standard_4_y = ground_robot_position.data.standard_4_y;
  msg.standard_5_x = 0;
  msg.standard_5_y = 0;

  ground_robot_position_pub_->publish(msg);
}

void Srm27NavProtocolNode::publishRfidStatus(ReceiveRfidStatus & rfid_status)
{
  rm_decision_interfaces::msg::RfidStatus msg;

  // 获取协议包中的数据
  // 根据图3的结构体定义：rfid_status_t 包含 uint32_t rfid_status 和 uint8_t rfid_status_2
  uint32_t status_1 = rfid_status.data.rfid_status;   // 偏移量 0, 大小 4字节
  uint8_t status_2 = rfid_status.data.rfid_status_2;  // 偏移量 4, 大小 1字节

  // --- Byte Offset 0 (uint32_t status_1) ---

  // 基础增益
  msg.friendly_base_gain_point = (status_1 >> 0) & 1;              // bit 0: 己方基地增益点
  msg.friendly_central_highland_gain_point = (status_1 >> 1) & 1;  // bit 1: 己方中央高地增益点
  msg.enemy_central_highland_gain_point = (status_1 >> 2) & 1;  // bit 2: 对方中央高地增益点
  msg.friendly_trapezoidal_gain_point = (status_1 >> 3) & 1;  // bit 3: 己方梯形高地增益点
  msg.enemy_trapezoidal_gain_point = (status_1 >> 4) & 1;     // bit 4: 对方梯形高地增益点

  // 飞坡增益 (Fly Slope)
  msg.friendly_fly_slope_pre_gain =
    (status_1 >> 5) & 1;  // bit 5: 己方地形跨越增益点（飞坡）（靠近己方一侧飞坡前）
  msg.friendly_fly_slope_post_gain =
    (status_1 >> 6) & 1;  // bit 6: 己方地形跨越增益点（飞坡）（靠近己方一侧飞坡后）
  msg.enemy_fly_slope_pre_gain =
    (status_1 >> 7) & 1;  // bit 7: 对方地形跨越增益点（飞坡）（靠近对方一侧飞坡前）
  msg.enemy_fly_slope_post_gain =
    (status_1 >> 8) & 1;  // bit 8: 对方地形跨越增益点（飞坡）（靠近对方一侧飞坡后）

  // 中央高地上下方 (Central Highland)
  msg.friendly_central_highland_under =
    (status_1 >> 9) & 1;  // bit 9: 己方地形跨越增益点（中央高地下方）
  msg.friendly_central_highland_upper =
    (status_1 >> 10) & 1;  // bit 10: 己方地形跨越增益点（中央高地上方）
  msg.enemy_central_highland_under =
    (status_1 >> 11) & 1;  // bit 11: 对方地形跨越增益点（中央高地下方）
  msg.enemy_central_highland_upper =
    (status_1 >> 12) & 1;  // bit 12: 对方地形跨越增益点（中央高地上方）

  // 公路上下方 (Highway)
  msg.friendly_highway_under = (status_1 >> 13) & 1;  // bit 13: 己方地形跨越增益点（公路下方）
  msg.friendly_highway_upper = (status_1 >> 14) & 1;  // bit 14: 己方地形跨越增益点（公路上方）
  msg.enemy_highway_under = (status_1 >> 15) & 1;  // bit 15: 对方地形跨越增益点（公路下方）
  msg.enemy_highway_upper = (status_1 >> 16) & 1;  // bit 16: 对方地形跨越增益点（公路上方）

  // 建筑与特殊区域
  msg.friendly_fortress_gain = (status_1 >> 17) & 1;  // bit 17: 己方堡垒增益点
  msg.friendly_outpost_gain = (status_1 >> 18) & 1;   // bit 18: 己方前哨站增益点
  msg.friendly_supply_zone_non_overlap =
    (status_1 >> 19) & 1;  // bit 19: 己方与资源区不重叠的补给区/RMUL 补给区
  msg.friendly_supply_zone_overlap = (status_1 >> 20) & 1;  // bit 20: 己方与资源区重叠的补给区

  // 能量机关/装配点
  msg.friendly_energy_mechanism_gain = (status_1 >> 21) & 1;  // bit 21: 己方装配增益点
  msg.enemy_energy_mechanism_gain = (status_1 >> 22) & 1;     // bit 22: 对方装配增益点

  // 其他
  msg.center_gain_point_rmul = (status_1 >> 23) & 1;  // bit 23: 中心增益点（仅 RMUL 适用）
  msg.enemy_fortress_gain = (status_1 >> 24) & 1;     // bit 24: 对方堡垒增益点
  msg.enemy_outpost_gain = (status_1 >> 25) & 1;      // bit 25: 对方前哨站增益点

  // 隧道增益 (Tunnel) - 32位整数的最后部分
  msg.friendly_tunnel_highway_under =
    (status_1 >> 26) & 1;  // bit 26: 己方地形跨越增益点（隧道）（靠近己方一侧公路区下方）
  msg.friendly_tunnel_highway_upper =
    (status_1 >> 27) & 1;  // bit 27: 己方地形跨越增益点（隧道）（靠近己方一侧公路区上方）
  msg.friendly_tunnel_trapezoid_low =
    (status_1 >> 28) & 1;  // bit 28: 己方地形跨越增益点（隧道）（靠近己方梯形高地较低处）
  msg.friendly_tunnel_trapezoid_high =
    (status_1 >> 29) & 1;  // bit 29: 己方地形跨越增益点（隧道）（靠近己方梯形高地较高处）
  msg.enemy_tunnel_highway_under =
    (status_1 >> 30) & 1;  // bit 30: 对方地形跨越增益点（隧道）（靠近对方一侧公路区下方）
  msg.enemy_tunnel_highway_upper =
    (status_1 >> 31) & 1;  // bit 31: 对方地形跨越增益点（隧道）（靠近对方一侧公路区上方）

  // --- Byte Offset 4 (uint8_t status_2) ---

  // 隧道增益 (Tunnel) - 接续的1个字节
  msg.enemy_tunnel_trapezoid_low =
    (status_2 >> 0) & 1;  // bit 0: 对方地形跨越增益点（隧道）（靠近对方梯形高地较低处）
  msg.enemy_tunnel_trapezoid_high =
    (status_2 >> 1) & 1;  // bit 1: 对方地形跨越增益点（隧道）（靠近对方梯形高地较高处）

  rfid_status_pub_->publish(msg);
}

void Srm27NavProtocolNode::publishRobotStatus(ReceiveRobotStatus & robot_status)
{
  robot_status_state_.shooter_heat = robot_status.data.shooter_17mm_1_barrel_heat;
  robot_status_state_.heat_valid = true;
  robot_status_pub_->publish(robot_status_state_);
}

void Srm27NavProtocolNode::publishProjectileAllowance(ReceiveProjectileAllowance & projectile_allowance)
{
  rm_decision_interfaces::msg::ProjectileAllowance msg;
  msg.projectile_allowance_17mm = projectile_allowance.data.projectile_allowance_17mm;
  msg.projectile_allowance_42mm = projectile_allowance.data.projectile_allowance_42mm;
  msg.remaining_gold_coin = projectile_allowance.data.remaining_gold_coin;
  msg.projectile_allowance_fortress = projectile_allowance.data.projectile_allowance_fortress;
  projectile_allowance_pub_->publish(msg);
  robot_status_state_.projectile_allowance_17mm = msg.projectile_allowance_17mm;
  robot_status_state_.remaining_gold_coin = msg.remaining_gold_coin;
  robot_status_state_.ammo_valid = true;
  robot_status_pub_->publish(robot_status_state_);
}

void Srm27NavProtocolNode::publishSwitchPosition(ReceiveSwitchPosition & switch_position)
{
  rm_decision_interfaces::msg::SwitchPosition msg;
  msg.switch_position = switch_position.data.switch_position;
  switch_position_pub_->publish(msg);
}

void Srm27NavProtocolNode::publishSefdefined(ReceiveSefdefinedData & sefdefined)
{
  rm_decision_interfaces::msg::Sefdefined msg;
  msg.robot_id = sefdefined.data.robot_id;
  msg.robot_level = sefdefined.data.robot_level;
  msg.current_hp = sefdefined.data.current_HP;
  msg.maximum_hp = sefdefined.data.maximum_HP;
  msg.shooter_barrel_cooling_value = sefdefined.data.shooter_barrel_cooling_value;
  msg.shooter_barrel_heat_limit = sefdefined.data.shooter_barrel_heat_limit;
  msg.chassis_power_limit = sefdefined.data.chassis_power_limit;
  msg.power_management_gimbal_output = sefdefined.data.power_management_gimbal_output;
  msg.power_management_chassis_output = sefdefined.data.power_management_chassis_output;
  msg.power_management_shooter_output = sefdefined.data.power_management_shooter_output;

  sefdefined_pub_->publish(msg);
  robot_status_state_.robot_id = msg.robot_id;
  robot_status_state_.robot_level = msg.robot_level;
  robot_status_state_.current_hp = msg.current_hp;
  robot_status_state_.maximum_hp = msg.maximum_hp;
  robot_status_state_.shooter_barrel_cooling_value = msg.shooter_barrel_cooling_value;
  robot_status_state_.shooter_barrel_heat_limit = msg.shooter_barrel_heat_limit;
  robot_status_state_.chassis_power_limit = msg.chassis_power_limit;
  robot_status_state_.team_color = msg.robot_id >= 100;
  robot_status_state_.status_valid = true;
  robot_status_pub_->publish(robot_status_state_);
}

/********************************************************/
/* Send data                                         */
/********************************************************/
void Srm27NavProtocolNode::sendData()
{
  RCLCPP_INFO(get_logger(), "Start sendData!");

  // [协议 §2.1] 帧长 19 字节 / 数据段 12 字节, 由 packet_typedef.hpp 里的
  // static_assert 在编译期钉死; 这里不再用 sizeof 现算, 避免将来静默漂移。
  send_robot_cmd_data_.frame_header.sof = SOF_SEND;
  send_robot_cmd_data_.frame_header.seq = 0;
  send_robot_cmd_data_.frame_header.data_length = DATA_LENGTH_CMD;

  SendRobotCmdData local_data_copy{};

  // [协议 §6.1] 定频节奏。周期以"上一帧实际发完之后"为基准计算,
  // 这样任何情况下相邻两帧之间都至少有 period 的空闲, 不会背靠背 (§6.2)。
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(1.0 / send_rate_hz_));
  int retry_count = 0;

  while (rclcpp::ok()) {
    if (!is_usb_ok_) {
      RCLCPP_WARN(get_logger(), "send: usb is not ok! Retry count: %d", retry_count++);
      std::this_thread::sleep_for(std::chrono::milliseconds(USB_NOT_OK_SLEEP_TIME));
      continue;
    }

    {  // 组帧: 只在取状态和递增 seq 时加锁; 打包与 write 都在锁外
      std::lock_guard<std::mutex> lk(send_mutex_);
      // 自增在拷贝前完成，避免循环开始后被拷贝覆盖
      send_robot_cmd_data_.frame_header.seq++;
      local_data_copy = send_robot_cmd_data_;
      // 速度字段每帧现填: 看门狗(§6.3) + NaN/Inf 兜底(§6.4) + 限幅(§6.5)
      fillSpeedVector(local_data_copy);
    }

    try {
      // 1. 帧头 CRC8: 覆盖偏移 0-3, 结果写在偏移 4
      append_CRC8_check_sum(
        reinterpret_cast<unsigned char *>(&local_data_copy), sizeof(HeaderFrame));

      // 2. 整包 CRC16: 覆盖偏移 0-16, 低字节写在偏移 17
      append_CRC16_check_sum(
        reinterpret_cast<uint8_t *>(&local_data_copy), sizeof(SendRobotCmdData));

      std::vector<uint8_t> send_data = toVector(local_data_copy);

      // [诊断] 打开 debug_print_hex 时打印实际发出去的整包 hex。
      // 排查"字节根本没发出去"和"发了但下位机不认"时, 这行能直接区分两者。
      // 注意它会影响定频心跳, 平时必须保持 false (§6.6)。
      if (debug_print_hex_) {
        uint16_t crc16_value = (static_cast<uint16_t>(send_data[send_data.size() - 1]) << 8) |
                               send_data[send_data.size() - 2];
        RCLCPP_INFO(
          get_logger(), "SEND len=%zu data_length=%u crc16=0x%04X vx=%.4f vy=%.4f wz=%.4f",
          send_data.size(), static_cast<unsigned>(local_data_copy.frame_header.data_length),
          crc16_value, local_data_copy.speed_vector.vx, local_data_copy.speed_vector.vy,
          local_data_copy.speed_vector.wz);
        printHex("SEND", ID_ROBOT_CMD, send_data);
      }

      serial_driver_->port()->send(send_data);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error sending data: %s", ex.what());
      is_usb_ok_ = false;
    }

    // 等满一个周期再发下一帧。以"上一帧刚发完"为基准, 所以被抢占时
    // 只会顺延、不会补发, 天然满足"每帧之间要有空闲间隔"(§6.2)。
    std::this_thread::sleep_until(std::chrono::steady_clock::now() + period);
  }

  // [协议 §6.3] 正常结束 / Ctrl-C / 异常都要补发零速帧。
  // 此刻串口仍然打开 —— 析构函数是先 join 本线程, 之后才 close 端口。
  sendZeroFramesOnExit();
}

void Srm27NavProtocolNode::fillSpeedVector(SendRobotCmdData & frame)
{
  // 调用者必须已持有 send_mutex_
  const auto now = std::chrono::steady_clock::now();

  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;

  // [协议 §6.3] 看门狗: 控制量超过 cmd_timeout_sec_ 没更新就自动归零。
  // 同时覆盖"进程刚起来还没收到 cmd_vel"的情况 —— 启动即发零速帧。
  const bool fresh = has_cmd_vel_ &&
    std::chrono::duration<double>(now - last_cmd_vel_time_).count() <= cmd_timeout_sec_;

  if (fresh) {
    vx = cmd_vx_;
    vy = cmd_vy_;
    wz = cmd_wz_;
    if (watchdog_active_) {
      RCLCPP_INFO(get_logger(), "看门狗解除: cmd_vel 已恢复更新");
      watchdog_active_ = false;
    }
  } else if (!watchdog_active_) {
    if (has_cmd_vel_) {
      RCLCPP_WARN(
        get_logger(), "看门狗触发: %.0f ms 未收到 cmd_vel, 控制量归零 (协议 §6.3)",
        cmd_timeout_sec_ * 1000.0);
    } else {
      RCLCPP_INFO(get_logger(), "尚未收到 cmd_vel, 先发零速帧 (协议 §6.3)");
    }
    watchdog_active_ = true;
  }

  // [协议 §6.4] 兜底再挡一次 NaN/Inf, 确保任何路径都不会把坏值发到下位机
  if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz)) {
    RCLCPP_ERROR(get_logger(), "控制量含 NaN/Inf, 本帧按零速发送 (协议 §6.4)");
    vx = 0.0;
    vy = 0.0;
    wz = 0.0;
  }

  // [协议 §6.5] 自己做限幅 —— 协议层不限幅, 限幅是上位机的责任
  bool clamped = false;
  if (std::abs(vx) > max_vx_) {
    vx = std::copysign(max_vx_, vx);
    clamped = true;
  }
  if (std::abs(vy) > max_vy_) {
    vy = std::copysign(max_vy_, vy);
    clamped = true;
  }
  if (std::abs(wz) > max_wz_) {
    wz = std::copysign(max_wz_, wz);
    clamped = true;
  }
  if (clamped) {
    if (!clamping_active_) {
      RCLCPP_WARN(
        get_logger(),
        "限幅生效: 收到 vx=%.4f vy=%.4f wz=%.4f, 已截到 (%.3f, %.3f, %.3f) "
        "(协议 §6.5; 需要更大请调 max_vx/max_vy/max_wz)",
        cmd_vx_, cmd_vy_, cmd_wz_, max_vx_, max_vy_, max_wz_);
      clamping_active_ = true;
    }
  } else {
    clamping_active_ = false;
  }

  frame.speed_vector.vx = static_cast<float>(vx);
  frame.speed_vector.vy = static_cast<float>(vy);
  frame.speed_vector.wz = static_cast<float>(wz);
}

void Srm27NavProtocolNode::sendZeroFramesOnExit()
{
  // [协议 §6.3] 协议没有超时保护 —— 下位机一直使用最后一次收到的有效指令。
  // 所以退出路径必须补发零速帧, 否则底盘会保持退出前的速度一直跑下去。
  if (!is_usb_ok_) {
    RCLCPP_WARN(
      get_logger(),
      "串口不可用, 无法补发零速帧 —— 请立即用物理急停确认底盘已停! (协议 §6.3)");
    return;
  }

  SendRobotCmdData zero{};
  zero.frame_header.sof = SOF_SEND;
  zero.frame_header.data_length = DATA_LENGTH_CMD;
  zero.speed_vector.vx = 0.0f;
  zero.speed_vector.vy = 0.0f;
  zero.speed_vector.wz = 0.0f;
  {
    std::lock_guard<std::mutex> lk(send_mutex_);
    zero.frame_header.seq = send_robot_cmd_data_.frame_header.seq;
  }

  // 连发若干帧, 保证在关串口前下位机一定收到至少一个有效零速帧
  constexpr int kZeroFrameCount = 10;   // 10 帧 × 10 ms ≈ 100 ms
  int sent = 0;
  for (int i = 0; i < kZeroFrameCount; ++i) {
    zero.frame_header.seq++;
    try {
      append_CRC8_check_sum(
        reinterpret_cast<unsigned char *>(&zero), sizeof(HeaderFrame));
      append_CRC16_check_sum(
        reinterpret_cast<uint8_t *>(&zero), sizeof(SendRobotCmdData));
      serial_driver_->port()->send(toVector(zero));
      ++sent;
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "补发零速帧失败: %s", ex.what());
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (sent == kZeroFrameCount) {
    RCLCPP_INFO(get_logger(), "退出前已补发 %d 个零速帧 (协议 §6.3)", sent);
  } else {
    RCLCPP_ERROR(
      get_logger(), "退出前只补发了 %d/%d 个零速帧 —— 请用物理急停确认底盘已停!",
      sent, kZeroFrameCount);
  }
}

void Srm27NavProtocolNode::CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  const double vx = msg->linear.x;
  const double vy = msg->linear.y;
  const double wz = msg->angular.z;

  std::lock_guard<std::mutex> lk(send_mutex_);

  // [协议 §6.4] 不要发送 NaN/Inf: 解算异常时退回零速, 而不是把坏值发下去
  if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz)) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "cmd_vel 含 NaN/Inf (vx=%.4f vy=%.4f wz=%.4f), 按零速处理 (协议 §6.4)",
      vx, vy, wz);
    cmd_vx_ = 0.0;
    cmd_vy_ = 0.0;
    cmd_wz_ = 0.0;
  } else {
    cmd_vx_ = vx;
    cmd_vy_ = vy;
    cmd_wz_ = wz;
  }

  // 只要收到消息就算"有控制量", 包括被判为 NaN 后置零的那次 ——
  // 否则看门狗会同时报警, 掩盖真正的原因。
  has_cmd_vel_ = true;
  last_cmd_vel_time_ = std::chrono::steady_clock::now();
}

void Srm27NavProtocolNode::printHex(
  const std::string & tag, uint16_t id, const std::vector<uint8_t> & data)
{
  std::stringstream ss;
  ss << "[" << tag << "] ID:0x" << std::hex << std::uppercase << std::setw(4) << std::setfill('0')
     << (int)id << " TotalLen:" << std::dec << data.size() << " Raw: ";
  for (auto b : data) {
    ss << std::hex << std::setw(2) << std::setfill('0') << (unsigned)b << " ";
  }
  RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
}

}  // namespace srm27_nav_protocol

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(srm27_nav_protocol::Srm27NavProtocolNode)