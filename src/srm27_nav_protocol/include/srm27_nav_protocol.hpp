/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       srm27_nav_protocol.hpp
  * @brief      上下位机通信模块 (Adapted for RoboMaster 2026 Protocol V1.0.0)
  * @history
  * Version    Date            Author          Modification
  * V1.0.0     Jul-18-2024     Penguin         1. done
  * V1.1.0     Nov-28-2025     AI              2. Update to new RM2026 Protocol
  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  */
#ifndef SRM27_NAV_PROTOCOL_HPP_
#define SRM27_NAV_PROTOCOL_HPP_

// 移除了 tf2_ros/transform_broadcaster.h (IMU 功能已移除)
// 移除了 float64, u_int8, imu, joint_state 等旧消息头文件

#include <atomic>
#include <chrono>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <rm_decision_interfaces/msg/event_data.hpp>
#include <rm_decision_interfaces/msg/game_robot_hp.hpp>
#include <rm_decision_interfaces/msg/game_status.hpp>
#include <rm_decision_interfaces/msg/ground_robot_position.hpp>
#include <rm_decision_interfaces/msg/rfid_status.hpp>
#include "rm_decision_interfaces/msg/ally_robot_hp.hpp"
#include <rm_decision_interfaces/msg/robot_status.hpp> // 注意：这里对应的是 0x0202 能量热量数据
#include <rm_decision_interfaces/msg/sefdefined.hpp>
#include <rm_decision_interfaces/msg/projectile_allowance.hpp>
#include <rm_decision_interfaces/msg/switch_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <serial_driver/serial_driver.hpp>

#include "packet_typedef.hpp"
// 移除了 "robot_info.hpp" (RobotModels 依赖已移除)

namespace srm27_nav_protocol
{
class Srm27NavProtocolNode : public rclcpp::Node
{
public:
  explicit Srm27NavProtocolNode(const rclcpp::NodeOptions & options);

  ~Srm27NavProtocolNode() override;

private:
  // 由 serialPortProtect / receive / send 三个线程共享, 用原子量避免数据竞争。
  std::atomic<bool> is_usb_ok_{false};
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

  std::thread receive_thread_;
  std::thread send_thread_;
  std::thread serial_port_protect_thread_;

  // Publish (仅保留新协议需要的发布者)
  rclcpp::Publisher<rm_decision_interfaces::msg::EventData>::SharedPtr event_data_pub_;             // 0x0101
  rclcpp::Publisher<rm_decision_interfaces::msg::AllyRobotHP>::SharedPtr all_robot_hp_pub_;
  rclcpp::Publisher<rm_decision_interfaces::msg::GameStatus>::SharedPtr game_status_pub_;           // 0x0001
  rclcpp::Publisher<rm_decision_interfaces::msg::GroundRobotPosition>::SharedPtr ground_robot_position_pub_; // 0x020B
  rclcpp::Publisher<rm_decision_interfaces::msg::RfidStatus>::SharedPtr rfid_status_pub_;           // 0x0209
  rclcpp::Publisher<rm_decision_interfaces::msg::RobotStatus>::SharedPtr robot_status_pub_;         // 0x0202
  rclcpp::Publisher<rm_decision_interfaces::msg::ProjectileAllowance>::SharedPtr projectile_allowance_pub_; // 0x0208
  rclcpp::Publisher<rm_decision_interfaces::msg::SwitchPosition>::SharedPtr switch_position_pub_;         // 0x000D
  rclcpp::Publisher<rm_decision_interfaces::msg::Sefdefined>::SharedPtr sefdefined_pub_;            // 0x000B
  // 0x0010 底盘速度反馈（下位机速度观测器的融合结果）。
  // 速度用 TwistStamped（带 stamp + frame_id，便于和 /cmd_vel_chassis 对时/对账）；
  // 底盘相对云台的偏角单独一条 Float32，保持下位机口径（度），不做隐式单位换算；
  // 再由它换算出 URDF gimbal_yaw_joint 的关节角（云台相对底盘、弧度）另发一条 Float32，
  // 供 gimbal_joint_state_relay 转成 /joint_states（详见 cpp 里 publishChassisSpeed）。
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr chassis_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr chassis_to_gimbal_yaw_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gimbal_joint_pub_;

  // Subscribe
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  // 移除了 robot_control 订阅: 它唯一的用途是把 is_recovering 塞进数据段, 而标准
  // 19 字节帧没有这个字节 (协议 §2.1)。该话题当前也没有任何发布者。
  // 若决策组需要下发"回血中"等状态, 必须先扩展协议, 不能私自加字节。

  // 移除了 robot_models_, debug_pub_map_
  // 移除了 imu_tf_broadcaster_

  /******** 发送侧状态 (协议 §3.3 / §6) ********/
  // 只承载帧头 + seq; 速度字段每帧由 fillSpeedVector() 现填, 不在这里存状态。
  SendRobotCmdData send_robot_cmd_data_{};

  // 最近一次 /cmd_vel 的原始指令值 (未限幅、未判超时)。受 send_mutex_ 保护。
  double cmd_vx_{0.0};
  double cmd_vy_{0.0};
  double cmd_wz_{0.0};
  bool has_cmd_vel_{false};
  std::chrono::steady_clock::time_point last_cmd_vel_time_{};

  // 参数
  double send_rate_hz_{100.0};     // §6.1 推荐 100 Hz 定频单帧
  double cmd_timeout_sec_{0.5};    // §6.3 看门狗: 超时未更新则归零
  double max_vx_{0.5};             // §6.5 上位机自己限幅
  double max_vy_{0.5};
  double max_wz_{1.0};
  // 速度对齐系数: 发往下位机之前把指令乘这个系数。
  // 用来修正"指令值 → 底盘实际速度"的尺度不一致 (实测依据见 config yaml 的注释)。
  // 1.0 = 不缩放(原行为)。只作用在**出口**上: cmd_vx_ 仍保存收到的原始值。
  double linear_velocity_scale_{1.0};
  double angular_velocity_scale_{1.0};
  bool debug_print_hex_{false};

  /******** 接收侧 0x0010 底盘速度反馈 ********/
  bool publish_chassis_speed_{true};
  std::string chassis_speed_topic_{"chassis_speed"};
  // 下位机原值：底盘相对云台的偏角（度），正常范围 (-180, 180]。
  std::string chassis_to_gimbal_yaw_topic_{"chassis_to_gimbal_yaw_deg"};
  // 换算后的 URDF gimbal_yaw_joint 关节角（云台相对底盘，rad）。这个才是接进 TF 的量。
  std::string gimbal_joint_topic_{"gimbal_joint_angle"};
  // 换算系数里的符号：joint_rad = gimbal_joint_sign * deg * π / 180。
  // 下位机是"底盘相对云台"、URDF 是"云台相对底盘"，所以默认 -1.0（互为相反数）。
  // 放到参数里是为了现场能用 --ros-args -p 翻转，不用重新编译。
  double gimbal_joint_sign_{-1.0};
  std::string chassis_speed_frame_{"base_link"};
  // 超过这么久没收到 0x0010 就告警一次（收到后恢复也告警一次）。
  // 0x0010 约 33 Hz（10 ms 槽位轮转），0.5 s 相当于连丢 ~16 个槽位。
  double chassis_speed_timeout_sec_{0.5};
  // 只在接收线程写；watchdog 定时器只读。
  std::atomic<uint64_t> chassis_speed_count_{0};
  std::atomic<uint64_t> chassis_speed_bad_length_{0};
  std::atomic<uint64_t> chassis_speed_nonfinite_{0};
  std::atomic<int64_t> chassis_speed_last_steady_ns_{0};
  bool chassis_speed_stale_{true};   // 起机时按"还没有数据"处理

  // 限流日志用的状态翻转标志
  bool watchdog_active_{false};
  bool clamping_active_{false};

  // Updated only by the receive thread. Partial packets retain other known fields.
  rm_decision_interfaces::msg::RobotStatus robot_status_state_{};

  void getParams();
  void createPublisher();
  void createSubscription();
  // 移除了 createNewDebugPublisher
  void receiveData();
  void sendData();
  void serialPortProtect();

  // 每帧组装发送量: 看门狗(§6.3) + NaN/Inf 兜底(§6.4) + 限幅(§6.5)。
  // 调用者必须已持有 send_mutex_。
  void fillSpeedVector(SendRobotCmdData & frame);
  // 退出前补发零速帧 (协议 §6.3)。由 sendData() 在循环结束后调用,
  // 此时串口仍然打开 (析构函数先 join send 线程, 之后才 close)。
  void sendZeroFramesOnExit();

  // Data Process Functions (仅保留新协议对应的处理函数)
  void publishEventData(ReceiveEventData & data);           // 0x0101
  void publishAllRobotHp(ReceiveAllRobotHpData & data);     // 0x0003
  void publishGameStatus(ReceiveGameStatusData & data);     // 0x0001
  void publishGroundRobotPosition(ReceiveGroundRobotPosition & data); // 0x020B
  void publishRfidStatus(ReceiveRfidStatus & data);         // 0x0209
  void publishRobotStatus(ReceiveRobotStatus & data);       // 0x0202
  void publishProjectileAllowance(ReceiveProjectileAllowance & data); // 0x0208
  void publishSwitchPosition(ReceiveSwitchPosition & data);  // 0x000D
  void publishSefdefined(ReceiveSefdefinedData & data);     // 0x000B
  void publishChassisSpeed(ReceiveChassisSpeedData & data); // 0x0010
  // 移除了 publishDebugData, publishImuData, publishRobotInfo, publishRobotMotion, publishJointState

  // Callbacks
  void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  // 移除了 RobotControlCallback (is_recovering 在标准 19 字节帧里没有位置)

  // 0x0010 失联/恢复检测（由 sendData 线程按 1 s 周期调用）
  void checkChassisSpeedFreshness();

  // debug (ID 类型修正为 uint16_t 以匹配 2 字节 ID)
  void printHex(const std::string& tag, uint16_t id, const std::vector<uint8_t>& data);
};
}  // namespace srm27_nav_protocol

#endif  // SRM27_NAV_PROTOCOL_HPP_