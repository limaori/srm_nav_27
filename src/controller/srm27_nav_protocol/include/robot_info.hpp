#ifndef SRM27_NAV_PROTOCOL__ROBOT_INFO_HPP_
#define SRM27_NAV_PROTOCOL__ROBOT_INFO_HPP_

#include <map>
#include <string>

namespace srm27_nav_protocol
{
const int CHASSIS_MODEL_NUM = 5;
const int GIMBAL_MODEL_NUM = 2;
const int SHOOT_MODEL_NUM = 3;
const int ARM_MODEL_NUM = 2;
const int CUSTOM_CONTROLLER_MODEL_NUM = 2;

struct RobotModels
{
  std::map<uint8_t, std::string> chassis;
  std::map<uint8_t, std::string> gimbal;
  std::map<uint8_t, std::string> shoot;
  std::map<uint8_t, std::string> arm;
  std::map<uint8_t, std::string> custom_controller;
};

}  // namespace srm27_nav_protocol

#endif  // SRM27_NAV_PROTOCOL__ROBOT_INFO_HPP_
