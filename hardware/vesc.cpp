#include "diffdrive_vesc/vesc.hpp"

/*erpm probably stands for electric rpm and is affected by the number of poles in a motor
You can try to look through https://github.com/vedderb/bldc/tree/master source code 
to understand it better but probably not necessary or worth the effort
*/

VESC::VESC(uint32_t can_id, double gear_ratio, double pole_pairs)
{
  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.DiffBot"));
  setup(can_id, gear_ratio, pole_pairs);
}

void VESC::setup(uint32_t can_id, double gear_ratio, double pole_pairs)
{
  can_id_ = can_id;
  gear_ratio_ = gear_ratio;
  pole_pairs_ = pole_pairs;
}

double VESC::vel_to_rpm(double vel)
{
  // rads/s to rpm
  return static_cast<double>((vel * SECONDS_PER_MINUTE) / FULL_ROTATION_RADS);
}

double VESC::rpm_to_vel(double rpm)
{
  return static_cast<double>((rpm * FULL_ROTATION_RADS) / SECONDS_PER_MINUTE);
}

can_frame VESC::create_rpm_cmd(double rpm)
{
  can_frame rpm_msg{};
  rpm_msg.can_id = can_id_ | RPM_CMD_ID | CAN_EFF_FLAG;
  rpm_msg.can_dlc = 4;
  
  RCLCPP_DEBUG(rclcpp::get_logger("diffdrive_vesc"), "rpm: '%f'", rpm);
  int32_t erpm = static_cast<int32_t>(std::llround(rpm * gear_ratio_ * pole_pairs_));
  RCLCPP_DEBUG(rclcpp::get_logger("diffdrive_vesc"), "erpm: '%d'", erpm);

  // data is big-endian (https://github.com/vedderb/bldc/blob/master/documentation/comm_can.md)
  rpm_msg.data[0] = static_cast<uint8_t>((erpm >> 24) & 0xFF);
  rpm_msg.data[1] = static_cast<uint8_t>((erpm >> 16) & 0xFF);
  rpm_msg.data[2] = static_cast<uint8_t>((erpm >> 8) & 0xFF);
  rpm_msg.data[3] = static_cast<uint8_t>(erpm & 0xFF);
  return rpm_msg;
}

void VESC::handle_rpm_feedback_msg(const can_frame& frame)
{
  RCLCPP_DEBUG(rclcpp::get_logger("diffdrive_vesc"), "dlc: '%d'", static_cast<int>(frame.can_dlc));
  if(static_cast<int>(frame.can_dlc) < 4)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("diffdrive_vesc"), "Leaving callback");
    return;
  }
  RCLCPP_DEBUG(rclcpp::get_logger("diffdrive_vesc"), "Inside callback");

  int32_t ERPM = (static_cast<int32_t>(frame.data[0]) << 24) |
                (static_cast<int32_t>(frame.data[1]) << 16) |
                (static_cast<int32_t>(frame.data[2]) << 8) |
                static_cast<int32_t>(frame.data[3]);
  RCLCPP_DEBUG(rclcpp::get_logger("diffdrive_vesc"), "erpm feedback: '%d'", ERPM);
  int32_t RPM = ERPM / (gear_ratio_ * pole_pairs_); 
  /* the (2 * gear_ratio_) has something to do with motor poles/erpm 
  (you'd have to dig into https://github.com/vedderb/bldc/tree/master to understand it) */
  vel = rpm_to_vel(RPM);
}