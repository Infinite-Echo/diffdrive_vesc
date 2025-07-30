#pragma once

#include <string>
#include <cmath>
#include <linux/can.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"

#define SECONDS_PER_MINUTE 60
#define FULL_ROTATION_RADS M_PI*2
#define RPM_CMD_ID 0x300U
#define RPM_STATUS_ID 0x900U

class VESC
{
public:
  std::string name = "";
  double cmd = 0;
  double pos = 0; //wheel pos in rads
  double vel = 0; //wheel vel in rads/s

  VESC() = default;

  VESC(uint32_t can_id, double gear_ratio);

  void setup(uint32_t can_id, double gear_ratio);

  int32_t vel_to_rpm(double vel); //pretty sure ros2_control uses all angular vels in rads/s

  double rpm_to_vel(int32_t rpm);

  can_frame create_rpm_cmd(int32_t rpm);

  void handle_rpm_feedback_msg(const can_frame& frame);

private:
  uint32_t can_id_ = 0;
  double gear_ratio_ = 1.0;
  std::shared_ptr<rclcpp::Logger> logger_;

};