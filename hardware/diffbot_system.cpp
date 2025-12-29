// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "diffdrive_vesc/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffdrive_vesc
{
hardware_interface::CallbackReturn DiffDriveVescHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.front_left_vesc_id = std::stoi(info_.hardware_parameters["front_left_vesc_id"]);
  cfg_.front_right_vesc_id = std::stoi(info_.hardware_parameters["front_right_vesc_id"]);
  cfg_.back_left_vesc_id = std::stoi(info_.hardware_parameters["back_left_vesc_id"]);
  cfg_.back_right_vesc_id = std::stoi(info_.hardware_parameters["back_right_vesc_id"]);
  cfg_.gear_ratio = hardware_interface::stod(info_.hardware_parameters["gear_ratio"]);
  cfg_.pole_pairs = hardware_interface::stod(info_.hardware_parameters["pole_pairs"]);
  cfg_.device = info_.hardware_parameters["device"];

  front_left_vesc_.setup(cfg_.front_left_vesc_id, cfg_.gear_ratio, cfg_.pole_pairs);
  front_right_vesc_.setup(cfg_.front_right_vesc_id, cfg_.gear_ratio, cfg_.pole_pairs);
  back_left_vesc_.setup(cfg_.back_left_vesc_id, cfg_.gear_ratio, cfg_.pole_pairs);
  back_right_vesc_.setup(cfg_.back_right_vesc_id, cfg_.gear_ratio, cfg_.pole_pairs);

  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.DiffBot"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveVescHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  /*
  NOTE: Temporarily hardcoding the values to ensure this works. Should switch to using an "array" (whatever C++ equivalent of array for this) 
  of VESCs and iterate through them later. Fully aware this is terrible code and it should be changed as soon as the plugin is 
  confirmed to work with hardware.

  Also: Position state interface isn't actually getting used and should either be removed or implemented since we could obtain the data for it.
  If removed, the error handling in the init needs to be updated since it checks for pos existence
  If implemented, check https://github.com/vedderb/bldc/blob/master/documentation/comm_can.md#status-commands for obtainable data.
  I believe pos is expected in radians (not sure if it is cumulative or if it is reset after each rotation), 
  PID pos from status 4 or tachometer from status 5 are the most likely to have the desired data to get pos
  */
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      "front_left_wheel_joint", hardware_interface::HW_IF_POSITION, &front_left_vesc_.pos));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      "front_left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &front_left_vesc_.vel));

  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      "front_right_wheel_joint", hardware_interface::HW_IF_POSITION, &front_right_vesc_.pos));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      "front_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &front_right_vesc_.vel));

  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      "back_left_wheel_joint", hardware_interface::HW_IF_POSITION, &back_left_vesc_.pos));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      "back_left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &back_left_vesc_.vel));

  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      "back_right_wheel_joint", hardware_interface::HW_IF_POSITION, &back_right_vesc_.pos));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      "back_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &back_right_vesc_.vel));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveVescHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "front_left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &front_left_vesc_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "front_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &front_right_vesc_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "back_left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &back_left_vesc_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "back_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &back_right_vesc_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveVescHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");
  can_.start(cfg_.device);
  /*
    for whoever takes over:
    Extended Frame Format (EFF):
    - basically makes it so that the CAN ID frame is 29-bits long instead of the 11-bits in the Standard Frame Format (SFF)
    - The CAN_EFF_FLAG is OR'd with the ID we want when setting a EFF frame's ID to ensure it is 29-bits
    - The CAN_EFF_FLAG constant comes from the <linux/can.h> header
    
    VESC CAN Message Frame Format (https://github.com/vedderb/bldc/blob/master/documentation/comm_can.md#frame-format):
    - First 8-bits of the frame ID are set to the value of the VESC's CAN ID which is set in the VESC tool GUI application 
    - We OR the VESC ID with a command or status ID to create the frame ID that 
      corresponds to a specific command or status for a specific VESC
        -NOTE: the commands/status ID's must be shifted 1 byte (8-bits) to the left 
               before ORing or two trailing HEX 0's must be added to the end
   */
  can_.register_callback(cfg_.front_left_vesc_id | RPM_STATUS_ID | CAN_EFF_FLAG, 
                        std::bind(&VESC::handle_rpm_feedback_msg, &front_left_vesc_, std::placeholders::_1));
  can_.register_callback(cfg_.front_right_vesc_id | RPM_STATUS_ID | CAN_EFF_FLAG, 
                        std::bind(&VESC::handle_rpm_feedback_msg, &front_right_vesc_, std::placeholders::_1));
  can_.register_callback(cfg_.back_left_vesc_id | RPM_STATUS_ID | CAN_EFF_FLAG,
                        std::bind(&VESC::handle_rpm_feedback_msg, &back_left_vesc_, std::placeholders::_1));
  can_.register_callback(cfg_.back_right_vesc_id | RPM_STATUS_ID | CAN_EFF_FLAG,
                        std::bind(&VESC::handle_rpm_feedback_msg, &back_right_vesc_, std::placeholders::_1));
  RCLCPP_INFO(get_logger(), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveVescHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");
  can_.unregister_callback(cfg_.front_left_vesc_id | RPM_STATUS_ID | CAN_EFF_FLAG);
  can_.unregister_callback(cfg_.front_right_vesc_id | RPM_STATUS_ID | CAN_EFF_FLAG);
  can_.unregister_callback(cfg_.back_left_vesc_id | RPM_STATUS_ID | CAN_EFF_FLAG);
  can_.unregister_callback(cfg_.back_right_vesc_id | RPM_STATUS_ID | CAN_EFF_FLAG);
  can_.stop();
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveVescHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // since we are using async callbacks that just update the vels' when they are received we don't do anything here
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type diffdrive_vesc ::DiffDriveVescHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  can_.send(front_left_vesc_.create_rpm_cmd(front_left_vesc_.vel_to_rpm(front_left_vesc_.cmd)));
  can_.send(front_right_vesc_.create_rpm_cmd(-1.0*front_right_vesc_.vel_to_rpm(front_right_vesc_.cmd)));
  can_.send(back_left_vesc_.create_rpm_cmd(back_left_vesc_.vel_to_rpm(back_left_vesc_.cmd)));
  can_.send(back_right_vesc_.create_rpm_cmd(-1.0*back_right_vesc_.vel_to_rpm(back_right_vesc_.cmd)));
  return hardware_interface::return_type::OK;
}

}  // namespace diffdrive_vesc

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_vesc::DiffDriveVescHardware, hardware_interface::SystemInterface)
