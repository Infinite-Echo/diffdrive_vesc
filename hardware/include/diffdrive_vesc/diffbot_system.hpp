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

#ifndef DIFFDRIVE_VESC__DIFFBOT_SYSTEM_HPP_
#define DIFFDRIVE_VESC__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>
#include <functional>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "can_interface/can_interface.hpp"
#include "diffdrive_vesc/vesc.hpp"

namespace diffdrive_vesc
{
class DiffDriveVescHardware : public hardware_interface::SystemInterface
{

struct Config
{
  uint32_t front_left_vesc_id = 0;
  uint32_t front_right_vesc_id = 1;
  uint32_t back_left_vesc_id = 2;
  uint32_t back_right_vesc_id = 3;
  double gear_ratio = 0.0;
  std::string device = "";
};

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffDriveVescHardware);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// Get the logger of the SystemInterface.
  /**
   * \return logger of the SystemInterface.
   */
  rclcpp::Logger get_logger() const { return *logger_; }

  /// Get the clock of the SystemInterface.
  /**
   * \return clock of the SystemInterface.
   */
  rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

private:

  CANInterface can_;
  Config cfg_;
  VESC front_left_vesc_;
  VESC front_right_vesc_;
  VESC back_left_vesc_;
  VESC back_right_vesc_;

  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;
};
}  // namespace diffdrive_vesc

#endif  // DIFFDRIVE_VESC__DIFFBOT_SYSTEM_HPP_
