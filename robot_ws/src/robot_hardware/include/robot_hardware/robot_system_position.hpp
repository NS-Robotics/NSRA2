// Copyright 2020 ros2_control Development Team
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

#ifndef ROBOT_HARDWARE__ROBOT_SYSTEM_POSITION_ONLY_HPP_
#define ROBOT_HARDWARE__ROBOT_SYSTEM_POSITION_ONLY_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp/macros.hpp"
#include "robot_hardware/visibility_control.h"

#include "rclcpp/rclcpp.hpp"

#include "robot_descriptions/msg/init.hpp"
#include "robot_descriptions/msg/pos.hpp"
#include "robot_descriptions/msg/vel.hpp"
#include "robot_descriptions/msg/stop.hpp"

namespace robot_hardware
{
class RobotSystemPositionOnlyHardware
: public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RobotSystemPositionOnlyHardware);

  ROBOT_HARDWARE_PUBLIC
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

  ROBOT_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROBOT_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROBOT_HARDWARE_PUBLIC
  hardware_interface::return_type start() override;

  ROBOT_HARDWARE_PUBLIC
  hardware_interface::return_type stop() override;

  ROBOT_HARDWARE_PUBLIC
  hardware_interface::return_type read() override;

  ROBOT_HARDWARE_PUBLIC
  hardware_interface::return_type write() override;

private:
  // Parameters for the RRBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;

};

}  // namespace robot_hardware

#endif  // ROBOT_HARDWARE__ROBOT_SYSTEM_POSITION_ONLY_HPP_