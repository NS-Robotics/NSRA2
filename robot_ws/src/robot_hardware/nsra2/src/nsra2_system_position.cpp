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

#include "nsra2_hardware/nsra2_system_position.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace nsra2_hardware
{
    hardware_interface::return_type NSRA2SystemPositionHardware::configure(
            const hardware_interface::HardwareInfo &info)
    {
        if (configure_default(info) != hardware_interface::return_type::OK)
        {
            return hardware_interface::return_type::ERROR;
        }

        hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
        hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
        hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
        hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        for (const hardware_interface::ComponentInfo &joint: info_.joints)
        {
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                        rclcpp::get_logger("NSRA2SystemPositionHardware"),
                        "Joint '%s' has %d command interfaces found. 1 expected.", joint.name.c_str(),
                        joint.command_interfaces.size());
                return hardware_interface::return_type::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                        rclcpp::get_logger("NSRA2SystemPositionHardware"),
                        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::return_type::ERROR;
            }

            if (joint.state_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                        rclcpp::get_logger("NSRA2SystemPositionHardware"),
                        "Joint '%s' has %d state interface. 1 expected.", joint.name.c_str(),
                        joint.state_interfaces.size());
                return hardware_interface::return_type::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                        rclcpp::get_logger("NSRA2SystemPositionHardware"),
                        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
                        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::return_type::ERROR;
            }
        }

        this->hw_interface = new nsra2_control::NSRA2Control();
        if (this->hw_interface->init() != nsra2_control::NSRA_SERIAL_DEVICE_READY)
        {
            RCLCPP_FATAL(
                    rclcpp::get_logger("NSRA2SystemPositionHardware"),
                    "Error: NSRA2 Hardware Interface not found"
                    );
            return hardware_interface::return_type::ERROR;
        }
        else
            RCLCPP_INFO(rclcpp::get_logger("NSRA2SystemPositionHardware"), "NSRA2 Hardware Interface connected");

        status_ = hardware_interface::status::CONFIGURED;
        return hardware_interface::return_type::OK;
    }

    std::vector<hardware_interface::StateInterface>
    NSRA2SystemPositionHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface>
    NSRA2SystemPositionHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
        }

        return command_interfaces;
    }

    hardware_interface::return_type NSRA2SystemPositionHardware::start()
    {
        /*
        RCLCPP_INFO(rclcpp::get_logger("NSRA2SystemPositionHardware"), "Starting ...please wait...");

        for (int i = 0; i < hw_start_sec_; i++)
        {
            rclcpp::sleep_for(std::chrono::seconds(1));
            RCLCPP_INFO(
                    rclcpp::get_logger("NSRA2SystemPositionHardware"), "%.1f seconds left...",
                    hw_start_sec_ - i);
        }
         */
        RCLCPP_INFO(rclcpp::get_logger("NSRA2SystemPositionHardware"), "Starting...");

        // set some default values when starting the first time
        for (uint i = 0; i < hw_states_.size(); i++)
        {
            if (std::isnan(hw_states_[i]))
            {
                hw_states_[i] = 0;
                hw_commands_[i] = 0;
            }
            else
            {
                hw_commands_[i] = hw_states_[i];
            }
        }

        status_ = hardware_interface::status::STARTED;

        RCLCPP_INFO(
                rclcpp::get_logger("NSRA2SystemPositionHardware"), "System Successfully started!");

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type NSRA2SystemPositionHardware::stop()
    {
        /*
        RCLCPP_INFO(rclcpp::get_logger("NSRA2SystemPositionHardware"), "Stopping ...please wait...");

        for (int i = 0; i < hw_stop_sec_; i++)
        {
            rclcpp::sleep_for(std::chrono::seconds(1));
            RCLCPP_INFO(
                    rclcpp::get_logger("NSRA2SystemPositionHardware"), "%.1f seconds left...",
                    hw_stop_sec_ - i);
        }
         */
        RCLCPP_INFO(rclcpp::get_logger("NSRA2SystemPositionHardware"), "Stopping...");

        status_ = hardware_interface::status::STOPPED;

        RCLCPP_INFO(
                rclcpp::get_logger("NSRA2SystemPositionHardware"), "System successfully stopped!");

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type NSRA2SystemPositionHardware::read()
    {

        for (uint i = 0; i < hw_states_.size(); i++)
        {
            // Simulate RRBot's movement
            hw_states_[i] = hw_commands_[i];
            // RCLCPP_INFO(
            //   rclcpp::get_logger("NSRA2SystemPositionHardware"), "Got state %.5f for joint %d!",
            //   hw_states_[i], i);
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type NSRA2SystemPositionHardware::write()
    {
        if (this->hw_interface->sendCommands(this->hw_commands_) == nsra2_control::NSRA_STATUS_SUCCESS)
            return hardware_interface::return_type::OK;
        else
            return hardware_interface::return_type::ERROR;
    }

}  // namespace nsra2_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
        nsra2_hardware::NSRA2SystemPositionHardware, hardware_interface::SystemInterface)