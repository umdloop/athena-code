// Copyright (c) 2024 UMD Loop
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
//

#ifndef LASER_ROS2_CONTROL__LASER_HARDWARE_INTERFACE_HPP_
#define LASER_ROS2_CONTROL__LASER_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace laser_ros2_control
{

// GPIO utility functions (Linux sysfs)
namespace gpio_utils
{
  int setup_gpio_output(int pin);
  void cleanup_gpio(int pin, int fd);
  bool write_gpio(int fd, bool value);
}

/**
 * @brief Hardware interface for GPIO-controlled laser via ros2_control
 * 
 * This interface controls a spectrometry laser through GPIO pins.
 * 
 * State Interfaces (read by controllers):
 * - laser_state: 0.0 = OFF, 1.0 = ON
 * - power_level: Current power level (0.0 to 1.0)
 * - is_ready: Is hardware connected and ready (0.0 or 1.0)
 * 
 * Command Interfaces (written by controllers):
 * - laser_command: 0.0 = turn OFF, 1.0 = turn ON
 * - power_command: Desired power level (0.0 to 1.0)
 * 
 * Hardware Parameters (from URDF):
 * - gpio_pin: GPIO pin number for laser control (required)
 * - min_power: Minimum power level (default: 0.0)
 * - max_power: Maximum power level (default: 1.0)
 */
class LaserHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(LaserHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // Configuration parameters
  int gpio_pin_;
  double min_power_;
  double max_power_;

  // State variables (hardware → ros2_control)
  double laser_state_;      // Current state: 0.0 = OFF, 1.0 = ON
  double power_level_;      // Current power level
  double is_ready_;         // Hardware ready status

  // Command variables (ros2_control → hardware)
  double laser_command_;    // Commanded state
  double power_command_;    // Commanded power level

  // GPIO interface
  int gpio_fd_;
  bool hw_connected_;
};

}  // namespace laser_ros2_control

#endif  // LASER_ROS2_CONTROL__LASER_HARDWARE_INTERFACE_HPP_

