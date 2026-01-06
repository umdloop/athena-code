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

#ifndef KILLSWITCH_ROS2_CONTROL__KILLSWITCH_HARDWARE_INTERFACE_HPP_
#define KILLSWITCH_ROS2_CONTROL__KILLSWITCH_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace killswitch_ros2_control
{

// GPIO utility functions (Linux sysfs)
namespace gpio_utils
{
  int setup_gpio_input(int pin);
  int setup_gpio_output(int pin);
  void cleanup_gpio(int pin, int fd);
  bool read_gpio(int fd);
  bool write_gpio(int fd, bool value);
}

/**
 * @brief Hardware interface for drive killswitch system via ros2_control
 * 
 * This is a SystemInterface that monitors a physical killswitch button
 * and controls power kill relays for main power, Jetson power, and all power.
 * 
 * State Interfaces (read by controllers):
 * - button_state: Physical button state (0.0 = not pressed, 1.0 = pressed)
 * - main_power_killed: Main power relay state (0.0 = on, 1.0 = killed)
 * - jetson_power_killed: Jetson power relay state (0.0 = on, 1.0 = killed)
 * - all_power_killed: All power relay state (0.0 = on, 1.0 = killed)
 * - is_connected: Hardware ready status
 * 
 * Command Interfaces (written by controllers):
 * - kill_main_power: Set to 1.0 to kill main power, 0.0 to restore
 * - kill_jetson_power: Set to 1.0 to kill Jetson power, 0.0 to restore
 * - kill_all_power: Set to 1.0 to kill all power, 0.0 to restore
 * 
 * Hardware Parameters (from URDF):
 * - button_gpio_pin: GPIO pin for killswitch button input (required)
 * - main_power_gpio_pin: GPIO pin for main power relay output (required)
 * - jetson_power_gpio_pin: GPIO pin for Jetson power relay output (required)
 * - all_power_gpio_pin: GPIO pin for all-power relay output (required)
 * - active_high: If true, HIGH = button pressed (default: true)
 */
class KillswitchHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KillswitchHardwareInterface)

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
  // Configuration parameters (GPIO pins)
  int button_gpio_pin_;
  int main_power_gpio_pin_;
  int jetson_power_gpio_pin_;
  int all_power_gpio_pin_;
  bool active_high_;

  // State variables (hardware → ros2_control)
  double button_state_;           // Physical button: 0.0 = not pressed, 1.0 = pressed
  double main_power_killed_;      // 0.0 = power on, 1.0 = killed
  double jetson_power_killed_;    // 0.0 = power on, 1.0 = killed
  double all_power_killed_;       // 0.0 = power on, 1.0 = killed
  double is_connected_;           // Hardware ready status

  // Command variables (ros2_control → hardware)
  double cmd_kill_main_power_;
  double cmd_kill_jetson_power_;
  double cmd_kill_all_power_;

  // GPIO file descriptors
  int button_fd_;
  int main_power_fd_;
  int jetson_power_fd_;
  int all_power_fd_;
  bool hw_connected_;
};

}  // namespace killswitch_ros2_control

#endif  // KILLSWITCH_ROS2_CONTROL__KILLSWITCH_HARDWARE_INTERFACE_HPP_

