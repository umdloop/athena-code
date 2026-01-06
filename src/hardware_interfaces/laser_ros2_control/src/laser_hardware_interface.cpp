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

#include "laser_ros2_control/laser_hardware_interface.hpp"

#include <cmath>
#include <fcntl.h>
#include <sstream>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace laser_ros2_control
{

// GPIO utility functions implementation
namespace gpio_utils
{

int setup_gpio_output(int pin)
{
  // Export GPIO
  int fd = ::open("/sys/class/gpio/export", O_WRONLY);
  if (fd >= 0) {
    std::string pin_str = std::to_string(pin);
    ::write(fd, pin_str.c_str(), pin_str.length());
    ::close(fd);
    usleep(100000);  // Wait for GPIO to be exported
  }

  // Set direction to output
  std::stringstream direction_path;
  direction_path << "/sys/class/gpio/gpio" << pin << "/direction";
  fd = ::open(direction_path.str().c_str(), O_WRONLY);
  if (fd < 0) {
    return -1;
  }
  ::write(fd, "out", 3);
  ::close(fd);

  // Open value file
  std::stringstream value_path;
  value_path << "/sys/class/gpio/gpio" << pin << "/value";
  int value_fd = ::open(value_path.str().c_str(), O_RDWR);
  
  return value_fd;
}

void cleanup_gpio(int pin, int fd)
{
  if (fd >= 0) {
    ::close(fd);
  }

  // Unexport GPIO
  int export_fd = ::open("/sys/class/gpio/unexport", O_WRONLY);
  if (export_fd >= 0) {
    std::string pin_str = std::to_string(pin);
    ::write(export_fd, pin_str.c_str(), pin_str.length());
    ::close(export_fd);
  }
}

bool write_gpio(int fd, bool value)
{
  if (fd < 0) return false;
  
  char val = value ? '1' : '0';
  lseek(fd, 0, SEEK_SET);
  return (::write(fd, &val, 1) == 1);
}

}  // namespace gpio_utils

// Hardware Interface Implementation

hardware_interface::CallbackReturn LaserHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != 
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize state variables
  laser_state_ = 0.0;      // OFF
  power_level_ = 0.0;
  is_ready_ = 0.0;

  // Initialize command variables
  laser_command_ = 0.0;    // OFF
  power_command_ = 0.0;

  // Parse hardware parameters
  if (!info_.hardware_parameters.count("gpio_pin")) {
    RCLCPP_ERROR(
      rclcpp::get_logger("LaserHardwareInterface"),
      "gpio_pin parameter is required");
    return hardware_interface::CallbackReturn::ERROR;
  }
  gpio_pin_ = std::stoi(info_.hardware_parameters.at("gpio_pin"));
  
  if (info_.hardware_parameters.count("min_power")) {
    min_power_ = std::stod(info_.hardware_parameters.at("min_power"));
  } else {
    min_power_ = 0.0;
  }
  
  if (info_.hardware_parameters.count("max_power")) {
    max_power_ = std::stod(info_.hardware_parameters.at("max_power"));
  } else {
    max_power_ = 1.0;
  }

  gpio_fd_ = -1;
  hw_connected_ = false;

  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Initialized laser on GPIO pin %d", gpio_pin_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Configuring laser hardware...");

  // Setup GPIO output
  gpio_fd_ = gpio_utils::setup_gpio_output(gpio_pin_);
  if (gpio_fd_ < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("LaserHardwareInterface"),
      "Failed to setup GPIO output on pin %d", gpio_pin_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_connected_ = true;
  is_ready_ = 1.0;

  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Successfully configured laser hardware");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> 
LaserHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Use the joint name from URDF
  const std::string& name = info_.joints[0].name;

  // Laser state (ON/OFF)
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(name, "laser_state", &laser_state_));

  // Power level
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(name, "power_level", &power_level_));

  // Hardware ready status
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(name, "is_ready", &is_ready_));

  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Exported %zu state interfaces", state_interfaces.size());

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> 
LaserHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Use the joint name from URDF
  const std::string& name = info_.joints[0].name;

  // Laser command (ON/OFF)
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(name, "laser_command", &laser_command_));

  // Power command
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(name, "power_command", &power_command_));

  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Exported %zu command interfaces", command_interfaces.size());

  return command_interfaces;
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Activating laser hardware...");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Deactivating laser hardware...");

  // Safety: Turn laser OFF on deactivation
  if (hw_connected_ && gpio_fd_ >= 0) {
    gpio_utils::write_gpio(gpio_fd_, false);
    laser_state_ = 0.0;
    power_level_ = 0.0;
    
    RCLCPP_INFO(
      rclcpp::get_logger("LaserHardwareInterface"),
      "Laser turned OFF (safety)");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Cleaning up laser hardware...");

  // Ensure laser is OFF before cleanup
  if (gpio_fd_ >= 0) {
    gpio_utils::write_gpio(gpio_fd_, false);
  }

  // Cleanup GPIO - unexport and close file descriptor
  if (gpio_fd_ >= 0) {
    gpio_utils::cleanup_gpio(gpio_pin_, gpio_fd_);
    gpio_fd_ = -1;
  }

  hw_connected_ = false;
  is_ready_ = 0.0;

  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Laser hardware cleanup complete");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Shutting down laser hardware...");

  // Delegate to cleanup to release resources
  return on_cleanup(previous_state);
}

hardware_interface::return_type LaserHardwareInterface::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // Laser state is known from commands, no need to read from GPIO
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LaserHardwareInterface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (!hw_connected_ || gpio_fd_ < 0) {
    return hardware_interface::return_type::OK;
  }

  // Check if laser command changed
  bool commanded_on = (laser_command_ > 0.5);
  bool currently_on = (laser_state_ > 0.5);

  if (commanded_on != currently_on) {
    if (gpio_utils::write_gpio(gpio_fd_, commanded_on)) {
      laser_state_ = commanded_on ? 1.0 : 0.0;
      
      RCLCPP_INFO(
        rclcpp::get_logger("LaserHardwareInterface"),
        "Laser turned %s", commanded_on ? "ON" : "OFF");
    }
  }

  // Update power level (clamped to min/max)
  double commanded_power = power_command_;
  if (commanded_power < min_power_) commanded_power = min_power_;
  if (commanded_power > max_power_) commanded_power = max_power_;
  
  if (std::abs(commanded_power - power_level_) > 0.01) {
    power_level_ = commanded_power;
    
    RCLCPP_DEBUG(
      rclcpp::get_logger("LaserHardwareInterface"),
      "Power level set to %.2f", power_level_);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace laser_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  laser_ros2_control::LaserHardwareInterface,
  hardware_interface::SystemInterface)

