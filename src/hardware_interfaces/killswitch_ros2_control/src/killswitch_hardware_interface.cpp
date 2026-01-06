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

#include "killswitch_ros2_control/killswitch_hardware_interface.hpp"

#include <fcntl.h>
#include <sstream>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace killswitch_ros2_control
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

int setup_gpio_input(int pin)
{
  // Export GPIO
  int fd = ::open("/sys/class/gpio/export", O_WRONLY);
  if (fd >= 0) {
    std::string pin_str = std::to_string(pin);
    ::write(fd, pin_str.c_str(), pin_str.length());
    ::close(fd);
    usleep(100000);  // Wait for GPIO to be exported
  }

  // Set direction to input
  std::stringstream direction_path;
  direction_path << "/sys/class/gpio/gpio" << pin << "/direction";
  fd = ::open(direction_path.str().c_str(), O_WRONLY);
  if (fd < 0) {
    return -1;
  }
  ::write(fd, "in", 2);
  ::close(fd);

  // Open value file
  std::stringstream value_path;
  value_path << "/sys/class/gpio/gpio" << pin << "/value";
  int value_fd = ::open(value_path.str().c_str(), O_RDONLY);
  
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

bool read_gpio(int fd)
{
  if (fd < 0) return false;
  
  char value;
  lseek(fd, 0, SEEK_SET);
  if (::read(fd, &value, 1) == 1) {
    return (value == '1');
  }
  return false;
}

}  // namespace gpio_utils

// Hardware Interface Implementation

hardware_interface::CallbackReturn KillswitchHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != 
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize state variables (all power ON, button not pressed)
  button_state_ = 0.0;
  main_power_killed_ = 0.0;
  jetson_power_killed_ = 0.0;
  all_power_killed_ = 0.0;
  is_connected_ = 0.0;

  // Initialize command variables (don't kill anything on startup)
  cmd_kill_main_power_ = 0.0;
  cmd_kill_jetson_power_ = 0.0;
  cmd_kill_all_power_ = 0.0;

  // Parse hardware parameters
  auto get_required_param = [&](const std::string& name) -> int {
    if (!info_.hardware_parameters.count(name)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("KillswitchHardwareInterface"),
        "%s parameter is required", name.c_str());
      return -1;
    }
    return std::stoi(info_.hardware_parameters.at(name));
  };

  button_gpio_pin_ = get_required_param("button_gpio_pin");
  main_power_gpio_pin_ = get_required_param("main_power_gpio_pin");
  jetson_power_gpio_pin_ = get_required_param("jetson_power_gpio_pin");
  all_power_gpio_pin_ = get_required_param("all_power_gpio_pin");

  if (button_gpio_pin_ < 0 || main_power_gpio_pin_ < 0 || 
      jetson_power_gpio_pin_ < 0 || all_power_gpio_pin_ < 0) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Active high/low for button (default: true)
  if (info_.hardware_parameters.count("active_high")) {
    active_high_ = (info_.hardware_parameters.at("active_high") == "true");
  } else {
    active_high_ = true;
  }

  // Initialize file descriptors
  button_fd_ = -1;
  main_power_fd_ = -1;
  jetson_power_fd_ = -1;
  all_power_fd_ = -1;
  hw_connected_ = false;

  RCLCPP_INFO(
    rclcpp::get_logger("KillswitchHardwareInterface"),
    "Initialized killswitch system - Button: GPIO%d, Main: GPIO%d, Jetson: GPIO%d, All: GPIO%d",
    button_gpio_pin_, main_power_gpio_pin_, jetson_power_gpio_pin_, all_power_gpio_pin_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KillswitchHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("KillswitchHardwareInterface"),
    "Configuring killswitch hardware...");

  // Setup GPIO input for button
  button_fd_ = gpio_utils::setup_gpio_input(button_gpio_pin_);
  if (button_fd_ < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("KillswitchHardwareInterface"),
      "Failed to setup button GPIO input on pin %d", button_gpio_pin_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Setup GPIO outputs for power relays (initially OFF = power ON)
  main_power_fd_ = gpio_utils::setup_gpio_output(main_power_gpio_pin_);
  if (main_power_fd_ < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("KillswitchHardwareInterface"),
      "Failed to setup main power GPIO output on pin %d", main_power_gpio_pin_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  gpio_utils::write_gpio(main_power_fd_, false);  // Initialize to LOW (power ON)

  jetson_power_fd_ = gpio_utils::setup_gpio_output(jetson_power_gpio_pin_);
  if (jetson_power_fd_ < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("KillswitchHardwareInterface"),
      "Failed to setup Jetson power GPIO output on pin %d", jetson_power_gpio_pin_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  gpio_utils::write_gpio(jetson_power_fd_, false);  // Initialize to LOW (power ON)

  all_power_fd_ = gpio_utils::setup_gpio_output(all_power_gpio_pin_);
  if (all_power_fd_ < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("KillswitchHardwareInterface"),
      "Failed to setup all-power GPIO output on pin %d", all_power_gpio_pin_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  gpio_utils::write_gpio(all_power_fd_, false);  // Initialize to LOW (power ON)

  hw_connected_ = true;
  is_connected_ = 1.0;

  RCLCPP_INFO(
    rclcpp::get_logger("KillswitchHardwareInterface"),
    "Successfully configured killswitch hardware");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> 
KillswitchHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Use the joint name from URDF
  const std::string& name = info_.joints[0].name;

  state_interfaces.emplace_back(
    hardware_interface::StateInterface(name, "button_state", &button_state_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(name, "main_power_killed", &main_power_killed_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(name, "jetson_power_killed", &jetson_power_killed_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(name, "all_power_killed", &all_power_killed_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(name, "is_connected", &is_connected_));

  RCLCPP_INFO(
    rclcpp::get_logger("KillswitchHardwareInterface"),
    "Exported %zu state interfaces", state_interfaces.size());

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> 
KillswitchHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Use the joint name from URDF
  const std::string& name = info_.joints[0].name;

  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(name, "kill_main_power", &cmd_kill_main_power_));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(name, "kill_jetson_power", &cmd_kill_jetson_power_));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(name, "kill_all_power", &cmd_kill_all_power_));

  RCLCPP_INFO(
    rclcpp::get_logger("KillswitchHardwareInterface"),
    "Exported %zu command interfaces", command_interfaces.size());

  return command_interfaces;
}

hardware_interface::CallbackReturn KillswitchHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("KillswitchHardwareInterface"),
    "Activating killswitch hardware...");

  // Reset commands to safe state (don't kill anything)
  cmd_kill_main_power_ = 0.0;
  cmd_kill_jetson_power_ = 0.0;
  cmd_kill_all_power_ = 0.0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KillswitchHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("KillswitchHardwareInterface"),
    "Deactivating killswitch hardware...");

  // Note: We intentionally do NOT kill power on deactivate
  // The killswitch should only respond to explicit commands
  // GPIO cleanup is done in on_cleanup

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KillswitchHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("KillswitchHardwareInterface"),
    "Cleaning up killswitch hardware...");

  // Cleanup GPIO - unexport and close file descriptors
  if (button_fd_ >= 0) {
    gpio_utils::cleanup_gpio(button_gpio_pin_, button_fd_);
    button_fd_ = -1;
  }
  if (main_power_fd_ >= 0) {
    gpio_utils::cleanup_gpio(main_power_gpio_pin_, main_power_fd_);
    main_power_fd_ = -1;
  }
  if (jetson_power_fd_ >= 0) {
    gpio_utils::cleanup_gpio(jetson_power_gpio_pin_, jetson_power_fd_);
    jetson_power_fd_ = -1;
  }
  if (all_power_fd_ >= 0) {
    gpio_utils::cleanup_gpio(all_power_gpio_pin_, all_power_fd_);
    all_power_fd_ = -1;
  }

  hw_connected_ = false;
  is_connected_ = 0.0;

  RCLCPP_INFO(
    rclcpp::get_logger("KillswitchHardwareInterface"),
    "Killswitch hardware cleanup complete");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KillswitchHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(
    rclcpp::get_logger("KillswitchHardwareInterface"),
    "Shutting down killswitch hardware...");

  // Delegate to cleanup to release resources
  return on_cleanup(previous_state);
}

hardware_interface::return_type KillswitchHardwareInterface::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (!hw_connected_) {
    return hardware_interface::return_type::OK;
  }

  // Read physical button state
  if (button_fd_ >= 0) {
    bool gpio_value = gpio_utils::read_gpio(button_fd_);
    bool is_pressed = active_high_ ? gpio_value : !gpio_value;
    
    double previous_state = button_state_;
    button_state_ = is_pressed ? 1.0 : 0.0;

    // Log state changes
    if (is_pressed && previous_state < 0.5) {
      RCLCPP_WARN(
        rclcpp::get_logger("KillswitchHardwareInterface"),
        "KILLSWITCH BUTTON PRESSED!");
    } else if (!is_pressed && previous_state > 0.5) {
      RCLCPP_INFO(
        rclcpp::get_logger("KillswitchHardwareInterface"),
        "Killswitch button released");
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KillswitchHardwareInterface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (!hw_connected_) {
    return hardware_interface::return_type::OK;
  }

  // Process kill_main_power command
  if (main_power_fd_ >= 0) {
    bool should_kill = (cmd_kill_main_power_ > 0.5);
    bool currently_killed = (main_power_killed_ > 0.5);
    
    if (should_kill != currently_killed) {
      gpio_utils::write_gpio(main_power_fd_, should_kill);
      main_power_killed_ = should_kill ? 1.0 : 0.0;
      
      if (should_kill) {
        RCLCPP_WARN(
          rclcpp::get_logger("KillswitchHardwareInterface"),
          "MAIN POWER KILLED");
      } else {
        RCLCPP_INFO(
          rclcpp::get_logger("KillswitchHardwareInterface"),
          "Main power restored");
      }
    }
  }

  // Process kill_jetson_power command
  if (jetson_power_fd_ >= 0) {
    bool should_kill = (cmd_kill_jetson_power_ > 0.5);
    bool currently_killed = (jetson_power_killed_ > 0.5);
    
    if (should_kill != currently_killed) {
      gpio_utils::write_gpio(jetson_power_fd_, should_kill);
      jetson_power_killed_ = should_kill ? 1.0 : 0.0;
      
      if (should_kill) {
        RCLCPP_WARN(
          rclcpp::get_logger("KillswitchHardwareInterface"),
          "JETSON POWER KILLED");
      } else {
        RCLCPP_INFO(
          rclcpp::get_logger("KillswitchHardwareInterface"),
          "Jetson power restored");
      }
    }
  }

  // Process kill_all_power command
  if (all_power_fd_ >= 0) {
    bool should_kill = (cmd_kill_all_power_ > 0.5);
    bool currently_killed = (all_power_killed_ > 0.5);
    
    if (should_kill != currently_killed) {
      gpio_utils::write_gpio(all_power_fd_, should_kill);
      all_power_killed_ = should_kill ? 1.0 : 0.0;
      
      if (should_kill) {
        RCLCPP_ERROR(
          rclcpp::get_logger("KillswitchHardwareInterface"),
          "ALL POWER KILLED");
      } else {
        RCLCPP_INFO(
          rclcpp::get_logger("KillswitchHardwareInterface"),
          "All power restored");
      }
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace killswitch_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  killswitch_ros2_control::KillswitchHardwareInterface,
  hardware_interface::SystemInterface)

