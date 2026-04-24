#include "led_ros2_control/led_hardware_interface.hpp"

#include <fcntl.h>
#include <sstream>
#include <unistd.h>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace led_ros2_control
{

namespace gpio_utils
{

int setup_gpio_output(int pin)
{
  int fd = ::open("/sys/class/gpio/export", O_WRONLY);
  if (fd >= 0) {
    const std::string pin_str = std::to_string(pin);
    ::write(fd, pin_str.c_str(), pin_str.length());
    ::close(fd);
    usleep(100000);
  }

  std::stringstream direction_path;
  direction_path << "/sys/class/gpio/gpio" << pin << "/direction";
  fd = ::open(direction_path.str().c_str(), O_WRONLY);
  if (fd < 0) {
    return -1;
  }
  ::write(fd, "out", 3);
  ::close(fd);

  std::stringstream value_path;
  value_path << "/sys/class/gpio/gpio" << pin << "/value";
  return ::open(value_path.str().c_str(), O_RDWR);
}

void cleanup_gpio(int pin, int fd)
{
  if (fd >= 0) {
    ::close(fd);
  }

  int export_fd = ::open("/sys/class/gpio/unexport", O_WRONLY);
  if (export_fd >= 0) {
    const std::string pin_str = std::to_string(pin);
    ::write(export_fd, pin_str.c_str(), pin_str.length());
    ::close(export_fd);
  }
}

bool write_gpio(int fd, bool value)
{
  if (fd < 0) {
    return false;
  }
  const char val = value ? '1' : '0';
  lseek(fd, 0, SEEK_SET);
  return (::write(fd, &val, 1) == 1);
}

}  // namespace gpio_utils

hardware_interface::CallbackReturn LEDHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!info_.hardware_parameters.count("gpio_pin")) {
    RCLCPP_ERROR(rclcpp::get_logger("LEDHardwareInterface"), "gpio_pin parameter is required");
    return hardware_interface::CallbackReturn::ERROR;
  }

  const int gpio_pin = std::stoi(info_.hardware_parameters.at("gpio_pin"));
  const bool default_state = info_.hardware_parameters.count("default_state") &&
    info_.hardware_parameters.at("default_state") == "on";

  LEDJoints_.clear();
  LEDJoints_.push_back(LEDJoint{
    info_.gpios[0].name,
    gpio_pin,
    default_state,
    default_state ? 1.0 : 0.0,
    0.0,
    0.0,
    default_state ? 1.0 : 0.0,
    0.0,
    0.0,
    0.0
  });

  gpio_fd_ = -1;
  hw_connected_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LEDHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto & joint = LEDJoints_.front();
  gpio_fd_ = gpio_utils::setup_gpio_output(joint.gpio_pin);
  hw_connected_ = gpio_fd_ >= 0;
  if (hw_connected_) {
    gpio_utils::write_gpio(gpio_fd_, joint.default_state);
  }
  joint.is_connected = hw_connected_ ? 1.0 : 0.0;
  joint.status = joint.is_connected;
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> LEDHardwareInterface::export_state_interfaces()
{
  auto & joint = LEDJoints_.front();
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(joint.name, "led_state", &joint.led_state);
  state_interfaces.emplace_back(joint.name, "is_connected", &joint.is_connected);
  state_interfaces.emplace_back(joint.name, "status", &joint.status);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> LEDHardwareInterface::export_command_interfaces()
{
  auto & joint = LEDJoints_.front();
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(joint.name, "led_command", &joint.led_command);
  command_interfaces.emplace_back(joint.name, "status_request", &joint.status_request);
  return command_interfaces;
}

hardware_interface::CallbackReturn LEDHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LEDHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  auto & joint = LEDJoints_.front();
  if (hw_connected_ && gpio_fd_ >= 0) {
    gpio_utils::write_gpio(gpio_fd_, false);
  }
  joint.led_state = 0.0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LEDHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  if (gpio_fd_ >= 0) {
    gpio_utils::write_gpio(gpio_fd_, false);
    gpio_utils::cleanup_gpio(LEDJoints_.front().gpio_pin, gpio_fd_);
    gpio_fd_ = -1;
  }
  hw_connected_ = false;
  auto & joint = LEDJoints_.front();
  joint.is_connected = 0.0;
  joint.status = 0.0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LEDHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

hardware_interface::return_type LEDHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LEDHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  auto & joint = LEDJoints_.front();

  const bool commanded_on = joint.led_command > 0.5;
  const bool currently_on = joint.led_state > 0.5;
  if (commanded_on != currently_on) {
    if (hw_connected_ && gpio_fd_ >= 0) {
      gpio_utils::write_gpio(gpio_fd_, commanded_on);
    }
    joint.led_state = commanded_on ? 1.0 : 0.0;
  }

  if (joint.status_request < 0.0 && joint.prev_status_request >= 0.0) {
    joint.status = hw_connected_ ? 1.0 : 0.0;
  } else if (joint.status_request > 0.0) {
    joint.elapsed_status_request_time += period.seconds();
    if (joint.elapsed_status_request_time > (1.0 / joint.status_request)) {
      joint.elapsed_status_request_time = 0.0;
      joint.status = hw_connected_ ? 1.0 : 0.0;
    }
  }
  joint.prev_status_request = joint.status_request;

  return hardware_interface::return_type::OK;
}

}  // namespace led_ros2_control

PLUGINLIB_EXPORT_CLASS(
  led_ros2_control::LEDHardwareInterface, hardware_interface::SystemInterface)
