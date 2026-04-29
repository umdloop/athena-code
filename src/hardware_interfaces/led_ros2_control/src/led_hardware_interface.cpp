#include "led_ros2_control/led_hardware_interface.hpp"

#include <fcntl.h>
#include <sstream>
#include <unistd.h>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace led_ros2_control
{

void LEDHardwareInterface::logger_function()
{
  if (LEDGPIOs_.empty()) {
    return;
  }

  const auto & gpio = LEDGPIOs_.front();
  std::ostringstream oss;
  oss << "\033[2J\033[H \nLED Logger"
      << "\n--- HWI Specific ---\n"
      << "HWI Update Rate: " << update_rate_
      << " | Logger Update Rate: " << logger_rate_ << "\n"
      << "Elapsed Time since first update: " << elapsed_time_ << "\n"
      << "\n--- GPIO Specific ---\n"
      << "GPIO: " << gpio.name << "\n"
      << "-- Commands --\n"
      << "Red Command: " << gpio.red_command
      << " | Green Command: " << gpio.green_command
      << " | Blue Command: " << gpio.blue_command
      << " | Status Request: " << gpio.status_request << "\n"
      << "-- State --\n"
      << "Status: " << gpio.status << "\n";

  RCLCPP_INFO(rclcpp::get_logger("LEDHardwareInterface"), "%s", oss.str().c_str());
}

hardware_interface::CallbackReturn LEDHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  // General HWI parameters
  can_interface_ = info_.hardware_parameters.count("can_interface") ?
    info_.hardware_parameters.at("can_interface") : "can0";
  update_rate_ = info_.hardware_parameters.count("update_rate") ?
    std::stoi(info_.hardware_parameters.at("update_rate")) : 5;
  logger_rate_ = info_.hardware_parameters.count("logger_rate") ?
    std::stoi(info_.hardware_parameters.at("logger_rate")) : 0;
  logger_state_ = info_.hardware_parameters.count("logger_state") ?
    std::stoi(info_.hardware_parameters.at("logger_state")) : 0;
  if(info_.hardware_parameters.count("can_id")) {
    const uint32_t can_id = static_cast<uint32_t>(std::stoul(info_.hardware_parameters.at("can_id"), nullptr, 0));
  }
  else {
    RCLCPP_ERROR(rclcpp::get_logger("PowerModuleHardwareInterface"), "CAN ID parameter 'can_id' is missing or invalid in hardware configuration.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (auto& gpio : info_.gpios) {
    // Collect state interface names
    std::vector<std::string> state_if_names;
    for (const auto &si : gpio.state_interfaces) {
      state_if_names.push_back(si.name);
    }

    // Collect command interface names
    std::vector<std::string> command_if_names;
    for (const auto &ci : gpio.command_interfaces) {
      command_if_names.push_back(ci.name);
    }

    // Copy parameters into an unordered_map<string,string>
    std::unordered_map<std::string, std::string> params_map;
    for (const auto &p : gpio.parameters) {
      params_map.emplace(p.first, p.second);
    }

    LEDGPIOs_.push_back(
      LEDGPIO{
        .name = info_.gpios[0].name,
        .is_rgb = false,
        .status = 0.0,
        .intensity = 0.0,
        .red_command = 0.0,
        .green_command = 0.0,
        .blue_command = 0.0,
        .status_request = 0.0,
        .prev_status_request = 0.0,
        .elapsed_status_request_time = 0.0,

        .state_interface_names = state_if_names,
        .command_interface_names = command_if_names,
        .parameters = params_map
      }
    );
  }

  elapsed_time_ = 0.0;
  elapsed_logger_time_ = 0.0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LEDHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> LEDHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto & gpio : LEDGPIOs_) {
    for (auto & iface : gpio.state_interface_names) {
      double * val = nullptr;
      if (iface == "status") {
        val = &gpio.status;
      } else {
        RCLCPP_WARN(
          rclcpp::get_logger("LEDHardwareInterface"), 
          "Unknown state interface '%s' for GPIO '%s'", 
          iface.c_str(), gpio.name.c_str());
        continue;
      }
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        gpio.name, iface, val));
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> LEDHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto & gpio : LEDGPIOs_) {
    for (auto & iface : gpio.command_interface_names) {
      double * val = nullptr;
      if (iface == "red") {
        val = &gpio.red_command;
      } else if (iface == "green") {
        val = &gpio.green_command;
      } else if (iface == "blue") {
        val = &gpio.blue_command;
      } else if (iface == "status_request") {
        val = &gpio.status_request;
      } else if (iface == "intensity") {
        if(gpio.is_rgb) {
          RCLCPP_ERROR(
            rclcpp::get_logger("LEDHardwareInterface"), 
            "Intensity command interface is not supported for RGB GPIO '%s'", 
            gpio.name.c_str());
            continue;
        }
        else {
          val = &gpio.intensity;
        }
      } else {
        RCLCPP_WARN(
          rclcpp::get_logger("LEDHardwareInterface"), 
          "Unknown command interface '%s' for GPIO '%s'", 
          iface.c_str(), gpio.name.c_str());
        continue;
      }
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        gpio.name, iface, val));
    }
  }

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
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LEDHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  auto & gpio = LEDGPIOs_.front();
  gpio.status = 0.0;
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
  auto & gpio = LEDGPIOs_.front();

  elapsed_time_ += period.seconds();
  elapsed_logger_time_ += period.seconds();
  if (logger_rate_ > 0 && elapsed_logger_time_ > (1.0 / static_cast<double>(logger_rate_))) {
    elapsed_logger_time_ = 0.0;
    if (logger_state_ == 1) {
      logger_function();
    }
  }

  if (gpio.status_request < 0.0 && gpio.prev_status_request >= 0.0) {
  } else if (gpio.status_request > 0.0) {
    gpio.elapsed_status_request_time += period.seconds();
    if (gpio.elapsed_status_request_time > (1.0 / gpio.status_request)) {
      gpio.elapsed_status_request_time = 0.0;
    }
  }
  gpio.prev_status_request = gpio.status_request;

  return hardware_interface::return_type::OK;
}

}  // namespace led_ros2_control

PLUGINLIB_EXPORT_CLASS(
  led_ros2_control::LEDHardwareInterface, hardware_interface::SystemInterface)
