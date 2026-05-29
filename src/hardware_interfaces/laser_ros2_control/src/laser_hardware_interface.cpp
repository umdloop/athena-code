#include "laser_ros2_control/laser_hardware_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <sstream>

namespace laser_ros2_control
{

void LaserHardwareInterface::logger_function()
{
  if (LASERGPIOs_.empty()) {
    return;
  }

  std::ostringstream oss;
  oss << "\033[2J\033[H \nLASER Logger"
      << "\n--- HWI Specific ---\n"
      << "CAN Interface: " << can_interface_
      << " | HWI Update Rate: " << update_rate_
      << " | Logger Update Rate: " << logger_rate_ << "\n"
      << "Elapsed Time since first update: " << elapsed_time_ << "\n"
      << "\n--- Joint Specific ---";

  for (const auto & joint : LASERGPIOs_) {
    oss << "\nJOINT: " << joint.name
        << " | CAN ID: 0x" << std::hex << std::uppercase << joint.can_id
        << " | Node ID: 0x" << static_cast<int>(joint.node_id) << std::dec << "\n"
        << "-- Commands --\n"
        << "Laser Command: " << joint.laser_command
        << " | Status Request: " << joint.status_request << "\n"
        << "-- State --\n"
        << "Laser State: " << joint.laser_state
        << " | Temperature: " << joint.temperature
        << " | Is Connected: " << joint.is_connected
        << " | Status: " << joint.status << "\n";
  }

  RCLCPP_INFO(rclcpp::get_logger("LaserHardwareInterface"), "%s", oss.str().c_str());
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  can_interface_ = info_.hardware_parameters.count("can_interface") ?
    info_.hardware_parameters.at("can_interface") : "can0";
  update_rate_ = info_.hardware_parameters.count("update_rate") ?
    std::stoi(info_.hardware_parameters.at("update_rate")) : 0;
  logger_rate_ = info_.hardware_parameters.count("logger_rate") ?
    std::stoi(info_.hardware_parameters.at("logger_rate")) : 0;
  logger_state_ = info_.hardware_parameters.count("logger_state") ?
    std::stoi(info_.hardware_parameters.at("logger_state")) : 0;

  can_id_ = info_.hardware_parameters.count("can_id") ?
    static_cast<uint32_t>(std::stoul(info_.hardware_parameters.at("can_id"), nullptr, 0)) : 0x130;

  if (info_.gpios.empty()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("LaserHardwareInterface"),
      "Laser hardware requires at least one configured gpio resource.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  LASERGPIOs_.clear();
  for (const auto & gpio : info_.gpios) {
    std::vector<std::string> state_if_names;
    for (const auto & si : gpio.state_interfaces) {
      state_if_names.push_back(si.name);
    }

    std::vector<std::string> command_if_names;
    for (const auto & ci : gpio.command_interfaces) {
      command_if_names.push_back(ci.name);
    }

    std::unordered_map<std::string, std::string> params_map;
    for (const auto & p : gpio.parameters) {
      params_map.emplace(p.first, p.second);
    }

    const uint8_t node_id = static_cast<uint8_t>(
      std::clamp(std::stoi(gpio.parameters.at("node_id"), nullptr, 0), 0, 15));

    LASERGPIOs_.push_back(LASERGPIO{
      gpio.name,
      can_id_,
      node_id,
      0.0,
      0.0,
      0.0,
      std::numeric_limits<double>::quiet_NaN(),
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      state_if_names,
      command_if_names,
      params_map
    });
  }

  can_connected_       = false;
  elapsed_time_        = 0.0;
  elapsed_logger_time_ = 0.0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Configuring laser hardware...");

  // Close existing connection if re-configuring
  if (can_connected_) {
    canBus_.close();
    can_connected_ = false;
  }

  if (!canBus_.open(
        can_interface_,
        std::bind(&LaserHardwareInterface::on_can_message, this, std::placeholders::_1)))
  {
    RCLCPP_WARN(
      rclcpp::get_logger("LaserHardwareInterface"),
      "Failed to open CAN interface %s - running in SIMULATION mode",
      can_interface_.c_str());
    can_connected_ = false;
  } else {
    can_connected_ = true;
    RCLCPP_INFO(
      rclcpp::get_logger("LaserHardwareInterface"),
      "Successfully opened CAN interface %s", can_interface_.c_str());
  }

  for (auto & joint : LASERGPIOs_) {
    joint.is_connected = can_connected_ ? 1.0 : 0.0;
    joint.status = joint.is_connected;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Laser hardware configured (%s)", can_connected_ ? "CAN MODE" : "SIMULATION");

  return hardware_interface::CallbackReturn::SUCCESS;
}

void LaserHardwareInterface::on_can_message(const CANLib::CanFrame & frame)
{
  if (frame.id != can_id_ || frame.dlc < 2) {
    return;
  }

  for (auto & joint : LASERGPIOs_) {
    if (frame.data[0] == static_cast<uint8_t>(CMD_LASER_CONTROL + joint.node_id)) {
      const bool success = frame.data[1] != 0;
      joint.status = success ? 1.0 : 0.0;
      if (success) {
        joint.laser_state = joint.laser_command > 0.5 ? 1.0 : 0.0;
      }
      RCLCPP_INFO(
        rclcpp::get_logger("LaserHardwareInterface"),
        "Laser control %s from CAN for %s", success ? "confirmed" : "failed", joint.name.c_str());
      return;
    }

    if (frame.data[0] == static_cast<uint8_t>(CMD_LASER_STATUS + joint.node_id)) {
      const bool success = frame.data[1] != 0;
      joint.status = success ? 1.0 : 0.0;
      RCLCPP_INFO(
        rclcpp::get_logger("LaserHardwareInterface"),
        "Laser status response for %s: %s", joint.name.c_str(), success ? "OK" : "FAILED");
      return;
    }
  }
}

std::vector<hardware_interface::StateInterface> LaserHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto & joint : LASERGPIOs_) {
    for (const auto & iface : joint.state_interface_names) {
      double * value = nullptr;
      if (iface == "laser_state") {
        value = &joint.laser_state;
      } else if (iface == "temperature") {
        value = &joint.temperature;
      } else if (iface == "is_connected") {
        value = &joint.is_connected;
      } else if (iface == "status") {
        value = &joint.status;
      }

      if (value == nullptr) {
        continue;
      }
      state_interfaces.emplace_back(joint.name, iface, value);
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> LaserHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto & joint : LASERGPIOs_) {
    for (const auto & iface : joint.command_interface_names) {
      double * value = nullptr;
      if (iface == "laser_command") {
        value = &joint.laser_command;
      } else if (iface == "status_request") {
        value = &joint.status_request;
      }

      if (value == nullptr) {
        continue;
      }
      command_interfaces.emplace_back(joint.name, iface, value);
    }
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (can_connected_) {
    for (const auto & joint : LASERGPIOs_) {
      can_tx_frame_ = CANLib::CanFrame();
      can_tx_frame_.id = joint.can_id;
      can_tx_frame_.dlc = 2;
      can_tx_frame_.data[0] = static_cast<uint8_t>(CMD_LASER_CONTROL + joint.node_id);
      can_tx_frame_.data[1] = 0;
      canBus_.send(can_tx_frame_);
    }

    canBus_.close();
    can_connected_ = false;
  }

  for (auto & joint : LASERGPIOs_) {
    joint.is_connected = 0.0;
    joint.laser_state = 0.0;
    joint.laser_command = 0.0;
    joint.prev_laser_command = 0.0;
    joint.status = 0.0;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  if (can_connected_) {
    for (const auto & joint : LASERGPIOs_) {
      can_tx_frame_ = CANLib::CanFrame();
      can_tx_frame_.id = joint.can_id;
      can_tx_frame_.dlc = 2;
      can_tx_frame_.data[0] = static_cast<uint8_t>(CMD_LASER_CONTROL + joint.node_id);
      can_tx_frame_.data[1] = 0;
      canBus_.send(can_tx_frame_);
    }
    canBus_.close();
  }

  can_connected_ = false;
  for (auto & joint : LASERGPIOs_) {
    joint.is_connected = 0.0;
    joint.laser_state = 0.0;
    joint.laser_command = 0.0;
    joint.prev_laser_command = 0.0;
    joint.status = 0.0;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

hardware_interface::return_type LaserHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LaserHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  elapsed_time_        += period.seconds();
  elapsed_logger_time_ += period.seconds();
  if (logger_rate_ > 0 &&
      elapsed_logger_time_ > (1.0 / static_cast<double>(logger_rate_)))
  {
    elapsed_logger_time_ = 0.0;
    if (logger_state_ == 1) {
      logger_function();
    }
  }

  for (auto & joint : LASERGPIOs_) {
    const bool command_changed = joint.laser_command != joint.prev_laser_command;
    if (command_changed) {
      const bool commanded_on = joint.laser_command > 0.5;
      if (can_connected_) {
        can_tx_frame_ = CANLib::CanFrame();
        can_tx_frame_.id = joint.can_id;
        can_tx_frame_.dlc = 2;
        can_tx_frame_.data[0] = static_cast<uint8_t>(CMD_LASER_CONTROL + joint.node_id);
        can_tx_frame_.data[1] = commanded_on ? 1 : 0;
        canBus_.send(can_tx_frame_);
        RCLCPP_INFO(
          rclcpp::get_logger("LaserHardwareInterface"),
          "Laser command sent for %s: %s", joint.name.c_str(), commanded_on ? "ON" : "OFF");
      } else {
        joint.laser_state = commanded_on ? 1.0 : 0.0;
        RCLCPP_INFO(
          rclcpp::get_logger("LaserHardwareInterface"),
          "Laser turned %s for %s (simulated)",
          commanded_on ? "ON" : "OFF",
          joint.name.c_str());
      }

      joint.prev_laser_command = joint.laser_command;
    }

    const double curr_status_req = joint.status_request;
    if (curr_status_req < 0.0 && joint.prev_status_request >= 0.0) {
      if (can_connected_) {
        can_tx_frame_ = CANLib::CanFrame();
        can_tx_frame_.id = joint.can_id;
        can_tx_frame_.dlc = 2;
        can_tx_frame_.data[0] = static_cast<uint8_t>(CMD_LASER_STATUS + joint.node_id);
        can_tx_frame_.data[1] = 1;
        canBus_.send(can_tx_frame_);
      }
    } else if (curr_status_req > 0.0) {
      joint.elapsed_status_request_time += period.seconds();
      if (joint.elapsed_status_request_time > (1.0 / curr_status_req)) {
        joint.elapsed_status_request_time = 0.0;
        if (can_connected_) {
          can_tx_frame_ = CANLib::CanFrame();
          can_tx_frame_.id = joint.can_id;
          can_tx_frame_.dlc = 2;
          can_tx_frame_.data[0] = static_cast<uint8_t>(CMD_LASER_STATUS + joint.node_id);
          can_tx_frame_.data[1] = 1;
          canBus_.send(can_tx_frame_);
        }
      }
    }
    joint.prev_status_request = curr_status_req;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace laser_ros2_control

PLUGINLIB_EXPORT_CLASS(
  laser_ros2_control::LaserHardwareInterface,
  hardware_interface::SystemInterface)
