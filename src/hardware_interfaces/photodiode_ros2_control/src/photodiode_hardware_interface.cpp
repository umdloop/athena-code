#include "photodiode_ros2_control/photodiode_hardware_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <sstream>

namespace photodiode_ros2_control
{

void PhotodiodeHardwareInterface::logger_function()
{
  if (photodiode_gpios_.empty()) {
    return;
  }

  std::ostringstream oss;
  oss << "\033[2J\033[H \nPHOTODIODE Logger"
      << "\n--- HWI Specific ---\n"
      << "CAN Interface: " << can_interface_
      << " | HWI Update Rate: " << update_rate_
      << " | Logger Update Rate: " << logger_rate_ << "\n"
      << "Elapsed Time since first update: " << elapsed_time_ << "\n"
      << "\n--- GPIO Specific ---";

  for (const auto & gpio : photodiode_gpios_) {
    oss << "\nGPIO: " << gpio.name
        << " | CAN ID: 0x" << std::hex << std::uppercase << gpio.can_id
        << " | Node ID: 0x" << static_cast<int>(gpio.node_id) << std::dec << "\n"
        << "-- Commands --\n"
        << "Request Measurement Rate: " << gpio.request_measurement_rate
        << " | Status Request: " << gpio.status_request
        << " | Maintenance Request: " << gpio.maintenance_request << "\n"
        << "-- State --\n"
        << "Wavelength Intensity: " << gpio.wavelength_intensity
        << " | Command Success: " << gpio.command_success
        << " | Is Connected: " << gpio.is_connected
        << " | Status: " << gpio.status << "\n";
  }

  RCLCPP_INFO(rclcpp::get_logger("PhotodiodeHardwareInterface"), "%s", oss.str().c_str());
}

hardware_interface::CallbackReturn PhotodiodeHardwareInterface::on_init(
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
    std::stoi(info_.hardware_parameters.at("update_rate")) : 10;
  logger_rate_ = info_.hardware_parameters.count("logger_rate") ?
    std::stoi(info_.hardware_parameters.at("logger_rate")) : 5;
  logger_state_ = info_.hardware_parameters.count("logger_state") ?
    std::stoi(info_.hardware_parameters.at("logger_state")) : 0;
  can_id_ = info_.hardware_parameters.count("can_id") ?
    static_cast<uint32_t>(std::stoul(info_.hardware_parameters.at("can_id"), nullptr, 0)) : 0x110;

  if (info_.gpios.empty()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("PhotodiodeHardwareInterface"),
      "Photodiode hardware requires at least one configured gpio resource.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  photodiode_gpios_.clear();
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

    photodiode_gpios_.push_back(PhotodiodeGPIO{
      gpio.name,
      can_id_,
      node_id,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      false,
      0.0,
      state_if_names,
      command_if_names,
      params_map
    });
  }

  can_connected_ = false;
  elapsed_time_ = 0.0;
  elapsed_logger_time_ = 0.0;

  RCLCPP_INFO(
    rclcpp::get_logger("PhotodiodeHardwareInterface"),
    "Initialized photodiode on %s, CAN ID 0x%X",
    can_interface_.c_str(), can_id_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PhotodiodeHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    rclcpp::get_logger("PhotodiodeHardwareInterface"),
    "Configuring photodiode hardware...");

  if (can_connected_) {
    canBus_.close();
    can_connected_ = false;
  }

  if (!canBus_.open(
        can_interface_,
        std::bind(&PhotodiodeHardwareInterface::on_can_message, this, std::placeholders::_1)))
  {
    RCLCPP_WARN(
      rclcpp::get_logger("PhotodiodeHardwareInterface"),
      "Failed to open CAN interface %s - running in SIMULATION mode",
      can_interface_.c_str());
    can_connected_ = false;
  } else {
    can_connected_ = true;
    RCLCPP_INFO(
      rclcpp::get_logger("PhotodiodeHardwareInterface"),
      "Successfully opened CAN interface %s", can_interface_.c_str());
  }

  for (auto & gpio : photodiode_gpios_) {
    gpio.is_connected = can_connected_ ? 1.0 : 0.0;
    gpio.status = gpio.is_connected;
    gpio.command_success = 0.0;
    gpio.awaiting_response = false;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("PhotodiodeHardwareInterface"),
    "Photodiode hardware configured (%s)", can_connected_ ? "CAN MODE" : "SIMULATION");

  return hardware_interface::CallbackReturn::SUCCESS;
}

void PhotodiodeHardwareInterface::on_can_message(const CANLib::CanFrame & frame)
{
  if (frame.id != can_id_ || frame.dlc < 2) {
    return;
  }

  for (auto & gpio : photodiode_gpios_) {
    if (frame.data[0] != static_cast<uint8_t>(CMD_READ_PHOTODIODE_VALUE + gpio.node_id)) {
      continue;
    }

    gpio.wavelength_intensity = static_cast<double>(frame.data[1]);
    gpio.command_success = 1.0;
    gpio.status = 1.0;
    gpio.awaiting_response = false;
    RCLCPP_DEBUG(
      rclcpp::get_logger("PhotodiodeHardwareInterface"),
      "Photodiode value received for %s: %.0f",
      gpio.name.c_str(),
      gpio.wavelength_intensity);
    return;
  }
}

std::vector<hardware_interface::StateInterface> PhotodiodeHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto & gpio : photodiode_gpios_) {
    for (const auto & iface : gpio.state_interface_names) {
      double * value = nullptr;
      if (iface == "wavelength_intensity") {
        value = &gpio.wavelength_intensity;
      } else if (iface == "command_success") {
        value = &gpio.command_success;
      } else if (iface == "is_connected") {
        value = &gpio.is_connected;
      } else if (iface == "status") {
        value = &gpio.status;
      }

      if (value == nullptr) {
        continue;
      }
      state_interfaces.emplace_back(gpio.name, iface, value);
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> PhotodiodeHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto & gpio : photodiode_gpios_) {
    for (const auto & iface : gpio.command_interface_names) {
      double * value = nullptr;
      if (iface == "request_measurement_rate") {
        value = &gpio.request_measurement_rate;
      } else if (iface == "status_request") {
        value = &gpio.status_request;
      } else if (iface == "maintenance_request") {
        value = &gpio.maintenance_request;
      } else if (iface == "maintenance_frame_high") {
        value = &gpio.maintenance_frame_high;
      } else if (iface == "maintenance_frame_low") {
        value = &gpio.maintenance_frame_low;
      } else if (iface == "maintenance_data_count") {
        value = &gpio.maintenance_data_count;
      }

      if (value == nullptr) {
        continue;
      }
      command_interfaces.emplace_back(gpio.name, iface, value);
    }
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn PhotodiodeHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PhotodiodeHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (can_connected_) {
    canBus_.close();
    can_connected_ = false;
  }

  for (auto & gpio : photodiode_gpios_) {
    gpio.awaiting_response = false;
    gpio.request_measurement_rate = 0.0;
    gpio.prev_request_measurement_rate = 0.0;
    gpio.elapsed_request_measurement_time = 0.0;
    gpio.is_connected = 0.0;
    gpio.status = 0.0;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PhotodiodeHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  if (can_connected_) {
    canBus_.close();
  }

  can_connected_ = false;
  for (auto & gpio : photodiode_gpios_) {
    gpio.is_connected = 0.0;
    gpio.status = 0.0;
    gpio.wavelength_intensity = 0.0;
    gpio.command_success = 0.0;
    gpio.awaiting_response = false;
    gpio.request_measurement_rate = 0.0;
    gpio.prev_request_measurement_rate = 0.0;
    gpio.elapsed_request_measurement_time = 0.0;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PhotodiodeHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

hardware_interface::return_type PhotodiodeHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PhotodiodeHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  elapsed_time_ += period.seconds();
  elapsed_logger_time_ += period.seconds();
  if (logger_rate_ > 0 &&
    elapsed_logger_time_ > (1.0 / static_cast<double>(logger_rate_)))
  {
    elapsed_logger_time_ = 0.0;
    if (logger_state_ == 1) {
      logger_function();
    }
  }

  for (auto & gpio : photodiode_gpios_) {
    const double curr_measurement_req = gpio.request_measurement_rate;
    if (curr_measurement_req < 0.0 && gpio.prev_request_measurement_rate >= 0.0) {
      gpio.command_success = 0.0;
      gpio.awaiting_response = true;
      if (can_connected_) {
        can_tx_frame_ = CANLib::CanFrame();
        can_tx_frame_.id = gpio.can_id;
        can_tx_frame_.dlc = 2;
        can_tx_frame_.data[0] = static_cast<uint8_t>(CMD_READ_PHOTODIODE_VALUE + gpio.node_id);
        can_tx_frame_.data[1] = CONFIRM_SEND;
        canBus_.send(can_tx_frame_);
        RCLCPP_INFO(
          rclcpp::get_logger("PhotodiodeHardwareInterface"),
          "Requested photodiode measurement for %s", gpio.name.c_str());
      }
    } else if (curr_measurement_req > 0.0) {
      gpio.elapsed_request_measurement_time += period.seconds();
      if (gpio.elapsed_request_measurement_time > (1.0 / curr_measurement_req)) {
        gpio.elapsed_request_measurement_time = 0.0;
        gpio.command_success = 0.0;
        gpio.awaiting_response = true;
        if (can_connected_) {
          can_tx_frame_ = CANLib::CanFrame();
          can_tx_frame_.id = gpio.can_id;
          can_tx_frame_.dlc = 2;
          can_tx_frame_.data[0] = static_cast<uint8_t>(CMD_READ_PHOTODIODE_VALUE + gpio.node_id);
          can_tx_frame_.data[1] = CONFIRM_SEND;
          canBus_.send(can_tx_frame_);
          RCLCPP_INFO(
            rclcpp::get_logger("PhotodiodeHardwareInterface"),
            "Requested photodiode measurement for %s at %.3f Hz",
            gpio.name.c_str(),
            curr_measurement_req);
        }
      }
    }
    gpio.prev_request_measurement_rate = curr_measurement_req;

    const double curr_status_req = gpio.status_request;
    if (curr_status_req < 0.0 && gpio.prev_status_request >= 0.0) {
      if (can_connected_) {
        can_tx_frame_ = CANLib::CanFrame();
        can_tx_frame_.id = gpio.can_id;
        can_tx_frame_.dlc = 2;
        can_tx_frame_.data[0] = static_cast<uint8_t>(CMD_READ_PHOTODIODE_VALUE + gpio.node_id);
        can_tx_frame_.data[1] = CONFIRM_SEND;
        canBus_.send(can_tx_frame_);
      }
    } else if (curr_status_req > 0.0) {
      gpio.elapsed_status_request_time += period.seconds();
      if (gpio.elapsed_status_request_time > (1.0 / curr_status_req)) {
        gpio.elapsed_status_request_time = 0.0;
        if (can_connected_) {
          can_tx_frame_ = CANLib::CanFrame();
          can_tx_frame_.id = gpio.can_id;
          can_tx_frame_.dlc = 2;
          can_tx_frame_.data[0] = static_cast<uint8_t>(CMD_READ_PHOTODIODE_VALUE + gpio.node_id);
          can_tx_frame_.data[1] = CONFIRM_SEND;
          canBus_.send(can_tx_frame_);
        }
      }
    }
    gpio.prev_status_request = curr_status_req;

    auto doubles_to_payload = [](double high, double low) -> int64_t {
      return static_cast<int64_t>(
        (static_cast<uint64_t>(high) << 32) | static_cast<uint64_t>(low));
    };

    gpio.maintenance_frame = static_cast<double>(doubles_to_payload(
      gpio.maintenance_frame_high, gpio.maintenance_frame_low));
    const auto decoded = unpack_command_full(
      static_cast<int32_t>(gpio.maintenance_data_count),
      static_cast<int64_t>(gpio.maintenance_frame));

    if (decoded.u8_data.size() == 1 &&
      decoded.i16_data.empty() &&
      decoded.i32_data.empty())
    {
      CANLib::CanFrame maint_frame{};
      maint_frame.id = gpio.can_id;
      maint_frame.dlc = 2;
      maint_frame.data[0] = decoded.command_id;
      maint_frame.data[1] = decoded.u8_data[0];

      const double curr_maint_req = gpio.maintenance_request;
      if (curr_maint_req < 0.0 && gpio.prev_maintenance_request >= 0.0) {
        if (can_connected_) {
          canBus_.send(maint_frame);
        }
      } else if (curr_maint_req > 0.0) {
        gpio.elapsed_maintenance_request_time += period.seconds();
        if (gpio.elapsed_maintenance_request_time > (1.0 / curr_maint_req)) {
          gpio.elapsed_maintenance_request_time = 0.0;
          if (can_connected_) {
            canBus_.send(maint_frame);
          }
        }
      }
      gpio.prev_maintenance_request = gpio.maintenance_request;
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace photodiode_ros2_control

PLUGINLIB_EXPORT_CLASS(
  photodiode_ros2_control::PhotodiodeHardwareInterface,
  hardware_interface::SystemInterface)
