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

#include "stepper_ros2_control/stepper_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace stepper_ros2_control
{

double STEPPERHardwareInterface::calculate_joint_position_from_motor_position(
  double motor_position, int gear_ratio)
{
  return (motor_position * 0.01 * (M_PI / 180.0)) / gear_ratio;
}

double STEPPERHardwareInterface::calculate_joint_velocity_from_motor_velocity(
  double motor_velocity, int gear_ratio)
{
  return (motor_velocity * (M_PI / 180.0)) / gear_ratio;
}

int32_t STEPPERHardwareInterface::calculate_motor_position_from_desired_joint_position(
  double joint_position, int gear_ratio)
{
  return static_cast<int32_t>(std::round((joint_position * (180.0 / M_PI) * 100.0) * gear_ratio));
}

int32_t STEPPERHardwareInterface::calculate_motor_velocity_from_desired_joint_velocity(
  double joint_velocity, int gear_ratio)
{
  return static_cast<int32_t>(std::round((joint_velocity * (180.0 / M_PI) * 100.0) * gear_ratio));
}

void STEPPERHardwareInterface::format_control_command(msgs::msg::CANA & frame, StepperJoint & joint)
{
  frame.id = joint.node_id;
  frame.data.assign(8, 0x00);

  if (
    joint.control_level == integration_level_t::POSITION &&
    std::isfinite(joint.joint_command_position) &&
    joint.joint_command_position != joint.prev_joint_command_position)
  {
    const int32_t joint_angle = joint.orientation *
      calculate_motor_position_from_desired_joint_position(
        joint.joint_command_position, joint.gear_ratio);
    frame.data[0] = static_cast<uint8_t>(ControlCommands::ABSOLUTE_POS_CONTROL_CMD);
    frame.data[2] = 200;
    frame.data[3] = 0x00;
    frame.data[4] = static_cast<uint8_t>(joint_angle & 0xFF);
    frame.data[5] = static_cast<uint8_t>((joint_angle >> 8) & 0xFF);
    frame.data[6] = static_cast<uint8_t>((joint_angle >> 16) & 0xFF);
    frame.data[7] = static_cast<uint8_t>((joint_angle >> 24) & 0xFF);
    joint.prev_joint_command_position = joint.joint_command_position;
    return;
  }

  if (
    joint.control_level == integration_level_t::VELOCITY &&
    std::isfinite(joint.joint_command_velocity) &&
    joint.joint_command_velocity != joint.prev_joint_command_velocity)
  {
    const int32_t joint_velocity = joint.orientation *
      calculate_motor_velocity_from_desired_joint_velocity(
        joint.joint_command_velocity, joint.gear_ratio);
    frame.data[0] = static_cast<uint8_t>(ControlCommands::SPEED_CONTROL_CMD);
    frame.data[4] = static_cast<uint8_t>(joint_velocity & 0xFF);
    frame.data[5] = static_cast<uint8_t>((joint_velocity >> 8) & 0xFF);
    frame.data[6] = static_cast<uint8_t>((joint_velocity >> 16) & 0xFF);
    frame.data[7] = static_cast<uint8_t>((joint_velocity >> 24) & 0xFF);
    joint.prev_joint_command_velocity = joint.joint_command_velocity;
    return;
  }

  frame.data[0] = static_cast<uint8_t>(StatusCommands::MOTOR_STATUS_2_CMD);
}

bool STEPPERHardwareInterface::format_status_command(
  msgs::msg::CANA & frame, uint8_t command_id, uint16_t node_id)
{
  frame.id = node_id;
  frame.data.assign(8, 0x00);
  switch (static_cast<StatusCommands>(command_id)) {
    case StatusCommands::READ_MULTI_TURN_ANGLE_CMD:
    case StatusCommands::MOTOR_STATUS_2_CMD:
      frame.data[0] = command_id;
      return true;
    default:
      return false;
  }
}

bool STEPPERHardwareInterface::format_maintenance_command(
  msgs::msg::CANA & frame, uint16_t node_id, const DecodedCommand & decoded_cmd)
{
  frame.id = node_id;
  frame.data.assign(8, 0x00);
  frame.data[0] = decoded_cmd.command_id;
  switch (static_cast<MaintenanceCommands>(decoded_cmd.command_id)) {
    case MaintenanceCommands::BRAKE_RELEASE_CMD:
    case MaintenanceCommands::BRAKE_LOCK_CMD:
    case MaintenanceCommands::MOTOR_SHUTDOWN_CMD:
    case MaintenanceCommands::MOTOR_STOP_CMD:
      return true;
    default:
      return false;
  }
void STEPPERHardwareInterface::logger_function()
{
  // Prevents breaking the logger
  if (num_joints == 0) return;

  // Building Message
  std::string log_msg = "\033[2J\033[H \nSTEPPER Logger";
  std::ostringstream oss;
  std::string status;

  // HWI Specific
  oss << "\n--- HWI Specific ---\n"
      << "CAN Interface: " << can_interface
      << " | Command CAN ID: 0x" << std::hex << std::uppercase << can_command_id
      << " | Response CAN ID: 0x" << std::hex << std::uppercase << can_response_id
      << " | HWI Update Rate: " << update_rate
      << " | Logger Update Rate: " << logger_rate << "\n"
      << "Elapsed Time since first update: " << elapsed_time << "\n"
      << "\n--- Joint Specific ---";

  for (int i = 0; i < num_joints; i++) {
    switch (device_status[i]) {
      case 0x00: status = "IDLE (ready)";      break;
      case 0x01: status = "ACTIVE (busy)";     break;
      case 0x02: status = "DOES NOT EXIST";    break;
      case 0x03: status = "ERROR";             break;
      default:   status = "UNDEFINED";         break;
    }

    oss << "\nJOINT: " << info_.joints[i].name << "\n"
        << "Parameters: Node ID: 0x" << std::hex << std::uppercase << joint_node_ids[i]
        << " | Gear Ratio: " << joint_gear_ratios[i]
        << " | Device Status: " << std::hex << std::uppercase << device_status[i]
        << " - " << status << "\n"
        << "-- Commands --\n"
        << "Control Mode: " << static_cast<int>(control_level_[i]) << "\n"
        << "Motor Position: " << motor_position[i]
        << " | Joint Command Position: " << joint_command_position_[i] << "\n"
        << "Motor Velocity: " << motor_velocity[i]
        << " | Joint Command Velocity: " << joint_command_velocity_[i] << "\n"
        << "-- State --\n"
        << "Joint Position: " << joint_state_position_[i]
        << " | Joint Velocity: " << joint_state_velocity_[i] << "\n";
  }

  log_msg += oss.str();
  RCLCPP_INFO(rclcpp::get_logger("STEPPERHardwareInterface"), log_msg.c_str());
}

hardware_interface::CallbackReturn STEPPERHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  STEPPERJoints_.clear();
  for (const auto & joint : info_.joints) {
    std::vector<std::string> state_if_names;
    for (const auto & si : joint.state_interfaces) {
      state_if_names.push_back(si.name);
    }

    std::vector<std::string> command_if_names;
    for (const auto & ci : joint.command_interfaces) {
      command_if_names.push_back(ci.name);
    }

    std::unordered_map<std::string, std::string> params_map;
    for (const auto & p : joint.parameters) {
      params_map.emplace(p.first, p.second);
    }

    const uint16_t node_id = static_cast<uint16_t>(
      joint.parameters.count("node_id") ? std::stoi(joint.parameters.at("node_id"), nullptr, 0) : 0);

    StepperJoint stepper_joint{};
    stepper_joint.name = joint.name;
    stepper_joint.node_id = node_id;
    stepper_joint.gear_ratio = std::abs(std::stoi(joint.parameters.at("gear_ratio")));
    stepper_joint.orientation = joint.parameters.count("joint_orientation") &&
      std::stoi(joint.parameters.at("joint_orientation")) == -1 ? -1 : 1;
    stepper_joint.initial_position = joint.parameters.count("initial_position") ?
      std::stod(joint.parameters.at("initial_position")) : 0.0;
    stepper_joint.control_level = integration_level_t::POSITION;
    stepper_joint.joint_state_position = 0.0;
    stepper_joint.joint_state_velocity = 0.0;
    stepper_joint.motor_temperature = 0.0;
    stepper_joint.motor_torque_current = 0.0;
    stepper_joint.motor_status = 0.0;
    stepper_joint.acceleration = std::numeric_limits<double>::quiet_NaN();
    stepper_joint.joint_command_position = 0.0;
    stepper_joint.joint_command_velocity = 0.0;
    stepper_joint.motor_status_req = 0.0;
    stepper_joint.motor_maintenance_req = 0.0;
    stepper_joint.maintenance_frame_high = 0.0;
    stepper_joint.maintenance_frame_low = 0.0;
    stepper_joint.maintenance_frame = 0.0;
    stepper_joint.maintenance_data_count = 0.0;
    stepper_joint.prev_status_req = 0.0;
    stepper_joint.prev_maintenance_req = 0.0;
    stepper_joint.elapsed_status_request_time = 0.0;
    stepper_joint.elapsed_maintenance_request_time = 0.0;
    stepper_joint.motor_position = 0.0;
    stepper_joint.motor_velocity = 0.0;
    stepper_joint.encoder_position = 0.0;
    stepper_joint.prev_joint_command_position = std::numeric_limits<double>::quiet_NaN();
    stepper_joint.prev_joint_command_velocity = std::numeric_limits<double>::quiet_NaN();
    stepper_joint.state_interface_names = state_if_names;
    stepper_joint.command_interface_names = command_if_names;
    stepper_joint.parameters = params_map;
    STEPPERJoints_.push_back(std::move(stepper_joint));
  }

  num_joints = static_cast<int>(STEPPERJoints_.size());
  current_iteration = 0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STEPPERHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

std::vector<hardware_interface::StateInterface> STEPPERHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto & joint : STEPPERJoints_) {
    for (const auto & iface : joint.state_interface_names) {
      double * value = nullptr;
      if (iface == hardware_interface::HW_IF_POSITION) {
        value = &joint.joint_state_position;
      } else if (iface == hardware_interface::HW_IF_VELOCITY) {
        value = &joint.joint_state_velocity;
      } else if (iface == "motor_temperature") {
        value = &joint.motor_temperature;
      } else if (iface == "torque_current") {
        value = &joint.motor_torque_current;
      } else if (iface == "status") {
        value = &joint.motor_status;
      }

      if (value != nullptr) {
        state_interfaces.emplace_back(joint.name, iface, value);
      }
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> STEPPERHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto & joint : STEPPERJoints_) {
    for (const auto & iface : joint.command_interface_names) {
      double * value = nullptr;
      if (iface == hardware_interface::HW_IF_POSITION) {
        value = &joint.joint_command_position;
      } else if (iface == hardware_interface::HW_IF_VELOCITY) {
        value = &joint.joint_command_velocity;
      } else if (iface == "status_request") {
        value = &joint.motor_status_req;
      } else if (iface == "maintenance_request") {
        value = &joint.motor_maintenance_req;
      } else if (iface == "maintenance_frame_high") {
        value = &joint.maintenance_frame_high;
      } else if (iface == "maintenance_frame_low") {
        value = &joint.maintenance_frame_low;
      } else if (iface == "maintenance_data_count") {
        value = &joint.maintenance_data_count;
      }

      if (value != nullptr) {
        command_interfaces.emplace_back(joint.name, iface, value);
      }
    }
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn STEPPERHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    rclcpp::get_logger("STEPPERHardwareInterface"),
    "Configuring stepper hardware...");

  if (!canBus_.open(can_interface,
      std::bind(&STEPPERHardwareInterface::onCanMessage, this, std::placeholders::_1)))
  {
    RCLCPP_WARN(
      rclcpp::get_logger("STEPPERHardwareInterface"),
      "Failed to open CAN interface %s - running in SIMULATION mode",
      can_interface.c_str());
    can_connected_ = false;
  } else {
    can_connected_ = true;
    RCLCPP_INFO(
      rclcpp::get_logger("STEPPERHardwareInterface"),
      "Successfully opened CAN interface %s", can_interface.c_str());
  }

  is_connected_ = can_connected_ ? 1.0 : 0.0;

  RCLCPP_INFO(
    rclcpp::get_logger("STEPPERHardwareInterface"),
    "Stepper hardware configured (%s)", can_connected_ ? "CAN MODE" : "SIMULATION");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Per protocol spec, response decoding uses 16-bit values (not 24-bit).
// All responses return: data[1-2] = position (int16, deg), data[3-4] = velocity (int16, deg/s).
void STEPPERHardwareInterface::onCanMessage(const CANLib::CanFrame& frame)
{
  // RCLCPP_INFO(
  //   rclcpp::get_logger("STEPPER"),
  //   "RX id=0x%X dlc=%d b0=0x%02X",
  //   frame.id, frame.dlc, frame.data[0]);

  can_rx_frame_ = frame;

  if (can_rx_frame_.id != can_response_id) {
    return;
  }

  uint8_t command_nibble  = (can_rx_frame_.data[0] >> 4) & 0x0F;
  uint8_t device_id_nibble = can_rx_frame_.data[0] & 0x0F;

  for (int i = 0; i < num_joints; i++) {
    if (device_id_nibble != static_cast<uint8_t>(joint_node_ids[i] & 0x0F)) {
      continue;
    }

    if (command_nibble == MOTOR_STATE_CMD) {
      // Per protocol spec, position and velocity are 16-bit signed (deg, deg/s), little endian
      int16_t raw_position = static_cast<int16_t>(
          can_rx_frame_.data[1] | (can_rx_frame_.data[2] << 8));
      int16_t raw_velocity = static_cast<int16_t>(
          can_rx_frame_.data[3] | (can_rx_frame_.data[4] << 8));

      // Convert deg -> rad and apply gear ratio
      motor_position[i] = (static_cast<double>(raw_position) * M_PI / 180.0) / joint_gear_ratios[i];
      motor_velocity[i] = (static_cast<double>(raw_velocity) * M_PI / 180.0) / joint_gear_ratios[i];

    } else if (command_nibble == MOTOR_STATUS_CMD) {
      device_status[i] = can_rx_frame_.data[1];
    }

    break;  // Matched joint, no need to keep iterating
  }
}

hardware_interface::CallbackReturn STEPPERHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    rclcpp::get_logger("STEPPERHardwareInterface"),
    "Activating ...please wait...");

  joint_command_position_ = joint_state_position_;

  for (size_t i = 0; i < joint_command_position_.size(); ++i) {
    RCLCPP_INFO(
      rclcpp::get_logger("STEPPERHardwareInterface"),
      "Joint %zu command position initialized to: %f", i, joint_command_position_[i]);
  }

  RCLCPP_INFO(
    rclcpp::get_logger("STEPPERHardwareInterface"),
    "Stepper hardware activated");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STEPPERHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    rclcpp::get_logger("STEPPERHardwareInterface"),
    "Deactivating stepper hardware...");

  if (can_connected_) {
    for (int i = 0; i < num_joints; i++) {
      can_tx_frame_     = CANLib::CanFrame();
      can_tx_frame_.id  = can_command_id;
      can_tx_frame_.dlc = 2;

      // (MAINTENANCE_CMD << 4) | port_id = 0x60 | port_id, maintenance cmd 1 = Stop stepper
      uint8_t device_id_nibble = joint_node_ids[i] & 0x0F;
      can_tx_frame_.data[0] = static_cast<uint8_t>((MAINTENANCE_CMD << 4) | device_id_nibble);
      can_tx_frame_.data[1] = 1;  // Maintenance cmd 1 = Stop stepper
      canBus_.send(can_tx_frame_);
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("STEPPERHardwareInterface"),
    "Stepper hardware deactivated%s", can_connected_ ? "" : " (simulated)");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// can_connected_ and is_connected_ are always reset regardless of whether CAN was open.
hardware_interface::CallbackReturn STEPPERHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("STEPPERHardwareInterface"),
    "Cleaning up stepper hardware...");

  if (can_connected_) {
    for (int i = 0; i < num_joints; i++) {
      can_tx_frame_     = CANLib::CanFrame();
      can_tx_frame_.id  = can_command_id;
      can_tx_frame_.dlc = 2;

      // FIX #4: Assign device_id_nibble (was previously using uninitialized variable)
      uint8_t device_id_nibble = joint_node_ids[i] & 0x0F;
      can_tx_frame_.data[0] = static_cast<uint8_t>((MAINTENANCE_CMD << 4) | device_id_nibble);
      can_tx_frame_.data[1] = 2;  // Maintenance cmd 2 = Shutdown stepper
      canBus_.send(can_tx_frame_);
    }

    canBus_.close();
  }

  // Always reset connection state
  can_connected_ = false;
  is_connected_  = 0.0;

  RCLCPP_INFO(
    rclcpp::get_logger("STEPPERHardwareInterface"),
    "Stepper hardware cleanup complete");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type STEPPERHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (can_connected_) {
    current_joint_ = (current_joint_ + 1) % num_joints;

    can_tx_frame_     = CANLib::CanFrame();
    can_tx_frame_.id  = can_command_id;
    can_tx_frame_.dlc = 2;

    // MOTOR_STATE_CMD = 0x4 → (0x4 << 4) | port_id = 0x40 | port_id
    uint8_t port_id = joint_node_ids[current_joint_] & 0x0F;
    can_tx_frame_.data[0] = static_cast<uint8_t>((MOTOR_STATE_CMD << 4) | port_id);
    can_tx_frame_.data[1] = 0x01;  // Validate the request
    canBus_.send(can_tx_frame_);
  }

  // Copy motor state (updated asynchronously by onCanMessage) into joint state
  for (int i = 0; i < num_joints; i++) {
    joint_state_position_[i] = motor_position[i];
    joint_state_velocity_[i] = motor_velocity[i];

    // Return error on any fault status
    if (device_status[i] != 0x00 && device_status[i] != 0x01 && device_status[i] != -1) {
      RCLCPP_ERROR(
        rclcpp::get_logger("STEPPERHardwareInterface"),
        "Joint %s has fault device_status=0x%02X", info_.joints[i].name.c_str(), device_status[i]);
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type STEPPERHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  elapsed_update_time += period.seconds();
  double update_period = 1.0 / update_rate;

  // Rate-limit CAN messages to configured update rate
  if (elapsed_update_time < update_period) {
    return hardware_interface::return_type::OK;
  }

  elapsed_update_time = 0.0;
  elapsed_time += period.seconds();

  for (int i = 0; i < num_joints; i++) {
    if (control_level_[i] == integration_level_t::VELOCITY &&
        std::isfinite(joint_command_velocity_[i]))
    {
      // Stepper only accepts three discrete speeds: +900, -900, or 0 (deg/s).
      // Map the signed velocity command to the nearest valid value.
      int16_t speed_dps;
      if (joint_command_velocity_[i] > 0.0) {
        speed_dps = 900;
      } else if (joint_command_velocity_[i] < 0.0) {
        speed_dps = -900;
      } else {
        speed_dps = 0;
      }

      if (can_connected_) {
        can_tx_frame_     = CANLib::CanFrame();
        can_tx_frame_.id  = can_command_id;
        can_tx_frame_.dlc = 3;  // 3 bytes per spec

        // VELOCITY_CONTROL_CMD = 0x3 → (0x3 << 4) | port_id = 0x30 | port_id
        uint8_t port_id = joint_node_ids[i] & 0x0F;
        can_tx_frame_.data[0] = static_cast<uint8_t>((VELOCITY_CONTROL_CMD & 0xF0) | port_id);
        can_tx_frame_.data[1] = static_cast<uint8_t>(speed_dps & 0xFF);         // low byte
        can_tx_frame_.data[2] = static_cast<uint8_t>((speed_dps >> 8) & 0xFF);  // high byte

        RCLCPP_INFO(rclcpp::get_logger("STEPPERHardwareInterface"), "Data[0]: 0x%02X | Data[1-2] (speed_dps): %d | Joint: %s",
          can_tx_frame_.data[0], speed_dps, info_.joints[i].name.c_str());

        canBus_.send(can_tx_frame_);
      }
    }
    else if (control_level_[i] == integration_level_t::POSITION &&
             std::isfinite(joint_command_position_[i]))
    {
      // Convert rad -> deg, apply gear ratio, clamp to int16 range
      int16_t position_deg = static_cast<int16_t>(std::clamp(
        joint_command_position_[i] * (180.0 / M_PI) * joint_gear_ratios[i],
        static_cast<double>(std::numeric_limits<int16_t>::min()),
        static_cast<double>(std::numeric_limits<int16_t>::max())
      ));

    if (received_joint_data_.data[0] == static_cast<uint8_t>(StatusCommands::MOTOR_STATUS_2_CMD)) {
      joint.encoder_position = static_cast<double>(
        static_cast<int16_t>((received_joint_data_.data[7] << 8) | received_joint_data_.data[6]));
      joint.motor_velocity = static_cast<double>(
        static_cast<int16_t>((received_joint_data_.data[5] << 8) | received_joint_data_.data[4]));
      joint.motor_temperature = static_cast<double>(received_joint_data_.data[1]);
      joint.motor_torque_current = static_cast<double>(
        static_cast<int16_t>((received_joint_data_.data[3] << 8) | received_joint_data_.data[2])) * 0.01;
      joint.motor_status = 1.0;
    } else if (
      received_joint_data_.data[0] == static_cast<uint8_t>(StatusCommands::READ_MULTI_TURN_ANGLE_CMD))
    {
      joint.motor_position = static_cast<double>(
        static_cast<int32_t>(
          (received_joint_data_.data[7] << 24) |
          (received_joint_data_.data[6] << 16) |
          (received_joint_data_.data[5] << 8) |
          received_joint_data_.data[4]));
    }

    joint.joint_state_position = calculate_joint_position_from_motor_position(
      joint.motor_position, joint.gear_ratio);
    joint.joint_state_velocity = calculate_joint_velocity_from_motor_velocity(
      joint.motor_velocity, joint.gear_ratio);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type STEPPERHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  (void)period;

  for (auto & joint : STEPPERJoints_) {
    const double curr_status_req = joint.motor_status_req;
    if (curr_status_req < 0.0 && joint.prev_status_req >= 0.0) {
      for (auto status_cmd : kStatusCommands) {
        msgs::msg::CANA frame;
        if (format_status_command(frame, static_cast<uint8_t>(status_cmd), joint.node_id)) {
          science_can_publisher_->publish(frame);
        }
      }
    } else if (curr_status_req > 0.0) {
      joint.elapsed_status_request_time += period.seconds();
      if (joint.elapsed_status_request_time > (1.0 / curr_status_req)) {
        joint.elapsed_status_request_time = 0.0;
        for (auto status_cmd : kStatusCommands) {
          msgs::msg::CANA frame;
          if (format_status_command(frame, static_cast<uint8_t>(status_cmd), joint.node_id)) {
            science_can_publisher_->publish(frame);
          }
        }
      }
    }
    joint.prev_status_req = curr_status_req;
  }

  for (auto & joint : STEPPERJoints_) {
    auto doubles_to_payload = [](double high, double low) -> int64_t {
      return static_cast<int64_t>(
        (static_cast<uint64_t>(high) << 32) | static_cast<uint64_t>(low));
    };

    joint.maintenance_frame = static_cast<double>(doubles_to_payload(
      joint.maintenance_frame_high, joint.maintenance_frame_low));
    const auto decoded_maintenance_cmd = unpack_command_full(
      static_cast<int32_t>(joint.maintenance_data_count),
      static_cast<int64_t>(joint.maintenance_frame));
    pack_decoded_maintenance_frame(joint, decoded_maintenance_cmd);

    msgs::msg::CANA frame;
    if (!format_maintenance_command(frame, joint.node_id, decoded_maintenance_cmd)) {
      joint.prev_maintenance_req = joint.motor_maintenance_req;
      continue;
    }

    const double curr_maintenance_req = joint.motor_maintenance_req;
    if (curr_maintenance_req < 0.0 && joint.prev_maintenance_req >= 0.0) {
      science_can_publisher_->publish(frame);
    } else if (curr_maintenance_req > 0.0) {
      joint.elapsed_maintenance_request_time += period.seconds();
      if (joint.elapsed_maintenance_request_time > (1.0 / curr_maintenance_req)) {
        joint.elapsed_maintenance_request_time = 0.0;
        science_can_publisher_->publish(frame);
      }
    }
    joint.prev_maintenance_req = curr_maintenance_req;
  }

  for (auto & joint : STEPPERJoints_) {
    msgs::msg::CANA frame;
    format_control_command(frame, joint);
    science_can_publisher_->publish(frame);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type STEPPERHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  std::vector<integration_level_t> requested_modes(
    static_cast<size_t>(num_joints), integration_level_t::UNDEFINED);

  for (const auto & ifname : stop_interfaces) {
    for (size_t i = 0; i < STEPPERJoints_.size(); ++i) {
      const auto & joint = STEPPERJoints_[i];
      const std::string pos_if = joint.name + "/" + hardware_interface::HW_IF_POSITION;
      const std::string vel_if = joint.name + "/" + hardware_interface::HW_IF_VELOCITY;
      if (ifname == pos_if || ifname == vel_if || ifname.find(joint.name) != std::string::npos) {
        requested_modes[i] = integration_level_t::UNDEFINED;
      }
    }
  }

  for (const auto & ifname : start_interfaces) {
    for (size_t i = 0; i < STEPPERJoints_.size(); ++i) {
      const auto & joint = STEPPERJoints_[i];
      const std::string pos_if = joint.name + "/" + hardware_interface::HW_IF_POSITION;
      const std::string vel_if = joint.name + "/" + hardware_interface::HW_IF_VELOCITY;
      if (ifname == pos_if) {
        requested_modes[i] = integration_level_t::POSITION;
      } else if (ifname == vel_if) {
        requested_modes[i] = integration_level_t::VELOCITY;
      }
    }
  }

  for (size_t i = 0; i < STEPPERJoints_.size(); ++i) {
    auto & joint = STEPPERJoints_[i];
    if (requested_modes[i] == integration_level_t::UNDEFINED) {
      bool was_stopped = false;
      for (const auto & ifname : stop_interfaces) {
        if (ifname.find(joint.name) != std::string::npos) {
          was_stopped = true;
          break;
        }
      }
      if (was_stopped) {
        joint.control_level = integration_level_t::UNDEFINED;
        joint.joint_command_velocity = 0.0;
        joint.joint_command_position = joint.joint_state_position;
      }
      continue;
    }

    joint.control_level = requested_modes[i];
    if (joint.control_level == integration_level_t::VELOCITY) {
      joint.joint_command_velocity = 0.0;
    } else {
      joint.joint_command_position = joint.joint_state_position;
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace stepper_ros2_control

PLUGINLIB_EXPORT_CLASS(
  stepper_ros2_control::STEPPERHardwareInterface, hardware_interface::SystemInterface)
