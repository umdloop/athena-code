#include "stepper_ros2_control/stepper_hardware_interface.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>

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
  node_ = rclcpp::Node::make_shared("stepper_hardware_node");
  science_can_publisher_ = node_->create_publisher<msgs::msg::CANA>("can_tx", rclcpp::QoS(10));
  science_can_subscriber_ = node_->create_subscription<msgs::msg::CANA>(
    "can_rx", rclcpp::QoS(50).reliable(),
    [this](const msgs::msg::CANA::SharedPtr message) {
      received_joint_data_ = *message;
    });
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STEPPERHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  for (const auto & joint : STEPPERJoints_) {
    msgs::msg::CANA frame;
    if (format_maintenance_command(
        frame, joint.node_id,
        DecodedCommand{static_cast<uint8_t>(MaintenanceCommands::MOTOR_SHUTDOWN_CMD), {}, {}, {}}))
    {
      science_can_publisher_->publish(frame);
    }
    if (format_maintenance_command(
        frame, joint.node_id,
        DecodedCommand{static_cast<uint8_t>(MaintenanceCommands::BRAKE_LOCK_CMD), {}, {}, {}}))
    {
      science_can_publisher_->publish(frame);
    }
  }

  science_can_publisher_.reset();
  science_can_subscriber_.reset();
  node_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STEPPERHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  for (auto & joint : STEPPERJoints_) {
    msgs::msg::CANA frame;
    if (format_maintenance_command(
        frame, joint.node_id,
        DecodedCommand{static_cast<uint8_t>(MaintenanceCommands::BRAKE_RELEASE_CMD), {}, {}, {}}))
    {
      science_can_publisher_->publish(frame);
    }
    joint.joint_command_position = joint.initial_position;
    joint.joint_command_velocity = 0.0;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STEPPERHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  for (const auto & joint : STEPPERJoints_) {
    msgs::msg::CANA frame;
    if (format_maintenance_command(
        frame, joint.node_id,
        DecodedCommand{static_cast<uint8_t>(MaintenanceCommands::MOTOR_STOP_CMD), {}, {}, {}}))
    {
      science_can_publisher_->publish(frame);
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type STEPPERHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!node_) {
    return hardware_interface::return_type::ERROR;
  }

  if (rclcpp::ok()) {
    rclcpp::spin_some(node_);
  }

  if (received_joint_data_.data.size() < 8) {
    return hardware_interface::return_type::OK;
  }

  for (auto & joint : STEPPERJoints_) {
    if (received_joint_data_.id != joint.node_id) {
      continue;
    }

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
