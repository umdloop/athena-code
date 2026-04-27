#include "power_module_ros2_control/power_module_hardware_interface.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include <sstream>

namespace power_module_ros2_control
{

void PowerModuleHardwareInterface::send_command(int can_id, int cmd_id){
  CANLib::CanFrame frame;
  frame.id  = can_id;
  frame.dlc = 2;
  frame.data.fill(0);
  frame.data[0] = cmd_id;
  frame.data[1] = CONFIRM_SEND;
  canBus_.send(frame);
}

void PowerModuleHardwareInterface::logger_function()
{
  if (PowerModuleJoints_.empty()) {
    return;
  }

  const auto & joint = PowerModuleJoints_.front();
  std::ostringstream oss;
  oss << "\033[2J\033[H \nPowerModule Logger"
      << "\n--- HWI Specific ---\n"
      << "CAN Interface: " << can_interface_
      << " | HWI Update Rate: " << update_rate_
      << " | Logger Update Rate: " << logger_rate_ << "\n"
      << "Elapsed Time since first update: " << elapsed_time_ << "\n"
      << "\n--- Joint Specific ---\n"
      << "JOINT: " << joint.name << "\n"
      << "Parameters: CAN ID: 0x" << std::hex << std::uppercase << joint.can_id << std::dec << "\n"
      << "-- Commands --\n"
      << "Kill All: " << joint.cmd_kill_all
      << " | Kill Main: " << joint.cmd_kill_main
      << " | Kill Jetson: " << joint.cmd_kill_jetson << "\n"
      << "Status Request: " << joint.status_request << "\n"
      << "-- State --\n"
      << "Kill All Sent: " << joint.kill_all_sent
      << " | Kill Main Sent: " << joint.kill_main_sent
      << " | Kill Jetson Sent: " << joint.kill_jetson_sent << "\n"
      << "Is Connected: " << joint.is_connected
      << " | Status: " << joint.status << "\n";

  RCLCPP_INFO(rclcpp::get_logger("PowerModuleHardwareInterface"), "%s", oss.str().c_str());
}

hardware_interface::CallbackReturn PowerModuleHardwareInterface::on_init(
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
  const uint32_t can_id = info_.hardware_parameters.count("can_id") ?
    static_cast<uint32_t>(std::stoul(info_.hardware_parameters.at("can_id"), nullptr, 0)) : 0x100;

  PowerModuleJoints_.clear();
  PowerModuleJoints_.push_back(PowerModuleJoint{
    info_.gpios[0].name,
    can_id,
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
    0.0
  });

  can_connected_ = false;
  elapsed_time_ = 0.0;
  elapsed_logger_time_ = 0.0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PowerModuleHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!canBus_.open(
      can_interface_,
      std::bind(&PowerModuleHardwareInterface::on_can_message, this, std::placeholders::_1)))
  {
    can_connected_ = false;
  } else {
    can_connected_ = true;
  }
  auto & joint = PowerModuleJoints_.front();
  joint.is_connected = can_connected_ ? 1.0 : 0.0;
  joint.status = joint.is_connected;
  return hardware_interface::CallbackReturn::SUCCESS;
}

void PowerModuleHardwareInterface::on_can_message(const CANLib::CanFrame &)
{
}

std::vector<hardware_interface::StateInterface> PowerModuleHardwareInterface::export_state_interfaces()
{
  auto & joint = PowerModuleJoints_.front();
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(joint.name, "kill_all_sent", &joint.kill_all_sent);
  state_interfaces.emplace_back(joint.name, "kill_main_sent", &joint.kill_main_sent);
  state_interfaces.emplace_back(joint.name, "kill_jetson_sent", &joint.kill_jetson_sent);
  state_interfaces.emplace_back(joint.name, "is_connected", &joint.is_connected);
  state_interfaces.emplace_back(joint.name, "status", &joint.status);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> PowerModuleHardwareInterface::export_command_interfaces()
{
  auto & joint = PowerModuleJoints_.front();
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(joint.name, "kill_all", &joint.cmd_kill_all);
  command_interfaces.emplace_back(joint.name, "kill_main", &joint.cmd_kill_main);
  command_interfaces.emplace_back(joint.name, "kill_jetson", &joint.cmd_kill_jetson);
  command_interfaces.emplace_back(joint.name, "status_request", &joint.status_request);
  return command_interfaces;
}

hardware_interface::CallbackReturn PowerModuleHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  auto & joint = PowerModuleJoints_.front();
  joint.cmd_kill_all = 0.0;
  joint.cmd_kill_main = 0.0;
  joint.cmd_kill_jetson = 0.0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PowerModuleHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PowerModuleHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  if (can_connected_) {
    canBus_.close();
  }
  can_connected_ = false;
  auto & joint = PowerModuleJoints_.front();
  joint.is_connected = 0.0;
  joint.status = 0.0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PowerModuleHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

hardware_interface::return_type PowerModuleHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PowerModuleHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  auto & joint = PowerModuleJoints_.front();

  elapsed_time_ += period.seconds();
  elapsed_logger_time_ += period.seconds();
  if (logger_rate_ > 0 && elapsed_logger_time_ > (1.0 / static_cast<double>(logger_rate_))) {
    elapsed_logger_time_ = 0.0;
    if (logger_state_ == 1) {
      logger_function();
    }
  }

  if (joint.cmd_kill_all > 0.5 && joint.kill_all_sent < 0.5) {
    if (can_connected_) {
      can_tx_frame_ = CANLib::CanFrame();
      can_tx_frame_.id = joint.can_id;
      can_tx_frame_.dlc = 1;
      can_tx_frame_.data[0] = CMD_KILL_ALL;
      canBus_.send(can_tx_frame_);
      RCLCPP_INFO(rclcpp::get_logger("PowerModuleHardwareInterface"), "Sent Kill All Command to CAN ID 0x%X", joint.can_id);
    }
    
    joint.kill_all_sent = 1.0;
    joint.status = 1.0;
  } else if (joint.cmd_kill_all < 0.5) {
    joint.kill_all_sent = 0.0;
  }

  if (joint.cmd_kill_main > 0.5 && joint.kill_main_sent < 0.5) {
    if (can_connected_) {
      can_tx_frame_ = CANLib::CanFrame();
      can_tx_frame_.id = joint.can_id;
      can_tx_frame_.dlc = 1;
      can_tx_frame_.data[0] = CMD_KILL_MAIN;
      canBus_.send(can_tx_frame_);
      RCLCPP_INFO(rclcpp::get_logger("PowerModuleHardwareInterface"), "Sent Kill Main Command to CAN ID 0x%X", joint.can_id);
    }
    joint.kill_main_sent = 1.0;
    joint.status = 1.0;
  } else if (joint.cmd_kill_main < 0.5) {
    joint.kill_main_sent = 0.0;
  }

  if (joint.cmd_kill_jetson > 0.5 && joint.kill_jetson_sent < 0.5) {
    if (can_connected_) {
      can_tx_frame_ = CANLib::CanFrame();
      can_tx_frame_.id = joint.can_id;
      can_tx_frame_.dlc = 1;
      can_tx_frame_.data[0] = CMD_KILL_JETSON;
      canBus_.send(can_tx_frame_);
      RCLCPP_INFO(rclcpp::get_logger("PowerModuleHardwareInterface"), "Sent Kill Jetson Command to CAN ID 0x%X", joint.can_id);
    }
    
    joint.kill_jetson_sent = 1.0;
    joint.status = 1.0;
  } else if (joint.cmd_kill_jetson < 0.5) {
    joint.kill_jetson_sent = 0.0;
  }

  if (joint.status_request < 0.0 && joint.prev_status_request >= 0.0) {
    joint.status = can_connected_ ? 1.0 : 0.0;
  } else if (joint.status_request > 0.0) {
    joint.elapsed_status_request_time += period.seconds();
    if (joint.elapsed_status_request_time > (1.0 / joint.status_request)) {
      joint.elapsed_status_request_time = 0.0;
      joint.status = can_connected_ ? 1.0 : 0.0;
    }
  }
  joint.prev_status_request = joint.status_request;

  return hardware_interface::return_type::OK;
}

}  // namespace power_module_ros2_control

PLUGINLIB_EXPORT_CLASS(
  power_module_ros2_control::PowerModuleHardwareInterface,
  hardware_interface::SystemInterface)
