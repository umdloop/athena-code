#include "killswitch_ros2_control/killswitch_hardware_interface.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace killswitch_ros2_control
{

hardware_interface::CallbackReturn KillswitchHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  can_interface_ = info_.hardware_parameters.count("can_interface") ?
    info_.hardware_parameters.at("can_interface") : "can0";
  const uint32_t can_id = info_.hardware_parameters.count("can_id") ?
    static_cast<uint32_t>(std::stoul(info_.hardware_parameters.at("can_id"), nullptr, 0)) : 0x100;

  KILLSWITCHJoints_.clear();
  KILLSWITCHJoints_.push_back(KillswitchJoint{
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
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KillswitchHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!canBus_.open(
      can_interface_,
      std::bind(&KillswitchHardwareInterface::onCanMessage, this, std::placeholders::_1)))
  {
    can_connected_ = false;
  } else {
    can_connected_ = true;
  }
  auto & joint = KILLSWITCHJoints_.front();
  joint.is_connected = can_connected_ ? 1.0 : 0.0;
  joint.status = joint.is_connected;
  return hardware_interface::CallbackReturn::SUCCESS;
}

void KillswitchHardwareInterface::onCanMessage(const CANLib::CanFrame &)
{
}

std::vector<hardware_interface::StateInterface> KillswitchHardwareInterface::export_state_interfaces()
{
  auto & joint = KILLSWITCHJoints_.front();
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(joint.name, "kill_all_sent", &joint.kill_all_sent);
  state_interfaces.emplace_back(joint.name, "kill_main_sent", &joint.kill_main_sent);
  state_interfaces.emplace_back(joint.name, "kill_jetson_sent", &joint.kill_jetson_sent);
  state_interfaces.emplace_back(joint.name, "is_connected", &joint.is_connected);
  state_interfaces.emplace_back(joint.name, "status", &joint.status);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> KillswitchHardwareInterface::export_command_interfaces()
{
  auto & joint = KILLSWITCHJoints_.front();
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(joint.name, "kill_all", &joint.cmd_kill_all);
  command_interfaces.emplace_back(joint.name, "kill_main", &joint.cmd_kill_main);
  command_interfaces.emplace_back(joint.name, "kill_jetson", &joint.cmd_kill_jetson);
  command_interfaces.emplace_back(joint.name, "status_request", &joint.status_request);
  return command_interfaces;
}

hardware_interface::CallbackReturn KillswitchHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  auto & joint = KILLSWITCHJoints_.front();
  joint.cmd_kill_all = 0.0;
  joint.cmd_kill_main = 0.0;
  joint.cmd_kill_jetson = 0.0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KillswitchHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KillswitchHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  if (can_connected_) {
    canBus_.close();
  }
  can_connected_ = false;
  auto & joint = KILLSWITCHJoints_.front();
  joint.is_connected = 0.0;
  joint.status = 0.0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KillswitchHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

hardware_interface::return_type KillswitchHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KillswitchHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  auto & joint = KILLSWITCHJoints_.front();

  if (joint.cmd_kill_all > 0.5 && joint.kill_all_sent < 0.5) {
    if (can_connected_) {
      can_tx_frame_ = CANLib::CanFrame();
      can_tx_frame_.id = joint.can_id;
      can_tx_frame_.dlc = 1;
      can_tx_frame_.data[0] = CMD_KILL_ALL;
      canBus_.send(can_tx_frame_);
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

}  // namespace killswitch_ros2_control

PLUGINLIB_EXPORT_CLASS(
  killswitch_ros2_control::KillswitchHardwareInterface,
  hardware_interface::SystemInterface)
