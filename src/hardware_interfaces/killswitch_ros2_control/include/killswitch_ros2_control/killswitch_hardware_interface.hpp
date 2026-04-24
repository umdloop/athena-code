#ifndef KILLSWITCH_ROS2_CONTROL__KILLSWITCH_HARDWARE_INTERFACE_HPP_
#define KILLSWITCH_ROS2_CONTROL__KILLSWITCH_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "umdloop_can_library/CanFrame.hpp"
#include "umdloop_can_library/SocketCanBus.hpp"

namespace killswitch_ros2_control
{

class KillswitchHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KillswitchHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct KillswitchJoint
  {
    std::string name;
    uint32_t can_id;
    double kill_all_sent;
    double kill_main_sent;
    double kill_jetson_sent;
    double is_connected;
    double status;
    double cmd_kill_all;
    double cmd_kill_main;
    double cmd_kill_jetson;
    double status_request;
    double prev_status_request;
    double elapsed_status_request_time;
  };

  void onCanMessage(const CANLib::CanFrame & frame);

  std::string can_interface_;
  CANLib::SocketCanBus canBus_;
  CANLib::CanFrame can_tx_frame_;
  bool can_connected_;
  std::vector<KillswitchJoint> KILLSWITCHJoints_;

  static constexpr uint8_t CMD_KILL_ALL = 0x01;
  static constexpr uint8_t CMD_KILL_MAIN = 0x03;
  static constexpr uint8_t CMD_KILL_JETSON = 0x05;
};

}  // namespace killswitch_ros2_control

#endif  // KILLSWITCH_ROS2_CONTROL__KILLSWITCH_HARDWARE_INTERFACE_HPP_
