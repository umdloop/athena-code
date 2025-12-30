#ifndef SERVO_HARDWARE_INTERACE_HPP_
#define SERVO_HARDWARE_INTERACE_HPP_

#include <netinet/in.h>
#include <memory>
#include <string>
#include <vector>
#include <cstdint>


#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"


#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include "umdloop_can_library/SocketCanBus.hpp"
#include "umdloop_can_library/CanFrame.hpp"

namespace servo_ros2_control
{
class SERVOHardwareInterface : public hardware_interface::SystemInterface // Inheriting from System Interface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SERVOHardwareInterface)

  // Initialization, so reading parameters, initializing variables, checking if all the joint state and command interfaces are correct
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  // Exports/exposes Interfaces that are available so that the controllers
  // know what to read and write to
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Lifecycle
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces
  ) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  void onCanMessage(const CANLib::CanFrame& frame);

  // Helper Functions
  double calculate_joint_position_from_motor_position(double motor_position, int gear_ratio);
  double calculate_joint_velocity_from_motor_velocity(double motor_velocity, int gear_ratio);

  int32_t calculate_motor_position_from_desired_joint_position(double joint_position, int gear_ratio);
  int32_t calculate_motor_velocity_from_desired_joint_velocity(double joint_velocity, int gear_ratio);

private:

  int num_joints;
  int current_joint;
  int update_rate;
  double elapsed_update_time;
  std::string can_interface;

  int can_command_id;
  int can_response_id;

  // Store the state for the simulated robot
  std::vector<double> joint_state_position_;
  std::vector<double> joint_state_velocity_;
  
  // Store the command for the simulated robot
  std::vector<double> joint_command_position_;
  std::vector<double> joint_command_velocity_;

  std::vector<double> encoder_position;
  std::vector<double> motor_velocity;
  std::vector<double> motor_position;
  std::vector<double> rated_max;

  CANLib::SocketCanBus canBus;
  CANLib::CanFrame can_tx_frame_;
  CANLib::CanFrame can_rx_frame_;

  std::vector<int> joint_node_ids;
  std::vector<int> joint_gear_ratios;

  enum class integration_level_t : std::uint8_t
  {
    UNDEFINED = 0,
    POSITION = 1,
    VELOCITY = 2,
  };

  // Active control mode for each actuator
  std::vector<integration_level_t> control_level_;

  enum class servo_type_t : std::uint8_t
  {
    UNDEFINED = 0,
    STANDARD = 1,
    CONTINUOUS = 2,
  };

  // Type of servo for each actuator
  std::vector<servo_type_t> servo_type_;

  enum class joint_type_t : std::uint8_t
  {
    UNDEFINED = 0,
    REVOLUTE = 1,
    PRISMATIC = 2,
  };

  // Type of joint for each actuator
  std::vector<joint_type_t> joint_type_;

};

}  // namespace servo_hardware_interface

#endif  // SERVO_HARDWARE_INTERACE_HPP_
