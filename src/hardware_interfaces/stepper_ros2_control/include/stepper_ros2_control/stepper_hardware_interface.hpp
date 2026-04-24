#ifndef STEPPER_HARDWARE_INTERFACE_HPP_
#define STEPPER_HARDWARE_INTERFACE_HPP_

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "msgs/msg/cana.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace stepper_ros2_control
{

class STEPPERHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(STEPPERHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

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
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  enum class integration_level_t : std::uint8_t
  {
    UNDEFINED = 0,
    POSITION = 1,
    VELOCITY = 2,
  };

  struct StepperJoint
  {
    std::string name;
    uint16_t node_id;
    int gear_ratio;
    int orientation;
    double initial_position;
    integration_level_t control_level;

    double joint_state_position;
    double joint_state_velocity;
    double motor_temperature;
    double motor_torque_current;
    double motor_status;
    double acceleration;

    double joint_command_position;
    double joint_command_velocity;
    double motor_status_req;
    double motor_maintenance_req;
    double maintenance_frame_high;
    double maintenance_frame_low;
    double maintenance_frame;
    double maintenance_data_count;
    std::vector<uint8_t> decoded_maintenance_frame;

    double prev_status_req;
    double prev_maintenance_req;
    double elapsed_status_request_time;
    double elapsed_maintenance_request_time;
    double motor_position;
    double motor_velocity;
    double encoder_position;
    double prev_joint_command_position;
    double prev_joint_command_velocity;

    std::vector<std::string> state_interface_names;
    std::vector<std::string> command_interface_names;
    std::unordered_map<std::string, std::string> parameters;
  };

  struct DecodedCommand
  {
    uint8_t command_id;
    std::vector<uint8_t> u8_data;
    std::vector<int16_t> i16_data;
    std::vector<int32_t> i32_data;
  };

  double calculate_joint_position_from_motor_position(double motor_position, int gear_ratio);
  double calculate_joint_velocity_from_motor_velocity(double motor_velocity, int gear_ratio);
  int32_t calculate_motor_position_from_desired_joint_position(double joint_position, int gear_ratio);
  int32_t calculate_motor_velocity_from_desired_joint_velocity(double joint_velocity, int gear_ratio);

  void format_control_command(msgs::msg::CANA & frame, StepperJoint & joint);
  bool format_status_command(msgs::msg::CANA & frame, uint8_t command_id, uint16_t node_id);
  bool format_maintenance_command(
    msgs::msg::CANA & frame, uint16_t node_id, const DecodedCommand & decoded_cmd);

private:
  int num_joints;
  uint16_t current_iteration;

  rclcpp::Publisher<msgs::msg::CANA>::SharedPtr science_can_publisher_;
  rclcpp::Subscription<msgs::msg::CANA>::SharedPtr science_can_subscriber_;
  rclcpp::Node::SharedPtr node_;
  msgs::msg::CANA received_joint_data_;

  std::vector<StepperJoint> STEPPERJoints_;

  enum class MaintenanceCommands : uint8_t
  {
    BRAKE_RELEASE_CMD = 0x77,
    BRAKE_LOCK_CMD = 0x78,
    MOTOR_SHUTDOWN_CMD = 0x80,
    MOTOR_STOP_CMD = 0x81,
  };

  enum class ControlCommands : uint8_t
  {
    SPEED_CONTROL_CMD = 0xA2,
    ABSOLUTE_POS_CONTROL_CMD = 0xA4,
  };

  enum class StatusCommands : uint8_t
  {
    READ_MULTI_TURN_ANGLE_CMD = 0x92,
    MOTOR_STATUS_2_CMD = 0x9C,
  };

  static constexpr std::array<StatusCommands, 2> kStatusCommands = {
    StatusCommands::READ_MULTI_TURN_ANGLE_CMD,
    StatusCommands::MOTOR_STATUS_2_CMD,
  };

  inline DecodedCommand unpack_command_full(int32_t counts_in, int64_t payload_in)
  {
    const uint32_t counts = static_cast<uint32_t>(counts_in);
    const uint64_t payload = static_cast<uint64_t>(payload_in);

    const uint8_t u8_count = static_cast<uint8_t>((counts >> 16) & 0xFF);
    const uint8_t i16_count = static_cast<uint8_t>((counts >> 8) & 0xFF);
    const uint8_t i32_count = static_cast<uint8_t>(counts & 0xFF);

    int bit_offset = 64;
    auto pop_bits = [&](int bits) -> uint64_t {
      bit_offset -= bits;
      const uint64_t mask = (bits == 64) ? std::numeric_limits<uint64_t>::max() : ((1ULL << bits) - 1ULL);
      return (payload >> bit_offset) & mask;
    };

    DecodedCommand result{};
    result.command_id = static_cast<uint8_t>(pop_bits(8));
    for (uint8_t i = 0; i < u8_count; ++i) {
      result.u8_data.push_back(static_cast<uint8_t>(pop_bits(8)));
    }
    for (uint8_t i = 0; i < i16_count; ++i) {
      result.i16_data.push_back(static_cast<int16_t>(static_cast<uint16_t>(pop_bits(16))));
    }
    for (uint8_t i = 0; i < i32_count; ++i) {
      result.i32_data.push_back(static_cast<int32_t>(static_cast<uint32_t>(pop_bits(32))));
    }
    return result;
  }

  template<typename JointT>
  inline void pack_decoded_maintenance_frame(
    JointT & joint, const DecodedCommand & decoded_maintenance_cmd)
  {
    joint.decoded_maintenance_frame.clear();
    joint.decoded_maintenance_frame.push_back(decoded_maintenance_cmd.command_id);
    joint.decoded_maintenance_frame.insert(
      joint.decoded_maintenance_frame.end(),
      decoded_maintenance_cmd.u8_data.begin(),
      decoded_maintenance_cmd.u8_data.end());
  }
};

}  // namespace stepper_ros2_control

#endif  // STEPPER_HARDWARE_INTERFACE_HPP_
