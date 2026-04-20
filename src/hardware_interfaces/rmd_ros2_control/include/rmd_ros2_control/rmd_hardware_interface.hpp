#ifndef RMD_HARDWARE_INTERACE_HPP_
#define RMD_HARDWARE_INTERACE_HPP_

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

namespace rmd_ros2_control
{
class RMDHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RMDHardwareInterface)

  // Initialization: Reading parameters, initializing variables, checking if all the joint state and command interfaces are correct
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  // Exports/exposes Interfaces that are available so that the controllers
  // know what to read and write to
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // -- Lifecycle Functions --
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


  // -- Helper Functions --
  void send_command(int can_id, int cmd_id);
  void on_can_message(const CANLib::CanFrame& frame);
  void logger_function();
  double calculate_joint_position_from_motor_position(double motor_position, int gear_ratio);
  double calculate_joint_velocity_from_motor_velocity(double motor_velocity, int gear_ratio);
  int32_t calculate_motor_position_from_desired_joint_position(double joint_position, int gear_ratio);
  int32_t calculate_motor_velocity_from_desired_joint_velocity(double joint_velocity, int gear_ratio);
  bool process_status(uint16_t status, const rclcpp::Logger& logger);

private:
  // Hardware Interface Parameters
  int update_rate;
  double elapsed_update_time; // Time since last hardware interface update
  double elapsed_time; // Time since first hardware interface update
  double elapsed_logger_time; // Time since last logger update
  int logger_rate; // Logger update rate
  int logger_state; // Logger on/off state
  
  // Keeps track of amount of joints
  int num_joints;

  // Store the state interfaces for the RMD
  std::vector<double> joint_state_position_;
  std::vector<double> joint_state_velocity_;
  std::vector<double> motor_temperature_;    // Motor temperature in °C
  std::vector<double> motor_torque_current_; // Motor torque current in A
  std::vector<double> motor_status_;
  
  // Store the command interfaces for the RMD
  std::vector<double> joint_command_position_;
  std::vector<double> joint_command_velocity_;
  std::vector<double> motor_status_req_;
  std::vector<double> motor_maintenance_req_;

  std::vector<double> prev_status_req_; // To track changes in status request for one-shot requests
  std::vector<double> elapsed_status_request_time_; // To track time for periodic status requests

  // Place holders for data from the CANBus, will be accessed in read()
  std::vector<double> motor_velocity;
  std::vector<double> motor_position;
  
  // Velocity at which **joint** rotates to reach position in 1 dps
  uint16_t operating_velocity;
  
  // CAN Library Setup
  CANLib::SocketCanBus canBus;
  CANLib::CanFrame can_tx_frame_;
  CANLib::CanFrame can_rx_frame_;
  std::string can_interface;
  
  // Joint specific parameters
  std::vector<uint32_t> joint_node_write_ids;
  std::vector<uint32_t> joint_node_read_ids;
  std::vector<int> joint_gear_ratios;
  std::vector<int> joint_orientation;

  // Modes for control mode
  enum class integration_level_t : std::uint8_t
  {
    UNDEFINED = 0,
    POSITION = 1,
    VELOCITY = 2,
  };

  // Active control mode for each actuator
  std::vector<integration_level_t> control_level_;

  // CAN Commands
  static constexpr uint8_t WRITE_ENCODER_MULTI_TURN_ZERO_CMD = 0x63;
  static constexpr uint8_t BRAKE_RELEASE_CMD = 0X77;
  static constexpr uint8_t BRAKE_LOCK_CMD = 0x78;
  static constexpr uint8_t MOTOR_SHUTDOWN_CMD = 0X80;
  static constexpr uint8_t MOTOR_STOP_CMD = 0x81;
  static constexpr uint8_t MOTOR_STATUS_1_CMD = 0x9A;
  static constexpr uint8_t MOTOR_STATUS_2_CMD = 0X9C;
  static constexpr uint8_t SPEED_CONTROL_CMD = 0xA2;
  static constexpr uint8_t ABSOLUTE_POS_CONTROL_CMD = 0xA4;

  enum class MotorStatus : uint8_t 
  {
    UNKNOWN = 0,
    IDLE = 1,
    ACTIVE = 2,
    WARNING = 3,
    ERROR = 4,
    DISABLED = 5
  };

  enum class RMDMotorStatus : uint16_t {
    BRAKE_LOCKED = 0x0000,
    BRAKE_RELEASED = 0x0001,
    MOTOR_STALL = 0x0002,
    LOW_PRESSURE = 0x0004,
    OVERVOLTAGE = 0x0008,
    OVERCURRENT = 0x0010,
    POWER_OVERRUN = 0x0040,
    CAL_PARAM_WRITE_ERROR = 0x0080,
    SPEEDING = 0x0100,
    OVER_TEMPERATURE = 0x1000,
    ENCODER_CALIBRATION_ERROR = 0x2000
  };

  struct StatusEntry {
    RMDMotorStatus flag;
    const char* name;
  };

  inline static const std::vector<StatusEntry> status_table = {
    {RMDMotorStatus::BRAKE_RELEASED, "Brake Released"},
    {RMDMotorStatus::BRAKE_LOCKED, "Brake Locked"},
    {RMDMotorStatus::MOTOR_STALL, "Motor Stall"},
    {RMDMotorStatus::LOW_PRESSURE, "Low Pressure"},
    {RMDMotorStatus::OVERVOLTAGE, "Overvoltage"},
    {RMDMotorStatus::OVERCURRENT, "Overcurrent"},
    {RMDMotorStatus::POWER_OVERRUN, "Power Overrun"},
    {RMDMotorStatus::CAL_PARAM_WRITE_ERROR, "Calibration Param Write Error"},
    {RMDMotorStatus::SPEEDING, "Overspeed"},
    {RMDMotorStatus::OVER_TEMPERATURE, "Over Temperature"},
    {RMDMotorStatus::ENCODER_CALIBRATION_ERROR, "Encoder Calibration Error"}
  };
};
}  // namespace rmd_hardware_interface

#endif  // RMD_HARDWARE_INTERACE_HPP_
