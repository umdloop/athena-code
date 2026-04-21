#ifndef RMD_HARDWARE_INTERACE_HPP_
#define RMD_HARDWARE_INTERACE_HPP_

#include <netinet/in.h>
#include <memory>
#include <string>
#include <vector>
#include <cstdint>
#include <unordered_map>


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

  struct RMDJoint
  {
    std::string name;
    uint32_t node_write_id;
    uint32_t node_read_id;
    int gear_ratio;
    int orientation;
    uint16_t operating_velocity;
    integration_level_t control_level;

    double joint_state_position;
    double joint_state_velocity;
    double motor_temperature;
    double motor_torque_current;
    double motor_status;

    double joint_command_position;
    double joint_command_velocity;
    double motor_status_req;
    double motor_maintenance_req;
    double maintenance_cmd_id;

    double prev_status_req;
    double prev_maintenance_req;
    double prev_maintenance_cmd_id;
    double elapsed_status_request_time;
    double elapsed_maintenance_request_time;
    double motor_velocity;
    double motor_position;
    double prev_joint_command_position;
    double prev_joint_command_velocity;

    std::vector<std::string> state_interface_names;
    std::vector<std::string> command_interface_names;
    std::unordered_map<std::string, std::string> parameters;
  };

  std::vector<RMDJoint> RMDJoints_;

  // CAN Commands
  // Maintenance commands: Commands specifically sent via maintenance_request interface
  enum class MaintenanceCommands : uint8_t {
    WRITE_PID_TO_RAM_CMD = 0x31,
    WRITE_PID_TO_ROM_CMD = 0x32,
    WRITE_ACCELERATION_CMD = 0x43,
    WRITE_ENCODER_MULTI_TURN_ZERO_CMD = 0x63,
    WRITE_CURRENT_MULTI_TURN_POS_ZERO_CMD = 0x64,
    SYSTEM_RESET_CMD = 0x76,
    BRAKE_RELEASE_CMD = 0X77,
    BRAKE_LOCK_CMD = 0x78,
    MOTOR_SHUTDOWN_CMD = 0X80,
    MOTOR_STOP_CMD = 0x81,
  };

  // Control commands: Commands that actuate the motor
  enum class ControlCommands : uint8_t {
    TORQUE_CONTROL_CMD = 0xA1,
    SPEED_CONTROL_CMD = 0xA2,
    ABSOLUTE_POS_CONTROL_CMD = 0xA4,
  };

  // Status commands: Commands sent to request specific status information from the motor
  enum class StatusCommands : uint8_t {
    READ_PID_CMD = 0x30,
    MOTOR_STATUS_1_CMD = 0x9A,
    MOTOR_STATUS_2_CMD = 0X9C,
  };

  // General motor status (can be interpreted by controller)
  enum class MotorStatus : uint8_t 
  {
    UNKNOWN = 0,
    IDLE = 1,
    ACTIVE = 2,
    WARNING = 3,
    ERROR = 4,
    DISABLED = 5
  };

  // Specific motor status flags
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

  // Corresponding descriptions for specific motor status flags
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
