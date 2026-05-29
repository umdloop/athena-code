// Copyright (c) 2025, UMDLoop
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

//
// Source of this file are templates in
// [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
//

#ifndef SCIENCE_CONTROLLERS__SCIENCE_MANUAL_HPP_
#define SCIENCE_CONTROLLERS__SCIENCE_MANUAL_HPP_

#include <memory>
#include <string>
#include <vector>

#include "science_controllers/science_manual_parameters.hpp"
#include "controller_interface/controller_interface.hpp"
#include "science_controllers/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "control_msgs/msg/joint_controller_state.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace science_controllers
{

// name constants for state interfaces
static constexpr size_t STATE_MY_ITFS = 0;

// name constants for command interfaces
static constexpr size_t CMD_MY_ITFS = 0;

// amount of joystick axes
static constexpr int joystick_axes = 6;

// amount of joystick buttons
static constexpr int joystick_buttons = 16;

// CONTROL MODE FOR DIFFERENT STAGES IN SCIENCE
enum class control_mode_type : std::uint8_t
{
  STAGE1 = 0,
  STAGE2 = 1,
  STAGE3 = 2,
  STAGE4 = 3
};

enum class control_speed_type : std::uint8_t
{
  VERY_SLOW = 0,
  SLOW = 1,
  MEDIUM = 2,
  FAST = 3
};

class ScienceManual : public controller_interface::ControllerInterface
{
public:
  SCIENCE_CONTROLLERS__VISIBILITY_PUBLIC
  ScienceManual();

  SCIENCE_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  SCIENCE_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  SCIENCE_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  SCIENCE_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  SCIENCE_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  SCIENCE_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  SCIENCE_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // TODO(anyone): replace the state and command message types
  using ControllerReferenceMsg = sensor_msgs::msg::Joy;
  using ControllerModeSrvType = std_srvs::srv::SetBool;
  using ControllerStateMsg = control_msgs::msg::JointControllerState;


protected:

  std::shared_ptr<science_manual::ParamListener> param_listener_;
  science_manual::Params params_;

  // Pumps
  int active_pump = 0; // 0 for pump a; 1 for pump b
  int pump_right_toggle = 0;
  int pump_left_toggle = 0;
  bool prev_pump_up_button_ = false;
  bool prev_pump_down_button_ = false;

  // Servos
  bool servo_scoop_f_toggle = true;
  bool prev_servo_scoop_f_button_ = false;
  bool servo_scoop_b_toggle = true;
  bool prev_servo_scoop_b_button_ = false;

  // Sampler
  bool auger_lift_toggle = true; // Current orientation starts at angle: 360 deg
  bool prev_auger_lift_button_ = false;

  // Temporary for SAR
  bool lift_toggle = false;
  bool prev_lift_button = false;

  int servo_scoop_b_counter; // TESTING

  std::string pump_right;
  std::string pump_left;
  std::string scoops_lift_f;
  std::string scoops_lift_b;
  std::vector<std::string> scoop_servos;
  std::string scoop_spinner;
  std::string sampler_lift_f;
  std::string sampler_lift_b;
  std::string conveyor_belt;
  std::string auger_spinner;
  std::string auger_lift;

  std::vector<std::string> state_joints_;
  std::vector<std::string> stepper_pump_joints_;
  std::vector<std::string> scoops_lift_joints_;
  std::vector<std::string> dc_joints_;
  std::vector<std::string> servo_joints_;
  std::vector<std::string> joints_;
  //std::string dc_auger_;

  // Command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

  rclcpp::Service<ControllerModeSrvType>::SharedPtr set_slow_control_mode_service_;
  realtime_tools::RealtimeBuffer<control_mode_type> control_mode_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> state_publisher_;

private:
  // callback for topic interface
  SCIENCE_CONTROLLERS__VISIBILITY_LOCAL
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);

  std::vector<int32_t> prev_buttons_;

  control_mode_type current_mode_{control_mode_type::STAGE1};
  void load_velocity_limits();  // called in on_configure()

  static constexpr double max_lift_velocity = 1.0;
  static constexpr double max_stepper_velocity = 1.0;
  static constexpr double scoop_talon_velocity = 1.0;
  static constexpr double auger_velocity = 1.0;
  
  double pump_right_cmd = 0.0;
  double pump_left_cmd = 0.0;
  double scoops_lift_front_vel = 0.0;
  double scoops_lift_back_vel = 0.0;
  double sampler_lift_front_vel = 0.0;
  double sampler_lift_back_vel = 0.0;
  double conveyor_belt_vel = 0.0;
  double scoop_servo_front_pos = 0.0;
  double scoop_servo_back_pos = 0.0;
  double auger_lift_pos = 0.0;

  enum CommandInterfaces
  {
    // ----- STEPPERS -----
    // --- Pump ---
    IDX_PUMP_RIGHT_VELOCITY = 0,
    IDX_PUMP_LEFT_VELOCITY = 1,

    // ----- SERVOS -----
    // --- Scoops Lift ---
    IDX_SCOOPS_LIFT_FRONT_VELOCITY  = 2,
    IDX_SCOOPS_LIFT_BACK_VELOCITY = 3,

    // --- Sampler Lift ----
    IDX_SAMPLER_LIFT_FRONT_VELOCITY = 4,
    IDX_SAMPLER_LIFT_BACK_VELOCITY = 5,

    // --- Conveyor Belt ---
    IDX_CONVEYOR_BELT_VELOCITY = 6,

    // --- Scoop Servos ---
    IDX_SCOOP_FRONT_POSITION = 7,
    IDX_SCOOP_BACK_POSITION = 8,
    
    // ----- Auger_lift (cap) -----
    IDX_AUGER_LIFT_POSITION = 9,


    // ----- DC MOTORS -----
    // --- Scoop Spinner ---
    IDX_SCOOP_SPINNER_VELOCITY = 10,

    // ----- Auger -----
    IDX_AUGER_SPINNER_VELOCITY = 11,
    
    // Total number of interfaces
    CMD_ITFS_COUNT
  };
};
  
};
// namespace science_controllers

#endif  // SCIENCE_CONTROLLERS__SCIENCE_MANUAL_HPP_
