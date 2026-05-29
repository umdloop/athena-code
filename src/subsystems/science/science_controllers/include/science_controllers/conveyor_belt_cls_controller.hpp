// Copyright (c) 2025, UMDLoop
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#ifndef SCIENCE_CONTROLLERS__CONVEYOR_BELT_CLS_CONTROLLER_HPP_
#define SCIENCE_CONTROLLERS__CONVEYOR_BELT_CLS_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "control_msgs/msg/joint_controller_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "science_controllers/conveyor_belt_cls_controller_parameters.hpp"
#include "science_controllers/visibility_control.h"
#include "std_msgs/msg/float64.hpp"

namespace science_controllers
{

class ConveyorBeltCLSController : public controller_interface::ControllerInterface
{
public:
  SCIENCE_CONTROLLERS__VISIBILITY_PUBLIC
  ConveyorBeltCLSController();

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
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  using ControllerReferenceMsg = std_msgs::msg::Float64;
  using ControllerStateMsg = control_msgs::msg::JointControllerState;
  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);
  void reset_motion_plan(double target_position, double current_position, const rclcpp::Time & time);

  std::shared_ptr<conveyor_belt_cls_controller::ParamListener> param_listener_;
  conveyor_belt_cls_controller::Params params_;

  std::string conveyor_belt_joint_;
  std::string rotary_encoder_joint_;
  std::string magnetic_limit_switch_name_;
  std::string reference_topic_;

  double operating_velocity_ = 0.0;
  double encoder_state_request_rate_ = 0.0;
  double position_tolerance_ = 0.0;
  double kP_ = 0.0;
  double kI_ = 0.0;
  double kD_ = 0.0;

  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr state_publisher_;
  std::unique_ptr<ControllerStatePublisher> realtime_state_publisher_;

  double target_position_ = 0.0;
  double last_reference_position_ = 0.0;
  bool has_reference_ = false;

  double open_loop_velocity_command_ = 0.0;
  double open_loop_duration_sec_ = 0.0;
  rclcpp::Time open_loop_start_time_{0, 0, RCL_ROS_TIME};

  double integral_error_ = 0.0;
  double previous_error_ = 0.0;
  double magnetic_limit_switch_status_ = 0.0;

  enum CommandInterfaces
  {
    CMD_CONVEYOR_BELT_VELOCITY = 0,
    CMD_ROTARY_ENCODER_STATE_REQUEST,
    CMD_ITFS_COUNT
  };

  enum StateInterfaces
  {
    STATE_ROTARY_ENCODER_POSITION = 0,
    STATE_MAGNETIC_LIMIT_SWITCH_STATUS,
    STATE_ITFS_COUNT
  };
};

}  // namespace science_controllers

#endif  // SCIENCE_CONTROLLERS__CONVEYOR_BELT_CLS_CONTROLLER_HPP_
