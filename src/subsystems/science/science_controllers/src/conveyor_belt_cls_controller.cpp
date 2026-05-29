#include "science_controllers/conveyor_belt_cls_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "pluginlib/class_list_macros.hpp"

namespace science_controllers
{

ConveyorBeltCLSController::ConveyorBeltCLSController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn ConveyorBeltCLSController::on_init()
{
  try {
    param_listener_ = std::make_shared<conveyor_belt_cls_controller::ParamListener>(get_node());
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during controller init: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ConveyorBeltCLSController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.push_back(conveyor_belt_joint_ + "/velocity");
  cfg.names.push_back(rotary_encoder_joint_ + "/state_request");
  return cfg;
}

controller_interface::InterfaceConfiguration
ConveyorBeltCLSController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.push_back(rotary_encoder_joint_ + "/position");
  cfg.names.push_back(magnetic_limit_switch_name_ + "/status");
  return cfg;
}

controller_interface::CallbackReturn ConveyorBeltCLSController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();
  conveyor_belt_joint_ = params_.conveyor_belt_joint;
  rotary_encoder_joint_ = params_.rotary_encoder_joint;
  magnetic_limit_switch_name_ = params_.magnetic_limit_switch_name;
  reference_topic_ = params_.reference_topic;
  operating_velocity_ = params_.operating_velocity;
  encoder_state_request_rate_ = params_.encoder_state_request_rate;
  position_tolerance_ = params_.position_tolerance;
  kP_ = params_.kP;
  kI_ = params_.kI;
  kD_ = params_.kD;

  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    reference_topic_,
    subscribers_qos,
    std::bind(&ConveyorBeltCLSController::reference_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> initial_msg = std::make_shared<ControllerReferenceMsg>();
  initial_msg->data = 0.0;
  input_ref_.writeFromNonRT(initial_msg);

  state_publisher_ = get_node()->create_publisher<ControllerStateMsg>(
    "~/state", rclcpp::QoS(rclcpp::KeepLast(1)));
  realtime_state_publisher_ = std::make_unique<ControllerStatePublisher>(state_publisher_);

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured ConveyorBeltCLSController for joint '%s' with encoder '%s'",
    conveyor_belt_joint_.c_str(),
    rotary_encoder_joint_.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ConveyorBeltCLSController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (command_interfaces_.size() != CMD_ITFS_COUNT) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu command interfaces, got %zu",
      static_cast<size_t>(CMD_ITFS_COUNT),
      command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (state_interfaces_.size() != STATE_ITFS_COUNT) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu state interfaces, got %zu",
      static_cast<size_t>(STATE_ITFS_COUNT),
      state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  const double current_position = state_interfaces_[STATE_ROTARY_ENCODER_POSITION].get_value();
  target_position_ = current_position;
  last_reference_position_ = current_position;
  has_reference_ = false;
  open_loop_velocity_command_ = 0.0;
  open_loop_duration_sec_ = 0.0;
  integral_error_ = 0.0;
  previous_error_ = 0.0;
  magnetic_limit_switch_status_ = state_interfaces_[STATE_MAGNETIC_LIMIT_SWITCH_STATUS].get_value();

  command_interfaces_[CMD_CONVEYOR_BELT_VELOCITY].set_value(0.0);
  command_interfaces_[CMD_ROTARY_ENCODER_STATE_REQUEST].set_value(encoder_state_request_rate_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ConveyorBeltCLSController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(std::numeric_limits<double>::quiet_NaN());
  }

  has_reference_ = false;
  open_loop_velocity_command_ = 0.0;
  open_loop_duration_sec_ = 0.0;
  integral_error_ = 0.0;
  previous_error_ = 0.0;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ConveyorBeltCLSController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  const auto current_ref = input_ref_.readFromRT();
  if (current_ref && *current_ref) {
    const double requested_position = (*current_ref)->data;
    if (!has_reference_ || requested_position != last_reference_position_) {
      const double current_position = state_interfaces_[STATE_ROTARY_ENCODER_POSITION].get_value();
      target_position_ = requested_position;
      last_reference_position_ = requested_position;
      has_reference_ = true;
      reset_motion_plan(target_position_, current_position, time);
    }
  }

  const double current_position = state_interfaces_[STATE_ROTARY_ENCODER_POSITION].get_value();
  magnetic_limit_switch_status_ = state_interfaces_[STATE_MAGNETIC_LIMIT_SWITCH_STATUS].get_value();

  const double error = target_position_ - current_position;
  const double dt = period.seconds();

  double open_loop_command = 0.0;
  if (open_loop_duration_sec_ > 0.0 &&
      (time - open_loop_start_time_).seconds() < open_loop_duration_sec_)
  {
    open_loop_command = open_loop_velocity_command_;
  }

  if (dt > 0.0) {
    integral_error_ += error * dt;
  }
  const double derivative = dt > 0.0 ? (error - previous_error_) / dt : 0.0;
  previous_error_ = error;

  double pid_command = kP_ * error + kI_ * integral_error_ + kD_ * derivative;
  double velocity_command = open_loop_command + pid_command;

  if (std::abs(error) <= position_tolerance_) {
    velocity_command = 0.0;
    integral_error_ = 0.0;
  }

  const double max_velocity = std::abs(operating_velocity_);
  velocity_command = std::clamp(velocity_command, -max_velocity, max_velocity);

  command_interfaces_[CMD_CONVEYOR_BELT_VELOCITY].set_value(velocity_command);
  command_interfaces_[CMD_ROTARY_ENCODER_STATE_REQUEST].set_value(encoder_state_request_rate_);

  if (realtime_state_publisher_ && realtime_state_publisher_->trylock()) {
    realtime_state_publisher_->msg_.header.stamp = time;
    realtime_state_publisher_->msg_.header.frame_id = conveyor_belt_joint_;
    realtime_state_publisher_->msg_.set_point = target_position_;
    realtime_state_publisher_->msg_.process_value = current_position;
    realtime_state_publisher_->msg_.command = velocity_command;
    realtime_state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

void ConveyorBeltCLSController::reference_callback(
  const std::shared_ptr<ControllerReferenceMsg> msg)
{
  input_ref_.writeFromNonRT(msg);
}

void ConveyorBeltCLSController::reset_motion_plan(
  double target_position,
  double current_position,
  const rclcpp::Time & time)
{
  const double error = target_position - current_position;
  const double direction = error >= 0.0 ? 1.0 : -1.0;
  open_loop_velocity_command_ = direction * std::abs(operating_velocity_);
  open_loop_duration_sec_ =
    std::abs(error) / std::max(std::abs(operating_velocity_), std::numeric_limits<double>::epsilon());
  open_loop_start_time_ = time;
  integral_error_ = 0.0;
  previous_error_ = error;
}

}  // namespace science_controllers

PLUGINLIB_EXPORT_CLASS(
  science_controllers::ConveyorBeltCLSController,
  controller_interface::ControllerInterface)
