// Copyright (c) 2025, UMDLoop
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "science_controllers/science_manual.hpp"
#include "science_controllers/science_manual_parameters.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>

#include "controller_interface/helpers.hpp"

#define DEBUG_MODE 0

namespace
{

using ControllerReferenceMsg = science_controllers::ScienceManual::ControllerReferenceMsg;

// Utility: reset joystick msg for RT buffer
void reset_controller_reference_msg(
  std::shared_ptr<ControllerReferenceMsg> & msg, const int & axes_count, const int & button_count)
{
  msg->axes.resize(axes_count, 0.0);
  msg->buttons.resize(button_count, 0.0);
}
}  // namespace

namespace science_controllers
{

ScienceManual::ScienceManual() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn ScienceManual::on_init()
{
  control_mode_.initRT(control_mode_type::STAGE1);

  // servo_scoop_f_toggle = false;
  servo_scoop_b_counter = 0; // TESTING

  try {
    param_listener_ = std::make_shared<science_manual::ParamListener>(get_node());
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during controller init: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn ScienceManual::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  // Set default STAGE 1
  current_mode_ = control_mode_type::STAGE1;

  // Fill each joint variable
  pump_right = params_.pump_right;
  pump_left = params_.pump_left;
  scoops_lift_f = params_.scoops_lift_f;
  scoops_lift_b = params_.scoops_lift_b;
  sampler_lift_f = params_.sampler_lift_f;
  sampler_lift_b = params_.sampler_lift_b;
  conveyor_belt = params_.conveyor_belt;
  scoop_servos = params_.scoop_servos;
  scoop_spinner = params_.scoop_spinner;
  auger_spinner = params_.auger_spinner;
  auger_lift = params_.auger_lift;

  // Fill composite joint groups
  stepper_pump_joints_.clear();
  stepper_pump_joints_ = {
    params_.pump_right,
    params_.pump_left
  };

  dc_joints_.clear();
  dc_joints_ = {
    params_.scoop_spinner,
    params_.auger_spinner
  }; 

  servo_joints_ = params_.scoop_servos;
  servo_joints_.push_back(params_.sampler_lift_f);
  servo_joints_.push_back(params_.sampler_lift_b);
  servo_joints_.push_back(params_.auger_lift);

  scoops_lift_joints_.clear();
  scoops_lift_joints_.push_back(params_.scoops_lift_f);
  scoops_lift_joints_.push_back(params_.scoops_lift_b);

  // Populate joints vector
  joints_.clear();

  joints_.push_back(pump_right);
  joints_.push_back(pump_left);
  joints_.push_back(scoops_lift_f);
  joints_.push_back(scoops_lift_b);

  joints_.insert(joints_.end(), scoop_servos.begin(), scoop_servos.end());

  joints_.push_back(scoop_spinner);
  joints_.push_back(sampler_lift_f);
  joints_.push_back(sampler_lift_b);
  joints_.push_back(conveyor_belt);
  joints_.push_back(auger_spinner);
  joints_.push_back(auger_lift);

  if (!params_.state_joints.empty()) {
    state_joints_ = params_.state_joints;
  } else {
    state_joints_ = joints_;
  }

  if (joints_.size() != state_joints_.size()) {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of 'joints' (%zu) and 'state_joints' (%zu) must be equal!",
      joints_.size(), state_joints_.size());
    return CallbackReturn::FAILURE;
  }

  prev_buttons_.assign(joystick_buttons, 0);

  // QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference subscriber
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "science_manual", subscribers_qos,
    std::bind(&ScienceManual::reference_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg, joystick_axes, joystick_buttons);
  input_ref_.writeFromNonRT(msg);

  // State publisher
  s_publisher_ = get_node()->create_publisher<ControllerStateMsg>(
    "science_manual/state", rclcpp::QoS(rclcpp::KeepLast(1)));
  state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);

  if (state_publisher_ && state_publisher_->trylock()) {
    state_publisher_->msg_.header.frame_id =
      (!joints_.empty() ? joints_[0] : std::string("base"));
    state_publisher_->msg_.set_point = 0.0;
    state_publisher_->msg_.process_value = 0.0;
    state_publisher_->msg_.command = 0.0;
    state_publisher_->unlockAndPublish();
  }

  RCLCPP_INFO(get_node()->get_logger(), "Configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

void ScienceManual::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  if (prev_buttons_.empty()) {
    prev_buttons_.resize(msg->buttons.size(), 0);
  }
  input_ref_.writeFromNonRT(msg);
}

controller_interface::InterfaceConfiguration ScienceManual::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Pumps
  for (const auto & joint : stepper_pump_joints_)
  {
    cfg.names.push_back(joint + "/velocity");
  }

  // Scoops Lift
  for (const auto & joint : scoops_lift_joints_) {
    cfg.names.push_back(joint + "/velocity");
  }

  // Sampler Lift
  cfg.names.push_back(sampler_lift_f + "/velocity");
  cfg.names.push_back(sampler_lift_b + "/velocity");

  // Conveyor Belt
  cfg.names.push_back(conveyor_belt + "/velocity");

  // Scoop servos
  for (const auto & joint : scoop_servos)
  {
    cfg.names.push_back(joint + "/position");
  }

  // Auger Lift
  cfg.names.push_back(auger_lift + "/position");

  // Scoop spinner
  cfg.names.push_back(scoop_spinner + "/velocity");

  // Sampler
  cfg.names.push_back(auger_spinner + "/velocity");

  return cfg; 
}

controller_interface::InterfaceConfiguration ScienceManual::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : state_joints_)
  {
    cfg.names.push_back(joint + "/position");
    cfg.names.push_back(joint + "/velocity");
  }

  return cfg;
}

controller_interface::CallbackReturn ScienceManual::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reset_controller_reference_msg(*(input_ref_.readFromRT)(), joystick_axes, joystick_buttons);
  if (command_interfaces_.size() != CMD_ITFS_COUNT) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "ScienceManual: expected %d command interfaces, got %zu",
      CMD_ITFS_COUNT, command_interfaces_.size());

    RCLCPP_ERROR(
    get_node()->get_logger(),
    "ScienceManual received command interfaces:");

  for (const auto & interface : command_interfaces_)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "  %s/%s",
      interface.get_prefix_name().c_str(),
      interface.get_interface_name().c_str());
  }
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}
  
controller_interface::CallbackReturn ScienceManual::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  // stepper_pump_joints_.clear();
  // dc_joints_.clear();
  // servo_joints_.clear();
  // scoops_lift_joints_.clear();
  // state_joints_.clear();

  RCLCPP_INFO(get_node()->get_logger(), "Manual controller deactivated and released interfaces");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ScienceManual::update(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  auto current_ref = input_ref_.readFromRT();

  if (!(*current_ref)) {
    return controller_interface::return_type::OK;
  }

  auto msg = *current_ref;  // shared_ptr<sensor_msgs::msg::Joy>
  int stage_idx = static_cast<int>(current_mode_);  // corresponds to STAGE1..STAGE4

  // -- Pumps --
  // Stepper motors command (velocity-like value, but we feed as position target here)
  if(msg->buttons[12]){ // Left arrow
    active_pump = 0;
  }
  else if (msg->buttons[11]){ // Right arrow
    active_pump = 1;
  }

  if(active_pump == 0){ // Pump A
    if (msg->buttons[13] && !prev_pump_up_button_) { // Up arrow 
      pump_right_toggle++;
    }
    if (msg->buttons[14] && !prev_pump_down_button_) { // Down Arrow
      pump_right_toggle--;
    }
    pump_right_toggle = std::clamp(pump_right_toggle, -1, 1);
  }
  else if (active_pump == 1){ // Pump B
        if (msg->buttons[13] && !prev_pump_up_button_) { // Up arrow 
      pump_left_toggle++;
    }
    if (msg->buttons[14] && !prev_pump_down_button_) { // Down Arrow
      pump_left_toggle--;
    }
    pump_left_toggle = std::clamp(pump_left_toggle, -1, 1);
  }

  prev_pump_up_button_ = msg->buttons[13];
  prev_pump_down_button_ = msg->buttons[14];

  pump_right_cmd = pump_right_toggle * params_.velocity_limits_pumps[stage_idx];
  pump_left_cmd = pump_left_toggle * params_.velocity_limits_pumps[stage_idx];
  

  // -- Scoops Lift --
  // Hardcoded values for top and bottom lift position
  scoops_lift_front_vel  = msg->axes[1] * params_.velocity_limits_scoops_lift[stage_idx];
  scoops_lift_back_vel = -msg->axes[1] * params_.velocity_limits_scoops_lift[stage_idx];

  // Sampler Lift
  double axis_sampler_lift = (msg->axes.size() > 3) ? msg->axes[3] : 0.0;
  sampler_lift_front_vel = axis_sampler_lift * params_.velocity_limits_sampler_lift[stage_idx];
  sampler_lift_back_vel = axis_sampler_lift * params_.velocity_limits_sampler_lift[stage_idx];

  // Scoop Spinner (Right Trigger for Vel, Right Bumper for direction)
  double scoop_axis = (msg->axes.size() > 5) ? msg->axes[5] : 0.0;
  bool scoop_reverse = (msg->buttons.size() > 5 && msg->buttons[5]);
  double scoop_spinner_cmd = scoop_axis * params_.velocity_limits_scoop_spinner[stage_idx] * (scoop_reverse ? -1.0 : 1.0);

  // Conveyor Belt L/R Right joystick
  conveyor_belt_vel = params_.velocity_limits_conveyor_belt[stage_idx] * msg->axes[2];

  // Scoop Servo
  bool servo_scoop_f_button = (msg->buttons.size() > 1 && msg->buttons[3]);
  if (servo_scoop_f_button && !prev_servo_scoop_f_button_) {
    servo_scoop_f_toggle = !servo_scoop_f_toggle;
  }
  prev_servo_scoop_f_button_ = servo_scoop_f_button;
  scoop_servo_front_pos = servo_scoop_f_toggle * params_.position_range_scoop_servo_f[1];

  bool servo_scoop_b_button = (msg->buttons.size() > 0 && msg->buttons[0]);
  if (servo_scoop_b_button && !prev_servo_scoop_b_button_) {
    servo_scoop_b_toggle = !servo_scoop_b_toggle;
  }
  prev_servo_scoop_b_button_ = servo_scoop_b_button;
  scoop_servo_back_pos = servo_scoop_b_toggle * params_.position_range_scoop_servo_b[1];

  // Auger (Left Trigger for Vel, Left Bumper for direction)
  double auger_axis = (msg->axes.size() > 4) ? msg->axes[4] : 0.0;
  bool auger_reverse = (msg->buttons.size() > 4 && msg->buttons[4]);
  double auger_spinner_cmd = auger_axis * params_.velocity_limits_auger[stage_idx] * (auger_reverse ? -1.0 : 1.0);

  // Auger_lift
  double auger_lift_axis = (msg->axes.size() > 0) ? msg->axes[0] : 0.0;

  // Use absolute value so direction doesn't matter
  double axis_mag = std::abs(auger_lift_axis);

  // Clamp just in case
  axis_mag = std::clamp(axis_mag, 0.0, 1.0);

  double min_pos = params_.position_range_auger_lift[0];
  double max_pos = params_.position_range_auger_lift[1];

  // Default = max_pos
  // More joystick deflection → move toward min_pos
  auger_lift_pos = max_pos - axis_mag * (max_pos - min_pos);

  // Clamp Positions
  scoop_servo_front_pos = std::clamp(scoop_servo_front_pos, params_.position_range_scoop_servo_f[0], params_.position_range_scoop_servo_f[1]);
  scoop_servo_back_pos = std::clamp(scoop_servo_back_pos, params_.position_range_scoop_servo_b[0], params_.position_range_scoop_servo_b[1]);
  auger_lift_pos = std::clamp(auger_lift_pos, params_.position_range_auger_lift[0], params_.position_range_auger_lift[1]);
  
  // SET VALUES
  // Stepper motors (position)
  command_interfaces_[IDX_PUMP_RIGHT_VELOCITY].set_value(pump_right_cmd * (M_PI / 180.0));
  command_interfaces_[IDX_PUMP_LEFT_VELOCITY].set_value(pump_left_cmd * (M_PI / 180.0));

  // -- Servos --
  // Scoops Lift servos
  command_interfaces_[IDX_SCOOPS_LIFT_FRONT_VELOCITY].set_value(scoops_lift_front_vel * (M_PI / 180.0));
  command_interfaces_[IDX_SCOOPS_LIFT_BACK_VELOCITY].set_value(scoops_lift_back_vel * (M_PI / 180.0));
  
  // Sampler Lift servos
  command_interfaces_[IDX_SAMPLER_LIFT_FRONT_VELOCITY].set_value(sampler_lift_front_vel * (M_PI / 180.0));
  command_interfaces_[IDX_SAMPLER_LIFT_BACK_VELOCITY].set_value(sampler_lift_back_vel * (M_PI / 180.0));
  
  // Conveyor Belt
  command_interfaces_[IDX_CONVEYOR_BELT_VELOCITY].set_value(conveyor_belt_vel * (M_PI / 180.0));

  // Scoop servos
  command_interfaces_[IDX_SCOOP_FRONT_POSITION].set_value(scoop_servo_front_pos * (M_PI / 180.0));
  command_interfaces_[IDX_SCOOP_BACK_POSITION].set_value(scoop_servo_back_pos * (M_PI / 180.0));
  
  // Auger Lift
  command_interfaces_[IDX_AUGER_LIFT_POSITION].set_value(auger_lift_pos * (M_PI / 180.0));
  
  // -- DC --
  // Scoops Spinner
  command_interfaces_[IDX_SCOOP_SPINNER_VELOCITY].set_value(scoop_spinner_cmd);
  
  // Auger Spinner
  command_interfaces_[IDX_AUGER_SPINNER_VELOCITY].set_value(auger_spinner_cmd);


  // Logger
  std::ostringstream oss;
  const char * active_pump_name = (active_pump == 0) ? "Pump A" : "Pump B";
  oss << "\033[2J\033[H\n"
    << "========= Science Controller =========\n\n"
    << "  active pump: " << active_pump_name << " (" << active_pump << ")\n\n"
    << std::fixed << std::setprecision(3)
    << "  "
    << std::left << std::setw(4) << "idx"
    << std::setw(18) << "actuator"
    << std::setw(30) << "xbox control"
    << std::setw(24) << "input value"
    << "command\n"
    << "  "
    << std::setw(4) << "---"
    << std::setw(18) << "-----------------"
    << std::setw(30) << "-----------------------------"
    << std::setw(24) << "-----------------------"
    << "----------------\n"
    << "  "
    << std::setw(4) << "[0]"
    << std::setw(18) << "Right Pump "
    << std::setw(30) << "Right D-pad, Up/Down D-pad"
    << std::setw(24) << ("R/U/D=" + std::to_string((msg->buttons.size() > 11) ? msg->buttons[11] : 0) + "/" +
                         std::to_string((msg->buttons.size() > 13) ? msg->buttons[13] : 0) + "/" +
                         std::to_string((msg->buttons.size() > 14) ? msg->buttons[14] : 0) +
                         " t=" + std::to_string(pump_right_toggle))
    << pump_right_cmd << " deg/s\n"
    << "  "
    << std::setw(4) << "[1]"
    << std::setw(18) << "Left Pump"
    << std::setw(30) << "Left D-pad, Up/Down D-pad"
    << std::setw(24) << ("L/U/D=" + std::to_string((msg->buttons.size() > 12) ? msg->buttons[12] : 0) + "/" +
                         std::to_string((msg->buttons.size() > 13) ? msg->buttons[13] : 0) + "/" +
                         std::to_string((msg->buttons.size() > 14) ? msg->buttons[14] : 0) +
                         " t=" + std::to_string(pump_left_toggle))
    << pump_left_cmd << " deg/s\n"
    << "  "
    << std::setw(4) << "[2]"
    << std::setw(18) << "Scoops Lift Front"
    << std::setw(30) << "Left joystick U/D"
    << std::setw(24) << ("axis[1]=" + std::to_string((msg->axes.size() > 1) ? msg->axes[1] : 0.0))
    << scoops_lift_front_vel << " deg/s\n"
    << "  "
    << std::setw(4) << "[3]"
    << std::setw(18) << "Scoops Lift Back"
    << std::setw(30) << "Left joystick U/D"
    << std::setw(24) << ("axis[1]=" + std::to_string((msg->axes.size() > 1) ? msg->axes[1] : 0.0))
    << scoops_lift_back_vel << " deg/s\n"
    << "  "
    << std::setw(4) << "[4]"
    << std::setw(18) << "Sampler Lift Front"
    << std::setw(30) << "Right joystick U/D"
    << std::setw(24) << ("axis[3]=" + std::to_string(axis_sampler_lift))
    << sampler_lift_front_vel << " deg/s\n"
    << "  "
    << std::setw(4) << "[5]"
    << std::setw(18) << "Sampler Lift Back"
    << std::setw(30) << "Right joystick U/D"
    << std::setw(24) << ("axis[3]=" + std::to_string(axis_sampler_lift))
    << sampler_lift_back_vel << " deg/s\n"
    << "  "
    << std::setw(4) << "[6]"
    << std::setw(18) << "Conveyor Belt"
    << std::setw(30) << "Right joystick L/R"
    << std::setw(24) << ("axis[2]=" + std::to_string((msg->axes.size() > 2) ? msg->axes[2] : 0.0))
    << conveyor_belt_vel << " deg/s\n"
    << "  "
    << std::setw(4) << "[7]"
    << std::setw(18) << "Scoop Door Front"
    << std::setw(30) << "Y button"
    << std::setw(24) << ("btn[3]=" + std::to_string((msg->buttons.size() > 3) ? msg->buttons[3] : 0) +
                         " t=" + std::to_string(servo_scoop_f_toggle))
    << scoop_servo_front_pos << " deg\n"
    << "  "
    << std::setw(4) << "[8]"
    << std::setw(18) << "Scoop Door Back"
    << std::setw(30) << "A button"
    << std::setw(24) << ("btn[0]=" + std::to_string((msg->buttons.size() > 0) ? msg->buttons[0] : 0) +
                         " t=" + std::to_string(servo_scoop_b_toggle))
    << scoop_servo_back_pos << " deg\n"
    << "  "
    << std::setw(4) << "[9]"
    << std::setw(18) << "Auger Lift"
    << std::setw(30) << "Left joystick L/R"
    << std::setw(24) << ("axis[0]=" + std::to_string((msg->axes.size() > 0) ? msg->axes[0] : 0.0))
    << auger_lift_pos << " deg\n"
    << "  "
    << std::setw(4) << "[10]"
    << std::setw(18) << "Scoop Spinner"
    << std::setw(30) << "Right trigger, Right bumper"
    << std::setw(24) << ("trig/btn=" + std::to_string(scoop_axis) + "/" +
                         std::to_string((msg->buttons.size() > 5) ? msg->buttons[5] : 0) +
                         " r=" + std::to_string(scoop_reverse))
    << scoop_spinner_cmd << " deg/s\n"
    << "  "
    << std::setw(4) << "[11]"
    << std::setw(18) << "Auger Spinner"
    << std::setw(30) << "Left trigger, Left bumper"
    << std::setw(24) << ("trig/btn=" + std::to_string(auger_axis) + "/" +
                         std::to_string((msg->buttons.size() > 4) ? msg->buttons[4] : 0) +
                         " r=" + std::to_string(auger_reverse))
    << auger_spinner_cmd << " deg/s\n";

  if(DEBUG_MODE == 1){
    RCLCPP_INFO(get_node()->get_logger(), "%s", oss.str().c_str());
  }

  // Basic state publish (still reusing existing signals)
  for (const auto & joint_name : joints_) {
    if (state_publisher_ && state_publisher_->trylock()) {
      state_publisher_->msg_.header.stamp = get_node()->get_clock()->now();
      state_publisher_->msg_.header.frame_id = joint_name;
      state_publisher_->msg_.set_point = pump_right_cmd;
      state_publisher_->msg_.process_value = auger_spinner_cmd;
      state_publisher_->msg_.command = sampler_lift_front_vel;
      state_publisher_->unlockAndPublish();
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace science_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  science_controllers::ScienceManual, controller_interface::ControllerInterface)
