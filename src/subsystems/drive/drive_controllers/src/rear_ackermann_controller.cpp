// Copyright (c) 2025, UMDLoop
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "athena_drive_controllers/rear_ackermann_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "rclcpp/rclcpp.hpp"

namespace drive_controllers
{

RearAckermannController::RearAckermannController()
: controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn RearAckermannController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<rear_ackermann_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RearAckermannController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();
  steer_joint_names_ = params_.steer_joints;   // [bl, br]
  drive_joint_names_ = params_.drive_joints;    // [fl, fr, bl, br]

  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference", subscribers_qos,
    std::bind(&RearAckermannController::reference_callback, this, std::placeholders::_1));

  input_ref_.writeFromNonRT(std::make_shared<ControllerReferenceMsg>());

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

void RearAckermannController::reference_callback(
  const std::shared_ptr<ControllerReferenceMsg> msg)
{
  input_ref_.writeFromNonRT(msg);
}

controller_interface::InterfaceConfiguration
RearAckermannController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : steer_joint_names_) {
    config.names.push_back(joint + "/position");
  }
  for (const auto & joint : drive_joint_names_) {
    config.names.push_back(joint + "/velocity");
  }

  return config;
}

controller_interface::InterfaceConfiguration
RearAckermannController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::CallbackReturn RearAckermannController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    command_interfaces_[i].set_value(0.0);
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "RearAckermannController activated with all commands set to zero");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RearAckermannController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    command_interfaces_[i].set_value(0.0);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

// command_interfaces_ layout:
//   [0] steer_bl / position
//   [1] steer_br / position
//   [2] drive_fl / velocity (rad/s)
//   [3] drive_fr / velocity (rad/s)
//   [4] drive_bl / velocity (rad/s)
//   [5] drive_br / velocity (rad/s)

controller_interface::return_type RearAckermannController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto current_ref = input_ref_.readFromRT();
  if (!current_ref || !(*current_ref)) {
    for (size_t i = 0; i < command_interfaces_.size(); ++i) {
      command_interfaces_[i].set_value(0.0);
    }
    return controller_interface::return_type::OK;
  }

  const double v_cmd = std::clamp(
    (*current_ref)->twist.linear.x, -params_.max_speed, params_.max_speed);
  const double omega_cmd = (*current_ref)->twist.angular.z;

  const double L  = params_.wheelbase;
  const double T  = params_.track_width;
  const double rw = params_.wheel_radius;

  // Straight-line: steer to zero, all wheels at equal speed.
  if (std::abs(omega_cmd) < 1e-6) {
    const double wheel_vel = v_cmd / rw;

    command_interfaces_[0].set_value(0.0);
    command_interfaces_[1].set_value(0.0);
    command_interfaces_[2].set_value(wheel_vel);
    command_interfaces_[3].set_value(wheel_vel);
    command_interfaces_[4].set_value(wheel_vel);
    command_interfaces_[5].set_value(wheel_vel);

    log_wheel_speeds(wheel_vel, wheel_vel, wheel_vel, wheel_vel);
    return controller_interface::return_type::OK;
  }

  // Turn radius from bicycle model: R = v / omega.
  // Positive R → ICR to the left → left turn.
  const double R = v_cmd / omega_cmd;

  // Exact Ackermann rear steer angles: delta = atan2(L, R_side).
  // R_side is the lateral distance from each rear wheel to the ICR.
  const double R_bl = R + T / 2.0;
  const double R_br = R - T / 2.0;

  double delta_bl = std::atan2(L, R_bl);
  double delta_br = std::atan2(L, R_br);

  delta_bl = std::clamp(delta_bl, params_.min_steering_angle, params_.max_steering_angle);
  delta_br = std::clamp(delta_br, params_.min_steering_angle, params_.max_steering_angle);

  // Wheel ground speeds: v_i = omega * d_i, where d_i is distance from ICR to wheel i.
  // ICR is at (L, R) in vehicle frame (x forward, y left, origin at rear axle centre).
  // Front wheels share the ICR's x-coordinate, so d is purely lateral.
  // Rear wheels are offset in x by L, so d uses hypot.
  // Signed lateral offsets preserve velocity direction for inner-wheel reversal.
  const double signed_d_fl = R - T / 2.0;
  const double signed_d_fr = R + T / 2.0;

  const double d_bl = std::hypot(L, R - T / 2.0);
  const double d_br = std::hypot(L, R + T / 2.0);

  const double max_wheel_vel = params_.max_speed / rw;

  const double vel_fl = std::clamp(omega_cmd * signed_d_fl / rw, -max_wheel_vel, max_wheel_vel);
  const double vel_fr = std::clamp(omega_cmd * signed_d_fr / rw, -max_wheel_vel, max_wheel_vel);
  const double vel_bl = std::clamp(
    omega_cmd * std::copysign(d_bl, signed_d_fl) / rw, -max_wheel_vel, max_wheel_vel);
  const double vel_br = std::clamp(
    omega_cmd * std::copysign(d_br, signed_d_fr) / rw, -max_wheel_vel, max_wheel_vel);

  command_interfaces_[0].set_value(delta_bl);
  command_interfaces_[1].set_value(delta_br);
  command_interfaces_[2].set_value(vel_fl);
  command_interfaces_[3].set_value(vel_fr);
  command_interfaces_[4].set_value(vel_bl);
  command_interfaces_[5].set_value(vel_br);

  log_wheel_speeds(vel_fl, vel_fr, vel_bl, vel_br);

  return controller_interface::return_type::OK;
}

void RearAckermannController::log_wheel_speeds(
  double fl, double fr, double bl, double br) const
{
  constexpr double rad_s_to_rpm = 60.0 / (2.0 * M_PI);
  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
    "Wheel speeds [RPM] — FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f",
    fl * rad_s_to_rpm, fr * rad_s_to_rpm,
    bl * rad_s_to_rpm, br * rad_s_to_rpm);
}

}  // namespace drive_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  drive_controllers::RearAckermannController, controller_interface::ControllerInterface)

