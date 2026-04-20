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

#include "general_controllers/motor_status_controller.hpp"

#include <sstream>
#include <string>

#include "pluginlib/class_list_macros.hpp"

namespace general_controllers
{

MotorStatusController::MotorStatusController() {}

controller_interface::CallbackReturn MotorStatusController::on_init()
{
  try {
    param_listener_ = std::make_shared<motor_status_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception during on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MotorStatusController::command_interface_configuration() const
{
  // Broadcaster only reads, no command interfaces
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
MotorStatusController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : params_.joints) {
    for (const auto & iface : params_.interfaces) {
      config.names.push_back(joint + "/" + iface);
    }
  }

  return config;
}

controller_interface::CallbackReturn MotorStatusController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  if (params_.joints.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints specified for motor_status_controller.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.interfaces.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No interfaces specified for motor_status_controller.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Set up publish rate
  if (params_.publish_rate > 0.0) {
    publish_period_ = rclcpp::Duration::from_seconds(1.0 / params_.publish_rate);
  } else {
    publish_period_ = rclcpp::Duration(0, 0);
  }

  motor_status_publisher_ = get_node()->create_publisher<msgs::msg::SystemInfo>(
    "~/motor_status", rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured motor_status_controller for %zu joints, %zu interfaces, publish rate: %.1f Hz",
    params_.joints.size(), params_.interfaces.size(), params_.publish_rate);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MotorStatusController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Build lookup map from "joint/interface" -> state_interfaces_ index
  state_interface_map_.clear();
  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    state_interface_map_[state_interfaces_[i].get_prefix_name() + "/" +
                         state_interfaces_[i].get_interface_name()] = i;
  }

  last_publish_time_ = get_node()->now();

  RCLCPP_INFO(
    get_node()->get_logger(),
    "MotorStatusController activated with %zu state interfaces",
    state_interfaces_.size());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MotorStatusController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  state_interface_map_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MotorStatusController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // Rate limit publishing
  if (publish_period_.seconds() > 0.0 && (time - last_publish_time_) < publish_period_) {
    return controller_interface::return_type::OK;
  }
  last_publish_time_ = time;

  msgs::msg::SystemInfo system_info_msg;
  system_info_msg.header.stamp = time;

  for (const auto & joint : params_.joints) {
    msgs::msg::JointStatus status;
    status.joint_name = joint;

    // Initializing values in case state interfaces don't exist for them
    status.temperature   = std::numeric_limits<int8_t>::quiet_NaN();
    status.torque_current       = std::numeric_limits<double>::quiet_NaN();
    status.motor_status  = std::numeric_limits<int8_t>::quiet_NaN();
    status.brake_status  = "No Brakes";
    
    for (const auto & iface : params_.interfaces) {
      std::string key = joint + "/" + iface;
      auto it = state_interface_map_.find(key);

      if (it != state_interface_map_.end()) {
        double value = state_interfaces_[it->second].get_value();

        if (iface == "temperature") {
          status.temperature = static_cast<int8_t>(value);
        } else if (iface == "torque_current") {
          status.torque_current = value;
        } else if (iface == "status") {
          status.motor_status = static_cast<int8_t>(value);
          if (value > sizeof(MotorStatus)){
            RCLCPP_WARN(get_node()->get_logger(), "Invalid motor status value");
          }
        } else if (iface == "brake_status") {
          switch (static_cast<BrakeStatus>(value)){
            case BrakeStatus::LOCKED:
              status.brake_status = "Brakes are Locked";
              break;
            case BrakeStatus::RELEASED:
              status.brake_status = "Brakes are released";
              break;
            default:
              RCLCPP_WARN(get_node()->get_logger(), "Invalid brake status value");
              break;
          }
        }
      }
    }

    system_info_msg.joints.push_back(status);
  }

  motor_status_publisher_->publish(system_info_msg);

  return controller_interface::return_type::OK;
}

}  // namespace general_controllers

PLUGINLIB_EXPORT_CLASS(
  general_controllers::MotorStatusController,
  controller_interface::ControllerInterface)
