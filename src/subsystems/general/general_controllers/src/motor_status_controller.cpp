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
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(params_.joints.size() * params_.command_interfaces.size());
  for (const auto & joint : params_.joints)
  {
    for (const auto & iface : params_.command_interfaces) {
      command_interfaces_config.names.push_back(joint + "/" + iface);
    }
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
MotorStatusController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : params_.joints) {
    for (const auto & iface : params_.state_interfaces) {
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

  if (params_.state_interfaces.empty()) {
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

  auto configure_request_rate =
    [this](const int32_t request_rate, const std::string & service_name) -> std::string
    {
      if (request_rate < 0) {
        publish_enabled_ = false;
        publish_once_requested_ = true;
        return service_name + " queued a one-shot publish";
      }

      publish_once_requested_ = false;
      if (request_rate == 0) {
        publish_enabled_ = false;
        publish_period_ = rclcpp::Duration(0, 0);
        return service_name + " stopped publishing";
      }

      publish_enabled_ = true;
      publish_period_ = rclcpp::Duration::from_seconds(1.0 / static_cast<double>(request_rate));
      return service_name + " set publish rate to " + std::to_string(request_rate) + " Hz";
    };

  status_request_service_ = get_node()->create_service<msgs::srv::StatusReq>(
    "~/status_request",
    [this, configure_request_rate](
      const std::shared_ptr<msgs::srv::StatusReq::Request> request,
      std::shared_ptr<msgs::srv::StatusReq::Response> response)
    {
      response->success = true;
      response->message = configure_request_rate(request->request_rate, "status_request");
      RCLCPP_INFO(
        get_node()->get_logger(),
        "Received status_request request_rate: %d",
        request->request_rate);
    });

  maintenance_request_service_ = get_node()->create_service<msgs::srv::MaintenanceReq>(
    "~/maintenance_request",
    [this, configure_request_rate](
      const std::shared_ptr<msgs::srv::MaintenanceReq::Request> request,
      std::shared_ptr<msgs::srv::MaintenanceReq::Response> response)
    {
      response->success = true;
      response->message = configure_request_rate(request->request_rate, "maintenance_request");
      RCLCPP_INFO(
        get_node()->get_logger(),
        "Received maintenance_request request_rate: %d",
        request->request_rate);
    });

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured motor_status_controller for %zu joints, %zu interfaces, publish rate: %.1f Hz",
    params_.joints.size(), params_.state_interfaces.size(), params_.publish_rate);

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

  // Build lookup map from "joint/interface" -> command_interfaces_ index
  command_interface_map_.clear();
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    command_interface_map_[command_interfaces_[i].get_prefix_name() + "/" +
                          command_interfaces_[i].get_interface_name()] = i;
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
  command_interface_map_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MotorStatusController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  const bool periodic_publish_due =
    publish_enabled_ &&
    (publish_period_.seconds() <= 0.0 || (time - last_publish_time_) >= publish_period_);

  if (!publish_once_requested_ && !periodic_publish_due) {
    return controller_interface::return_type::OK;
  }

  publish_once_requested_ = false;
  last_publish_time_ = time;

  msgs::msg::SystemInfo system_info_msg;
  system_info_msg.header.stamp = time;

  for (const auto & joint : params_.joints) {

    // Command Interfaces
    for (const auto & iface : params_.command_interfaces) {
      std::string key = joint + "/" + iface;
      auto it = command_interface_map_.find(key);
      if (it != command_interface_map_.end()) {
        double value = command_interfaces_[it->second].get_value();
        RCLCPP_INFO(get_node()->get_logger(), "Command Interface - Joint: %s, Interface: %s, Value: %f",
                    joint.c_str(), iface.c_str(), value);
      } else {
        RCLCPP_WARN(get_node()->get_logger(), "Command interface %s not found for joint %s", iface.c_str(), joint.c_str());
      }
    }
    
    // State Interfaces
    msgs::msg::JointStatus status;
    status.joint_name = joint;

    // Initializing values in case state interfaces don't exist for them
    status.temperature   = std::numeric_limits<int8_t>::quiet_NaN();
    status.torque_current       = std::numeric_limits<double>::quiet_NaN();
    status.motor_status  = std::numeric_limits<int8_t>::quiet_NaN();
    status.brake_status  = "No Brakes";
    
    for (const auto & iface : params_.state_interfaces) {
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
