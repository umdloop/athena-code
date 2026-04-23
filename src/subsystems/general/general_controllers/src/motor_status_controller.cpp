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

  status_req_joint_name = "";
  status_request_rate = 0;
  maintenance_req_joint_name = "";
  maintenance_request_rate = 0;
  status_one_shot_sent = false;

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

  status_request_service_ = get_node()->create_service<msgs::srv::StatusReq>(
    "~/status_request",
    [this](
      const std::shared_ptr<msgs::srv::StatusReq::Request> request,
      std::shared_ptr<msgs::srv::StatusReq::Response> response)
    {
      if (request->joint_name.empty()) {
        response->success = false;
        response->message = "Joint name cannot be empty";
        RCLCPP_WARN(get_node()->get_logger(), "%s", response->message.c_str());
        return;
      }

      status_req_joint_name = request->joint_name;
      status_request_rate = request->request_rate;

      std::string msg = "Received status_request for joint " + status_req_joint_name + " at request_rate: " + std::to_string(status_request_rate);
      response->success = true;
      response->message = msg;
      RCLCPP_INFO(get_node()->get_logger(), "%s", msg.c_str());
    });

  maintenance_request_service_ = get_node()->create_service<msgs::srv::MaintenanceReq>(
    "~/maintenance_request",
    [this](
      const std::shared_ptr<msgs::srv::MaintenanceReq::Request> request,
      std::shared_ptr<msgs::srv::MaintenanceReq::Response> response)
    {
      if (request->joint_name.empty()) {
        response->success = false;
        response->message = "Joint name cannot be empty";
        RCLCPP_WARN(get_node()->get_logger(), "%s", response->message.c_str());
        return;
      }

      maintenance_req_joint_name = request->joint_name;
      maintenance_request_rate = request->request_rate;

      auto packed = pack_command_full(
                    request->command_id,
                    request->u8_data,
                    request->i16_data,
                    request->i32_data);

      auto payload_to_doubles = [](int64_t value) -> std::pair<double, double>
      {
          uint64_t u = static_cast<uint64_t>(value);
          double high = static_cast<double>(u >> 32);
          double low  = static_cast<double>(u & 0xFFFFFFFF);
          return {high, low};
      };

      auto [high, low] = payload_to_doubles(packed.payload);

      maintenance_frame_high = high;
      maintenance_frame_low  = low;
      maintenance_data_count = static_cast<double>(packed.counts);

      std::ostringstream oss;

      oss << "Received maintenance_request for joint "
          << maintenance_req_joint_name
          << " with frame: 0x"
          << std::hex << std::uppercase
          << std::setfill('0') << std::setw(8) << static_cast<uint64_t>(maintenance_frame_high)
          << std::setfill('0') << std::setw(8) << static_cast<uint64_t>(maintenance_frame_low)
          << std::dec << std::nouppercase
          << " and data count(s): "
          << maintenance_data_count
          << " at request_rate: "
          << maintenance_request_rate;

      std::string msg = oss.str();
      response->success = true;
      response->message = msg;
      RCLCPP_INFO(get_node()->get_logger(), "%s", msg.c_str());
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
  // Rate limit publishing
  const bool periodic_publish_due =
    (publish_period_.seconds() <= 0.0 || (time - last_publish_time_) >= publish_period_);

  if (!periodic_publish_due) {
    return controller_interface::return_type::OK;
  }
  last_publish_time_ = time;

  msgs::msg::SystemInfo system_info_msg;
  system_info_msg.header.stamp = time;

  for (const auto & joint : params_.joints) {

    // Command Interfaces
    for (const auto & iface : params_.command_interfaces) {
      std::string key = joint + "/" + iface;
      auto it = command_interface_map_.find(key);
      if (it != command_interface_map_.end()) {
        if (iface == "maintenance_frame_high" && joint == maintenance_req_joint_name) {
          command_interfaces_[it->second].set_value(maintenance_frame_high);
        }
        else if (iface == "maintenance_frame_low" && joint == maintenance_req_joint_name) {
          command_interfaces_[it->second].set_value(maintenance_frame_low);
        }
        else if(iface == "maintenance_data_count" && joint == maintenance_req_joint_name) {
          command_interfaces_[it->second].set_value(maintenance_data_count);
        }
        else if (iface == "status_request" && joint == status_req_joint_name) {
          // Controller must turn status request back to 0 if it's a one-shot request
          if (status_request_rate < 0 && status_one_shot_sent == false) {
            command_interfaces_[it->second].set_value(status_request_rate);
            RCLCPP_WARN(get_node()->get_logger(), "Status one shot sent: ");
            status_one_shot_sent = true;
            status_one_shot_time = time;
          }
          else if (status_request_rate < 0 && status_one_shot_sent == true && (time - status_one_shot_time) >= one_shot_delay) {
            status_request_rate = 0;
            command_interfaces_[it->second].set_value(status_request_rate);
            status_one_shot_sent = false;
            RCLCPP_WARN(get_node()->get_logger(), "Status one shot reset.");
          }
          else if (status_request_rate >= 0) {
            command_interfaces_[it->second].set_value(status_request_rate);
          }
        }
        else if (iface == "maintenance_request" && joint == maintenance_req_joint_name) {
          // Controller must turn maintenance request back to 0 if it's a one-shot request
          if (maintenance_request_rate < 0 && maintenance_one_shot_sent == false) {
            command_interfaces_[it->second].set_value(maintenance_request_rate);
            RCLCPP_WARN(get_node()->get_logger(), "Maintenance one shot sent: ");
            maintenance_one_shot_sent = true;
            maintenance_one_shot_time = time;
          }
          else if (maintenance_request_rate < 0 && maintenance_one_shot_sent == true && (time - maintenance_one_shot_time) >= one_shot_delay) {
            maintenance_request_rate = 0;
            command_interfaces_[it->second].set_value(maintenance_request_rate);
            maintenance_one_shot_sent = false;
            RCLCPP_WARN(get_node()->get_logger(), "Maintenance one shot reset.");
          }
          else if (maintenance_request_rate >= 0) {
            command_interfaces_[it->second].set_value(maintenance_request_rate);
          }
        }
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
