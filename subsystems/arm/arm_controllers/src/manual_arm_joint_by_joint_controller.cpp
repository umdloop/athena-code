// Copyright (c) 2025, UMDLoop
// Copyright (c) 2025, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

//
// Source of this file are templates in
// [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
//

#include "arm_controllers/manual_arm_joint_by_joint_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"

namespace
{  // utility

// TODO(destogl): remove this when merged upstream
// Changed services history QoS to keep all so we don't lose any client service calls
static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  1,  // message queue depth
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};

using ControllerReferenceMsg = arm_controllers::ManualArmJointByJointController::ControllerReferenceMsg;

// called from RT control loop
void reset_controller_reference_msg(
  std::shared_ptr<ControllerReferenceMsg> & msg, const int & axes_count, const int & button_count)
{
  msg->axes.resize(axes_count, std::numeric_limits<double>::quiet_NaN());
  msg->buttons.resize(button_count, std::numeric_limits<int32_t>::quiet_NaN());
}

}  // namespace

namespace arm_controllers
{
ManualArmJointByJointController::ManualArmJointByJointController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn ManualArmJointByJointController::on_init()
{
  control_mode_.initRT(control_mode_type::FAST);

  try
  {
    param_listener_ = std::make_shared<manual_arm_joint_by_joint_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ManualArmJointByJointController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  if (!params_.state_joints.empty())
  {
    state_joints_ = params_.state_joints;
  }
  else
  {
    state_joints_ = params_.joints;
  }

  if (params_.joints.size() != state_joints_.size())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of 'joints' (%zu) and 'state_joints' (%zu) parameters has to be the same!",
      params_.joints.size(), state_joints_.size());
    return CallbackReturn::FAILURE;
  }

  joint_velocities_.resize(params_.joints.size(), 0.0); // Output
  max_velocities_ = params_.joint_max_velocities;

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Subscriber
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "controller_input", subscribers_qos,
    std::bind(&ManualArmJointByJointController::reference_callback, this, std::placeholders::_1));
  
  // Create, populate with NaN, and write message to input_ref_ to be used in reference callback
  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg, joystick_axes, joystick_buttons);
  input_ref_.writeFromNonRT(msg);

  auto set_slow_mode_service_callback =
    [&](
      const std::shared_ptr<ControllerModeSrvType::Request> request,
      std::shared_ptr<ControllerModeSrvType::Response> response)
  {
    if (request->data)
    {
      control_mode_.writeFromNonRT(control_mode_type::SLOW);
    }
    else
    {
      control_mode_.writeFromNonRT(control_mode_type::FAST);
    }
    response->success = true;
  };

  set_slow_control_mode_service_ = get_node()->create_service<ControllerModeSrvType>(
    "~/set_slow_control_mode", set_slow_mode_service_callback,
    rmw_qos_profile_services_hist_keep_all);

  try
  {
    // State publisher
    s_publisher_ =
      get_node()->create_publisher<ControllerStateMsg>("~/state", rclcpp::SystemDefaultsQoS());
    state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // TODO(anyone): Reserve memory in state publisher depending on the message type
  state_publisher_->lock();
  state_publisher_->msg_.header.frame_id = params_.joints[0];
  state_publisher_->unlock();

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

void ManualArmJointByJointController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  /*
  if (msg->joint_names.size() == params_.joints.size())
  {
    input_ref_.writeFromNonRT(msg);
  }
  else
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Received %zu , but expected %zu joints in command. Ignoring message.",
      msg->joint_names.size(), params_.joints.size());
  }
      */

  input_ref_.writeFromNonRT(msg);
}

controller_interface::InterfaceConfiguration ManualArmJointByJointController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(params_.joints.size());
  for (const auto & joint : params_.joints)
  {
    command_interfaces_config.names.push_back(joint + "/" + params_.interface_name);
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration ManualArmJointByJointController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(state_joints_.size());
  for (const auto & joint : state_joints_)
  {
    state_interfaces_config.names.push_back(joint + "/" + params_.interface_name);
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn ManualArmJointByJointController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): if you have to manage multiple interfaces that need to be sorted check
  // `on_activate` method in `JointTrajectoryController` for exemplary use of
  // `controller_interface::get_ordered_interfaces` helper function

  // Set default value in command
  reset_controller_reference_msg(*(input_ref_.readFromRT)(), joystick_axes, joystick_buttons);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ManualArmJointByJointController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ManualArmJointByJointController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  auto current_ref = input_ref_.readFromRT();

  if (!std::isnan((*current_ref)->axes[0]))
  {
    // ODrive
    joint_velocities_[0] = ((*current_ref)->buttons[1] == 1) ? 0.0 : (*current_ref)->axes[0] * max_velocities_[0];

    // RMD
    joint_velocities_[1] = ((*current_ref)->buttons[1] == 1) ? 0.0 : -(*current_ref)->axes[1] * max_velocities_[1]; // Shoulder Pitch
    joint_velocities_[2] = (*current_ref)->axes[3] * max_velocities_[2];  // Elbow Pitch

    // SMC
    joint_velocities_[3] = -(*current_ref)->axes[1] * static_cast<float>((*current_ref)->buttons[1]) * max_velocities_[3];
    joint_velocities_[4] = -(*current_ref)->axes[0] * static_cast<float>((*current_ref)->buttons[1]) * max_velocities_[4];

    // Talons
    if ((*current_ref)->buttons[4] && (*current_ref)->buttons[5]) {
    // closeClaw(motor);
    } else if ((*current_ref)->buttons[4]) { // Left bumper
      joint_velocities_[5] = max_velocities_[5]; // open claw
    } else if ((*current_ref)->buttons[5]) { // Right bumper
      joint_velocities_[5] = -max_velocities_[5]; // close claw
    } else if ((*current_ref)->axes[4]) { // Left Trigger
      joint_velocities_[5] = (*current_ref)->axes[4] * max_velocities_[5]; // open claw
    } else if ((*current_ref)->axes[5]) { // Right Trigger
      joint_velocities_[5] = -(*current_ref)->axes[5] * max_velocities_[5]; // close claw
    } else{
      joint_velocities_[5] = 0.0;
    }
    // RCLCPP_INFO(get_node()->get_logger(), "Talon: %f", joint_velocities_[5]);
  }
  else{
    RCLCPP_INFO(get_node()->get_logger(), "Returning NaN");
    joint_velocities_.resize(6, 0.0);
  }

  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    if (*(control_mode_.readFromRT()) == control_mode_type::SLOW)
    {
      joint_velocities_[i] /= 2;
    }
    // RCLCPP_INFO(get_node()->get_logger(), "Joint %d: %f", i, joint_velocities_[i]);
    command_interfaces_[i].set_value(joint_velocities_[i]);

    // (*current_ref)->axes[i] = std::numeric_limits<double>::quiet_NaN();
    // (*current_ref)->buttons[i] = std::numeric_limits<double>::quiet_NaN();
  }

  if (state_publisher_ && state_publisher_->trylock())
  {
    state_publisher_->msg_.header.stamp = time;
    state_publisher_->msg_.set_point = command_interfaces_[CMD_MY_ITFS].get_value();
    state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace arm_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  arm_controllers::ManualArmJointByJointController, controller_interface::ControllerInterface)
