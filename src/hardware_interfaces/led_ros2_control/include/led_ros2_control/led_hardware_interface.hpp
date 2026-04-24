#ifndef LED_ROS2_CONTROL__LED_HARDWARE_INTERFACE_HPP_
#define LED_ROS2_CONTROL__LED_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace led_ros2_control
{

namespace gpio_utils
{
int setup_gpio_output(int pin);
void cleanup_gpio(int pin, int fd);
bool write_gpio(int fd, bool value);
}

class LEDHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(LEDHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct LEDJoint
  {
    std::string name;
    int gpio_pin;
    bool default_state;
    double led_state;
    double is_connected;
    double status;
    double led_command;
    double status_request;
    double prev_status_request;
    double elapsed_status_request_time;
  };

  int gpio_fd_;
  bool hw_connected_;
  std::vector<LEDJoint> LEDJoints_;
};

}  // namespace led_ros2_control

#endif  // LED_ROS2_CONTROL__LED_HARDWARE_INTERFACE_HPP_
