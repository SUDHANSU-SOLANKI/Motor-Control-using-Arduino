#include <string>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#ifndef MY_ROBOT_HARDWARE_HPP_
#define MY_ROBOT_HARDWARE_HPP_

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_robot_hardware
{

class MyRobotHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MyRobotHardware)

  // Serial
  int serial_fd_;
  std::string serial_port_;
  int baud_rate_;

  // Encoder
  long prev_left_ticks_;
  long prev_right_ticks_;
  const double CPR_ = 800.0;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  double hw_position_left_;
  double hw_position_right_;
  double hw_velocity_left_;
  double hw_velocity_right_;
  double hw_command_left_;
  double hw_command_right_;
};

}  // namespace my_robot_hardware

#endif  // MY_ROBOT_HARDWARE_HPP_
