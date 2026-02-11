#include "my_robot_hardware/my_robot_hardware.hpp"

#include <unistd.h>     // ::read, ::write, close
#include <fcntl.h>      // open
#include <termios.h>    // termios
#include <cstring>      // strlen
#include <cmath>        // M_PI
#include <cstdio>       // sscanf

namespace my_robot_hardware
{

/* ================= SERIAL HELPER ================= */

int open_serial(const std::string &port, int baudrate)
{
  int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0)
  {
    return -1;
  }

  struct termios tty;
  if (tcgetattr(fd, &tty) != 0)
  {
    close(fd);
    return -1;
  }

  // Baud rate (fixed to 115200 for now)
  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_iflag &= ~IGNBRK;
  tty.c_lflag = 0;
  tty.c_oflag = 0;

  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 1;

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  tcsetattr(fd, TCSANOW, &tty);
  return fd;
}

/* ================= ROS2 CONTROL ================= */

hardware_interface::CallbackReturn MyRobotHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Joint states
  hw_position_left_ = 0.0;
  hw_position_right_ = 0.0;
  hw_velocity_left_ = 0.0;
  hw_velocity_right_ = 0.0;
  hw_command_left_ = 0.0;
  hw_command_right_ = 0.0;

  prev_left_ticks_ = 0;
  prev_right_ticks_ = 0;

  // Serial settings (hardcoded for now)
  serial_port_ = "/dev/ttyACM0";
  baud_rate_ = 115200;

  serial_fd_ = open_serial(serial_port_, baud_rate_);
  if (serial_fd_ < 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("MyRobotHardware"),
      "Failed to open serial port %s",
      serial_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("MyRobotHardware"),
    "Serial port opened: %s",
    serial_port_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

/* ================= INTERFACES ================= */

std::vector<hardware_interface::StateInterface>
MyRobotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(
    info_.joints[0].name,
    hardware_interface::HW_IF_POSITION,
    &hw_position_left_);

  state_interfaces.emplace_back(
    info_.joints[0].name,
    hardware_interface::HW_IF_VELOCITY,
    &hw_velocity_left_);

  state_interfaces.emplace_back(
    info_.joints[1].name,
    hardware_interface::HW_IF_POSITION,
    &hw_position_right_);

  state_interfaces.emplace_back(
    info_.joints[1].name,
    hardware_interface::HW_IF_VELOCITY,
    &hw_velocity_right_);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MyRobotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
    info_.joints[0].name,
    hardware_interface::HW_IF_VELOCITY,
    &hw_command_left_);

  command_interfaces.emplace_back(
    info_.joints[1].name,
    hardware_interface::HW_IF_VELOCITY,
    &hw_command_right_);

  return command_interfaces;
}

/* ================= LIFECYCLE ================= */

hardware_interface::CallbackReturn MyRobotHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    rclcpp::get_logger("MyRobotHardware"),
    "Hardware interface activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyRobotHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    rclcpp::get_logger("MyRobotHardware"),
    "Hardware interface deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

/* ================= READ ================= */

hardware_interface::return_type MyRobotHardware::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  char buf[64];
  int n = ::read(serial_fd_, buf, sizeof(buf) - 1);

  if (n <= 0)
  {
    return hardware_interface::return_type::OK;
  }

  buf[n] = '\0';

  long left_ticks = 0;
  long right_ticks = 0;

  if (sscanf(buf, "E %ld %ld", &left_ticks, &right_ticks) == 2)
  {
    double dt = period.seconds();
    if (dt <= 0.0)
      return hardware_interface::return_type::OK;

    long d_left = left_ticks - prev_left_ticks_;
    long d_right = right_ticks - prev_right_ticks_;

    prev_left_ticks_ = left_ticks;
    prev_right_ticks_ = right_ticks;

    // Position (rad)
    hw_position_left_  += (d_left  / CPR_) * 2.0 * M_PI;
    hw_position_right_ += (d_right / CPR_) * 2.0 * M_PI;

    // Velocity (rad/s)
    hw_velocity_left_  = (d_left  / CPR_) * 2.0 * M_PI / dt;
    hw_velocity_right_ = (d_right / CPR_) * 2.0 * M_PI / dt;
  }

  return hardware_interface::return_type::OK;
}

/* ================= WRITE ================= */

hardware_interface::return_type MyRobotHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  char cmd[64];
  snprintf(
    cmd, sizeof(cmd),
    "V %.3f %.3f\n",
    hw_command_left_,
    hw_command_right_);

  ::write(serial_fd_, cmd, strlen(cmd));

  return hardware_interface::return_type::OK;
}

}  // namespace my_robot_hardware
