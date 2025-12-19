#include "meca500_hardware/meca500_hardware.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <sstream>

namespace Meca500Robot
{

  hardware_interface::CallbackReturn
  Meca500Hardware::on_init(const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    robot_ip_ = info.hardware_parameters.at("robot_ip");
    robot_port_ = std::stoi(info.hardware_parameters.at("robot_port"));

    size_t num_joints = info.joints.size();
    hw_states_pos_.resize(num_joints, 0.0);
    hw_commands_pos_.resize(num_joints, 0.0);

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface>
  Meca500Hardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < hw_states_pos_.size(); i++)
    {
      state_interfaces.emplace_back(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_pos_[i]);
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  Meca500Hardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < hw_commands_pos_.size(); i++)
    {
      command_interfaces.emplace_back(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_pos_[i]);
    }
    return command_interfaces;
  }

  hardware_interface::CallbackReturn
  Meca500Hardware::on_activate(const rclcpp_lifecycle::State &)
  {
    if (!connect())
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Meca500 startup sequence see Mecademic examples
    send_command("ActivateRobot");
    send_command("Home");
    send_command("StartJog");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn
  Meca500Hardware::on_deactivate(const rclcpp_lifecycle::State &)
  {
    send_command("DeactivateRobot");
    disconnect();
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type
  Meca500Hardware::read(const rclcpp::Time &, const rclcpp::Duration &)
  {
    // Optional: query joint positions if needed
    // send_command("GetJoints");
    // parse response into hw_states_pos_

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type
  Meca500Hardware::write(const rclcpp::Time &, const rclcpp::Duration &)
  {
    std::lock_guard<std::mutex> lock(socket_mutex_);

    std::ostringstream cmd;
    cmd << "MoveJoints(";
    for (size_t i = 0; i < hw_commands_pos_.size(); i++)
    {
      cmd << hw_commands_pos_[i];
      if (i < hw_commands_pos_.size() - 1)
        cmd << ",";
    }
    cmd << ")";

    send_command(cmd.str());
    return hardware_interface::return_type::OK;
  }

  /* ---------------- TCP helpers ---------------- */

  bool Meca500Hardware::connect()
  {
    socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd_ < 0)
      return false;

    sockaddr_in server{};
    server.sin_family = AF_INET;
    server.sin_port = htons(robot_port_);
    inet_pton(AF_INET, robot_ip_.c_str(), &server.sin_addr);

    return ::connect(socket_fd_, (sockaddr *)&server, sizeof(server)) == 0;
  }

  void Meca500Hardware::disconnect()
  {
    if (socket_fd_ >= 0)
      close(socket_fd_);
  }

  bool Meca500Hardware::send_command(const std::string &cmd)
  {
    std::string msg = cmd + "\n";
    return send(socket_fd_, msg.c_str(), msg.size(), 0) >= 0;
  }

  std::string Meca500Hardware::receive_response()
  {
    char buffer[1024];
    ssize_t len = recv(socket_fd_, buffer, sizeof(buffer) - 1, 0);
    if (len > 0)
    {
      buffer[len] = '\0';
      return std::string(buffer);
    }
    return "";
  }

} // namespace Meca500Hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  Meca500Robot::Meca500Hardware,
  hardware_interface::SystemInterface)
