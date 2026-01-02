#include "meca500_hardware/meca500_hardware.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <sstream>

using meca500_hardware::SuccessCode;

namespace meca500_hardware
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

    // Meca500 startup sequence
    if (!activateRobot())
    {
      RCLCPP_WARN(rclcpp::get_logger("Meca500Hardware"), "Failed to activate robot");
      return hardware_interface::CallbackReturn::ERROR;
    }
    
    if (!homeRobot())
    {
      RCLCPP_WARN(rclcpp::get_logger("Meca500Hardware"), "Failed to home robot");
      return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn
  Meca500Hardware::on_deactivate(const rclcpp_lifecycle::State &)
  {
    deactivateRobot();
    disconnect();
    return hardware_interface::CallbackReturn::SUCCESS;
  }

hardware_interface::return_type
Meca500Hardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    std::lock_guard<std::mutex> lock(socket_mutex_);

    sendCommand("GetJoints");
    std::string response = receiveResponse();

    // Look for the first '[' after the timestamp
    auto start = response.find('[');
    if (start != std::string::npos)
    {
        start = response.find('[', start + 1);
        auto end = response.find(']', start);
        if (end != std::string::npos)
        {
            std::string joints_str = response.substr(start + 1, end - start - 1);
            std::istringstream ss(joints_str);
            std::string token;
            size_t i = 0;

            while (std::getline(ss, token, ',') && i < hw_states_pos_.size())
            {
                // Remove whitespace
                token.erase(remove_if(token.begin(), token.end(), ::isspace), token.end());

                // Only parse numeric tokens
                if (!token.empty() && (isdigit(token[0]) || token[0] == '-' || token[0] == '+'))
                {
                    try
                    {
                        hw_states_pos_[i] = std::stod(token) * M_PI / 180.0;  // convert to radians
                    }
                    catch (const std::exception &e)
                    {
                        RCLCPP_WARN(rclcpp::get_logger("Meca500Hardware"),
                            "Failed to parse joint %zu: '%s'", i, token.c_str());
                    }
                }
                else
                {
                    RCLCPP_DEBUG(rclcpp::get_logger("Meca500Hardware"),
                        "Ignoring non-numeric token: '%s'", token.c_str());
                }

                i++;
            }
        }
    }

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
    double degrees = hw_commands_pos_[i] * 180.0 / M_PI;  // convert radians -> degrees
    cmd << degrees;
    if (i < hw_commands_pos_.size() - 1)
      cmd << ",";
  }
  cmd << ")";

    sendCommand(cmd.str());
    return hardware_interface::return_type::OK;
  }

  bool Meca500Hardware::activateRobot()
  {
    RCLCPP_INFO(rclcpp::get_logger("Meca500Hardware"), "Activating Robot...");
    sendCommand("ActivateRobot");
    return waitForReturnCode(SuccessCode::MOTORS_ACTIVATED, SuccessCode::MOTORS_ALREADY_ACTIVATED); // Wait for activation done
  }
  
  bool Meca500Hardware::homeRobot()
  {
    RCLCPP_INFO(rclcpp::get_logger("Meca500Hardware"), "Homing Robot...");
    sendCommand("Home");
    return waitForReturnCode(SuccessCode::HOMING_DONE, SuccessCode::ALREADY_HOMED); // Wait for activation done
  }
  
  bool Meca500Hardware::deactivateRobot()
  {
    RCLCPP_INFO(rclcpp::get_logger("Meca500Hardware"), "Deactivating Robot...");
    sendCommand("DeactivateRobot");
    return waitForReturnCode(SuccessCode::MOTORS_DEACTIVATED); // Wait for deactivation done
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

  bool Meca500Hardware::sendCommand(const std::string &cmd)
  {
    std::string msg = cmd + "\n";
    return send(socket_fd_, msg.c_str(), msg.size(), 0) >= 0;
  }

  std::string Meca500Hardware::receiveResponse()
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

  bool Meca500Hardware::waitForReturnCode(SuccessCode code1, SuccessCode code2)
  {
    SuccessCode received_code{0};

    while (received_code != code1 && received_code != code2)
    {
      std::string full_code = receiveResponse();
      received_code = parseReturnCode(full_code); 
      
      RCLCPP_INFO(rclcpp::get_logger("Meca500Hardware"),
                        "got return code: %s", full_code.c_str());

      // return 1 and break if an error happens
      if (isErrorCode(received_code))
      {
        RCLCPP_WARN(rclcpp::get_logger("Meca500Hardware"),
                        "got error code: %s", getErrorMessage(received_code).c_str());
        return false; // failure
      }
    }

    return true; // Success
  }

  SuccessCode Meca500Hardware::parseReturnCode(std::string raw_code)
  {
    return static_cast<meca500_hardware::SuccessCode>(std::stoul(raw_code.substr(1, 4)));
  }

} // namespace meca500_hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  meca500_hardware::Meca500Hardware,
  hardware_interface::SystemInterface)
