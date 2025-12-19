/**
 * meca500 robot hardware interface
 * 
 * Author: Garrison Johnston 12/2025
 */
#ifndef MECA500_HARDWARE_HPP
#define MECA500_HARDWARE_HPP

#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/macros.hpp>

#include <string>
#include <vector>
#include <mutex>

namespace Meca500Robot
{

    class HARDWARE_INTERFACE_PUBLIC Meca500Hardware : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(Meca500Hardware)

        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::return_type read(
            const rclcpp::Time &time,
            const rclcpp::Duration &period) override;

        hardware_interface::return_type write(
            const rclcpp::Time &time,
            const rclcpp::Duration &period) override;

    private:
        // Networking
        bool connect();
        void disconnect();
        bool send_command(const std::string &cmd);
        std::string receive_response();

        std::string robot_ip_;
        int robot_port_;
        int socket_fd_;

        // Joint data
        std::vector<double> hw_states_pos_;
        std::vector<double> hw_commands_pos_;

        std::mutex socket_mutex_;
    };

} // namespace meca500_hardware

#endif // MECA500_HARDWARE_HPP
