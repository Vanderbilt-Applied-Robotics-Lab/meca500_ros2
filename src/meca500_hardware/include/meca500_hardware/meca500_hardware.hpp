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

#include <rclcpp/rclcpp.hpp>
#include <meca500_hardware/response_codes.hpp>


namespace meca500_hardware
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
        
        /**
         * Activates the robot. Will block until completed.
         * @return true if robot activated, false if error received 
         */
        bool activateRobot();

        /**
         * Homes the robot. Will block until completed.
         * @return true if robot homed, false if error 
         */
        bool homeRobot();

        /**
         * Deactivates the robot. Will block until completed.
         * @return true if robot deactivated, false if error 
         */
        bool deactivateRobot();

    private:
        // Networking
        /**
         * Connect to the robot
         * @return true if connection successful, false otherwise
         */
        bool connect();

        /**
         * disconnect from the robot
         */
        void disconnect();

        /**
         * Send a command to the robot
         * @param cmd The command to send to the robot
         * @return true if send was successful, false otherwise
         */
        bool sendCommand(const std::string &cmd);

        /**
         * Receive a single response from the robot.
         * @return Response string received from the robot
         */
        std::string receiveResponse();

        /**
         * Wait for specific return code from robot
         * blocks until code1 or code2 is returned or error happens
         * @param code1 Primary expected return code
         * @param code2 Optional secondary expected return code (default 0)
         * @return true if success, false if error received
         */
        bool waitForReturnCode(SuccessCode code1, SuccessCode code2 = SuccessCode{0});

        /**
         * Parse a raw string from the robot into a SuccessCode.
         * @param raw_code Raw string received from the robot
         * @return Parsed SuccessCode
         */
        SuccessCode parseReturnCode(std::string raw_code);


        std::string robot_ip_; 
        int robot_port_;
        int socket_fd_; // socket file descriptor

        // Joint data
        std::vector<double> hw_states_pos_;
        std::vector<double> hw_commands_pos_;

        std::mutex socket_mutex_;
    };

} // namespace meca500_hardware

#endif // MECA500_HARDWARE_HPP
