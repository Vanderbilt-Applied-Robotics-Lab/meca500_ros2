/**
 * meca500 robot servo interface
 * 
 * Author: Garrison Johnston 12/2025
 */
#ifndef SERVO_HPP
#define SERVO_HPP

#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <moveit_servo/servo.hpp>
#include <moveit_servo/servo_parameters.hpp>
#include <planning_scene_monitor/planning_scene_monitor.h>

class Meca500ServoNode : public rclcpp::Node
{
public:
    explicit Meca500ServoNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    void initializeServo();

    moveit_servo::ServoParameters servo_params_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    std::shared_ptr<moveit_servo::Servo> servo_;
};

#endif // SERVO_HPP
