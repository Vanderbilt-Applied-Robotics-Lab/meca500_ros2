#include "meca500_servo/servo.hpp"

#include <chrono>

using namespace std::chrono_literals;

Meca500ServoNode::Meca500ServoNode(const rclcpp::NodeOptions& options)
: Node("meca500_servo_node", options)
{
  RCLCPP_INFO(get_logger(), "Starting Meca500 Servo Node");
  initializeServo();
}

void Meca500ServoNode::initializeServo()
{
  // -----------------------------------------
  // Load Servo parameters
  // -----------------------------------------
  auto param_listener =
    std::make_shared<moveit_servo::ServoParameters::ParamListener>(
      shared_from_this());

  servo_params_ = param_listener->get_params();

  // -----------------------------------------
  // Planning Scene Monitor
  // -----------------------------------------
  planning_scene_monitor_ =
    std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      shared_from_this(), "robot_description");

  planning_scene_monitor_->startSceneMonitor();
  planning_scene_monitor_->startWorldGeometryMonitor();
  planning_scene_monitor_->startStateMonitor();

  if (!planning_scene_monitor_->getPlanningScene())
  {
    RCLCPP_FATAL(get_logger(), "Failed to initialize planning scene");
    rclcpp::shutdown();
    return;
  }

  // -----------------------------------------
  // Servo
  // -----------------------------------------
  servo_ = std::make_shared<moveit_servo::Servo>(
    shared_from_this(),
    servo_params_,
    planning_scene_monitor_);

  servo_->start();

  RCLCPP_INFO(get_logger(), "Meca500 Servo Node started successfully");
}
