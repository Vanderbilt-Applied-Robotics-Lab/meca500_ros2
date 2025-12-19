cmake_minimum_required(VERSION 3.8)
project(meca500_servo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# include
include_directories(include)

# executable
add_executable(servo src/servo.cpp)
ament_target_dependencies(servo
  rclcpp
  geometry_msgs
  moveit_msgs
  control_msgs
  std_srvs
  moveit_servo
  moveit_ros_planning
  moveit_ros_planning_interface
)

# install
install(
  TARGETS servo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
