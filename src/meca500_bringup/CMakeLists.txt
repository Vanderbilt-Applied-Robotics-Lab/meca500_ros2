cmake_minimum_required(VERSION 3.5)
project(meca500_bringup)


# Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(pluginlib REQUIRED)
find_package(urdf REQUIRED)

# Install launch files
install(
    DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()

