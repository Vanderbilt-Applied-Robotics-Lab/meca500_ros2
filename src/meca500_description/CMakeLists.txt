cmake_minimum_required(VERSION 3.8)
project(meca500_description)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY urdf meshes/collision meshes/visual
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
