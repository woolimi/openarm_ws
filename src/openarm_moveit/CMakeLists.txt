cmake_minimum_required(VERSION 3.8)
project(openarm_moveit)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
