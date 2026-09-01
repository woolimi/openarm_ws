cmake_minimum_required(VERSION 3.8)
project(openarm_follower)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
