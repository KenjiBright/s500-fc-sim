cmake_minimum_required(VERSION 3.8)
project(flight_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

set(FLIGHT_CONTROLLER_SOURCES
  src/flight_controller_node.cpp
  src/pid_controller.cpp
  src/motor_mixer.cpp
  src/imu_processor.cpp
)

add_executable(flight_controller_node ${FLIGHT_CONTROLLER_SOURCES})

ament_target_dependencies(flight_controller_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  gazebo_ros
  yaml-cpp
)

target_link_libraries(flight_controller_node ${YAML_CPP_LIBRARIES})

install(TARGETS flight_controller_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY ../../launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ../../config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ../../urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ../../worlds DESTINATION share/${PROJECT_NAME})

ament_package()
