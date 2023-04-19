cmake_minimum_required(VERSION 3.8)
project(wx200_control)

set(CMAKE_CXX_STANDARD 20)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(wx200_control_interfaces REQUIRED)

add_executable(multi_motor_writer src/motor_pub.cpp)
ament_target_dependencies(multi_motor_writer
        rclcpp
        std_msgs
        dynamixel_sdk
        wx200_control_interfaces)

add_executable(state_publisher src/state_publisher.cpp)
ament_target_dependencies(state_publisher
        rclcpp
        sensor_msgs
        geometry_msgs
        dynamixel_sdk
        robot_state_publisher
        wx200_control_interfaces)

install(TARGETS
        multi_motor_writer
        state_publisher
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch
        urdf
        rviz
        DESTINATION share/${PROJECT_NAME})

ament_package()
