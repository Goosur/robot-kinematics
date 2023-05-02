cmake_minimum_required(VERSION 3.8)
project(wx200_control)

set(CMAKE_CXX_STANDARD 20)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_helper REQUIRED)
find_package(wx200_control_interfaces REQUIRED)

add_executable(multi_motor_writer src/motor_pub.cpp)
ament_target_dependencies(multi_motor_writer
        rclcpp
        wx200_control_interfaces
        dynamixel_sdk
        dynamixel_helper)

add_executable(state_publisher src/state_publisher.cpp)
ament_target_dependencies(state_publisher
        rclcpp
        sensor_msgs
        dynamixel_sdk
        dynamixel_helper)

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
