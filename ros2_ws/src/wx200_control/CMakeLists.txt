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
find_package(geometry_msgs REQUIRED)
find_package(dynamixel_helper REQUIRED)
find_package(wx200_control_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(multi_motor_writer src/motor_pub.cpp)
ament_target_dependencies(multi_motor_writer
        rclcpp
	sensor_msgs	
        dynamixel_helper
	wx200_control_interfaces)

add_executable(state_publisher src/state_publisher.cpp)
ament_target_dependencies(state_publisher
        rclcpp
        sensor_msgs
        dynamixel_helper)

add_executable(inverse_kinematics
  src/ik_srv/ik_server.cpp
  src/ik_srv/ik.cpp
  src/ik_srv/fk.cpp)
ament_target_dependencies(inverse_kinematics
	rclcpp
	wx200_control_interfaces
	sensor_msgs
	tf2_ros
	dynamixel_helper)

install(TARGETS
        multi_motor_writer
        state_publisher
	inverse_kinematics
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch
        urdf
        rviz
        DESTINATION share/${PROJECT_NAME})

ament_package()
