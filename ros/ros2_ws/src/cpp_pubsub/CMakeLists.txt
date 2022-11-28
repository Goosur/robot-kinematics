cmake_minimum_required(VERSION 3.8)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(cpp_pubsub)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add clang specific compile options if clang being used
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add publisher executable to project
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

# Add subscriber executable to project
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

# Add executables to project installer
install(TARGETS
    talker
    listener
    DESTINATION lib/${PROJECT_NAME})

ament_package()
