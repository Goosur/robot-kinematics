cmake_minimum_required(VERSION 3.18)
project(forward_kinematics)

find_package(Eigen3 REQUIRED NO_MODULE)

add_executable(record_hand_coordinates
    ./src/main.cpp
    ./src/fk.cpp)
target_link_libraries(record_hand_coordinates
    dynamixel_helper
    Eigen3::Eigen)
