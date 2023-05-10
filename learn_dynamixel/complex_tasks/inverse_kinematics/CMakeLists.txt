cmake_minimum_required(VERSION 3.18)
project(forward_kinematics)

find_package(Eigen3 REQUIRED NO_MODULE)

add_executable(inverse_kinematics
    ./src/main.cpp
    ./src/ik.cpp
    ./src/fk.cpp)
target_link_libraries(inverse_kinematics
    dynamixel_helper
    Eigen3::Eigen)
