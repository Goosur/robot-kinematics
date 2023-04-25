cmake_minimum_required(VERSION 3.18)
project(playback_motion_primitives)

add_executable(playback_motion_primitives
    ./src/main.cpp
    ./src/motion_primitives.cpp)
target_link_libraries(playback_motion_primitives
    dynamixel_helper
    dxl_x64_cpp)
