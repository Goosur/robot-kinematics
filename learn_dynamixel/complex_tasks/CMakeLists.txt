cmake_minimum_required(VERSION 3.18)
project(complex_tasks)

link_libraries(dynamixel_helper dxl_x64_cpp)

add_subdirectory(./motion_primitives/)
add_subdirectory(./forward_kinematics/)
add_subdirectory(./inverse_kinematics/)
