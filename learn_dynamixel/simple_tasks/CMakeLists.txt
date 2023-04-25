cmake_minimum_required(VERSION 3.18)
project(simple_tasks)

link_libraries(dynamixel_helper)

add_executable(blink ./src/blink.cpp)
add_executable(read_motor ./src/read_motor.cpp)
add_executable(read_motors ./src/read_motors.cpp)
add_executable(write_motor ./src/write_motor.cpp)
add_executable(write_motors ./src/write_motors.cpp)
add_executable(record_motors ./src/record_motors.cpp)
add_executable(change_settings ./src/change_settings.cpp)
add_executable(playback_recorded_motors ./src/playback_recorded_motors.cpp)

target_link_libraries(change_settings dxl_x64_cpp)
