cmake_minimum_required(VERSION 3.18)

# Set project name
project(playback_motion_primitives)

# Add project executable and extra sources
add_executable(${PROJECT_NAME}
    ${PROJECT_NAME}.cpp
    motion_primitives.cpp
)

# Link shared libraries
link_directories(/usr/local/lib)

# Include dynamixel stuff
include_directories(
    /usr/local/include/dynamixel_sdk
    /usr/local/include/DynamixelHelper
)

# Link libraries to project
target_link_libraries(${PROJECT_NAME}
    DynamixelHelper
    dxl_x64_cpp # DynamixelSDK
)
