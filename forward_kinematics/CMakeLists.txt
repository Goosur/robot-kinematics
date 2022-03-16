cmake_minimum_required(VERSION 3.18)

# Set project name
project(record_motors)

# Add project executable
add_executable(${PROJECT_NAME}
    ${PROJECT_NAME}.cpp
    fk.cpp
)

# Link shared libraries
link_directories(/usr/local/lib)

# Include dynamixel stuff
include_directories(
    /usr/local/include/dynamixel_sdk
    /usr/local/include/DynamixelHelper
    /usr/local/include/eigen3
)

# Link libraries to project
target_link_libraries(${PROJECT_NAME}
    DynamixelHelper
    dxl_x64_cpp # DynamixelSDK
)
