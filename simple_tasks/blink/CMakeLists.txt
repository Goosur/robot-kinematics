cmake_minimum_required(VERSION 3.18)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Copy compile commands for vim
add_custom_target(
    copy-compile-commands ALL
    ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_BINARY_DIR}/compile_commands.json
        ${CMAKE_CURRENT_LIST_DIR}
    )

# Set project name
project(blink)

# Add project executable
add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)

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
