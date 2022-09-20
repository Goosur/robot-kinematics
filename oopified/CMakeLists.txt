cmake_minimum_required(VERSION 3.18)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set project name
project(RobotControl)

# Add project executable
add_executable(${PROJECT_NAME}
  main.cpp
  Robot.h
  Robot.cpp
  BasisFunction.h
  gaussian.h
  gmm.h
  gmm.cpp
  MP.h
  MP.cpp
  ik.h
  fk.h
  fk.cpp)

# Link shared libraries and include headers
# Dynamixel sdk and helper installed here
link_directories(/usr/local/lib)
include_directories(/usr/local/include)

# Link libraries to project
target_link_libraries(${PROJECT_NAME}
    DynamixelHelper
    dxl_x64_cpp # DynamixelSDK
)
