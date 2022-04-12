cmake_minimum_required(VERSION 3.18)

# Set project name
project(RobotControl)

# Add project executable
add_executable(${PROJECT_NAME}
	main.cpp
	Robot.h
	Robot.cpp
	transform.h
	transform.cpp
	gmm.h
	gmm.cpp
	MP.h
	MP.cpp
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
