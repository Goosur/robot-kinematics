cmake_minimum_required(VERSION 3.18)

project(DynamixelHelper)

include(GNUInstallDirs)

add_library(DynamixelHelper SHARED dynamixel_helper.cpp)
set_target_properties(DynamixelHelper PROPERTIES PUBLIC_HEADER dynamixel_helper.h)

# Link dynamixel sdk
include_directories(/usr/local/include/dynamixel_sdk)
target_link_libraries(DynamixelHelper
    dxl_x64_cpp
    rt
)

# Install to system (/usr/local/lib and /usr/local/include)
install(
    TARGETS DynamixelHelper
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
