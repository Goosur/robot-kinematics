cmake_minimum_required(VERSION 3.18)
project(dynamixel_helper)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(dynamixel_helper SHARED ./src/dynamixel_helper.cpp)

set_target_properties(dynamixel_helper PROPERTIES PUBLIC_HEADER ./include/dynamixel_helper/dynamixel_helper.h)
target_link_libraries(dynamixel_helper PRIVATE dxl_x64_cpp rt)

include(GNUInstallDirs)
install(TARGETS dynamixel_helper
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/dynamixel_helper)
