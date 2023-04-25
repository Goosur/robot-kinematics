cmake_minimum_required(VERSION 3.8)
project(dynamixel_helper)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)

include_directories(include)
add_library(dynamixel_helper src/dynamixel_helper.cpp)
ament_target_dependencies(dynamixel_helper dynamixel_sdk)
ament_export_targets(dynamixel_helper HAS_LIBRARY_TARGET)

install(
    DIRECTORY include/dynamixel_helper
    DESTINATION include
)

install(
    TARGETS dynamixel_helper
    EXPORT dynamixel_helper
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

ament_package()
