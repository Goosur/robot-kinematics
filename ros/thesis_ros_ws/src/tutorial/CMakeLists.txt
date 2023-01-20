cmake_minimum_required(VERSION 3.0.2)
project(tutorial)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_custom_target(
  copy-compile-commands ALL
  ${CMAKE_COMMAND} -E copy_if_different
  ${CMAKE_BINARY_DIR}/compile_commands.json ${CMAKE_CURRENT_LIST_DIR})

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs
                                        message_generation)

# Declare messages and services in the 'msg' and 'srv' folders
add_message_files(FILES Num.msg)
add_service_files(FILES AddTwoInts.srv)

# Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

# Delcare a catkin package
catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs message_runtime)

include_directories(${catkin_INCLUDE_DIRS})

# Add talker executable
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker ${PROJECT_NAME}_generate_messages_cpp)

# Add listener executable
add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener ${PROJECT_NAME}_generate_messages_cpp)

# Add add two ints service server executable
add_executable(add_two_ints_server src/add_two_ints_server.cpp)
target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
add_dependencies(add_two_ints_server ${PROJECT_NAME}_gencpp)

# Add add two ints client server executable
add_executable(add_two_ints_client src/add_two_ints_client.cpp)
target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
add_dependencies(add_two_ints_client ${PROJECT_NAME}_gencpp)

catkin_install_python(
  PROGRAMS
  scripts/talker.py
  scripts/listener.py
  scripts/add_two_ints_server.py
  scripts/add_two_ints_client.py
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION})
