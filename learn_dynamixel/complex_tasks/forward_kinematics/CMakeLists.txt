add_executable(forward_kinematics
    ./src/record_hand_coordinates.cpp
    ./src/fk.cpp)
target_include_directories(forward_kinematics
    PUBLIC ./include/)
find_package(Eigen3 REQUIRED NO_MODULE)
target_link_libraries(forward_kinematics
    Eigen3::Eigen)
