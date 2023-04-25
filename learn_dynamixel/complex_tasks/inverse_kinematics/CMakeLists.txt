add_executable(inverse_kinematics
    ./src/main.cpp
    ./src/ik.cpp
    ./src/fk.cpp)
target_include_directories(inverse_kinematics
    PUBLIC ./include/)
find_package(Eigen3 REQUIRED NO_MODULE)
target_link_libraries(inverse_kinematics
    Eigen3::Eigen)
