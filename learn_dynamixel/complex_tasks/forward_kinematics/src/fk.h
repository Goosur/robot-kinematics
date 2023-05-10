#ifndef FK_H
#define FK_H

#include <array>
#include <eigen3/Eigen/Eigen>

namespace FK {
std::array<double, 3> where(const std::array<double, 5> &joint_angles);
std::array<std::array<double, 4>, 6>
generate_parameters(const std::array<double, 5> &thetas);
Eigen::Matrix4d
generate_dh_transform(const std::array<double, 4> &joint_parameters);
}; // namespace FK

#endif
