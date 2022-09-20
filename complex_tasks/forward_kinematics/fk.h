#ifndef FK_H
#define FK_H

#include <vector>
#include <eigen3/Eigen/Eigen>

using std::vector;

namespace forward_kinematics {
vector<vector<double>> wx200_parameters(vector<double> thetas);
Eigen::Matrix4d dh_transform(vector<double> parameters);
vector<double> get_hand_coordinates(vector<double> angles);
} // namespace forward_kinematics

#endif
