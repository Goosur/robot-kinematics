#ifndef IK_H
#define IK_H

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <vector>

using std::vector;

namespace IK {
Eigen::Matrix<double, 5, 5> jacobian(vector<double> thetas) {
  Eigen::Matrix<double, 5, 5> J{
      {(-200.0 * sin(thetas[1]) - 50.0 * cos(thetas[1]) +
        200.0 * cos(thetas[1] + thetas[2]) -
        174.15 * cos(thetas[1] + thetas[2] + thetas[3])) *
           sin(thetas[0]),
       (-50.0 * sin(thetas[1]) + 200.0 * sin(thetas[1] + thetas[2]) -
        174.15 * sin(thetas[1] + thetas[2] + thetas[3]) +
        200.0 * cos(thetas[1])) *
           cos(thetas[0]),
       (200.0 * sin(thetas[1] + thetas[2]) -
        174.15 * sin(thetas[1] + thetas[2] + thetas[3])) *
           cos(thetas[0]),
       -174.15 * sin(thetas[1] + thetas[2] + thetas[3]) * cos(thetas[0]), 0},
      {(200.0 * sin(thetas[1]) + 50.0 * cos(thetas[1]) -
        200.0 * cos(thetas[1] + thetas[2]) +
        174.15 * cos(thetas[1] + thetas[2] + thetas[3])) *
           cos(thetas[0]),
       (-50.0 * sin(thetas[1]) + 200.0 * sin(thetas[1] + thetas[2]) -
        174.15 * sin(thetas[1] + thetas[2] + thetas[3]) +
        200.0 * cos(thetas[1])) *
           sin(thetas[0]),
       (200.0 * sin(thetas[1] + thetas[2]) -
        174.15 * sin(thetas[1] + thetas[2] + thetas[3])) *
           sin(thetas[0]),
       -174.15 * sin(thetas[0]) * sin(thetas[1] + thetas[2] + thetas[3]), 0},
      {0,
       200.0 * sin(thetas[1]) + 50.0 * cos(thetas[1]) -
           200.0 * cos(thetas[1] + thetas[2]) +
           174.15 * cos(thetas[1] + thetas[2] + thetas[3]),
       -200.0 * cos(thetas[1] + thetas[2]) +
           174.15 * cos(thetas[1] + thetas[2] + thetas[3]),
       174.15 * cos(thetas[1] + thetas[2] + thetas[3]), 0},
      {0, 0, 0, 0, 1},
      {0, 1, 1, 1, 0}};

  return J;
}

vector<double> compute_dthetas(vector<double> current_thetas,
                               vector<double> current_gripper_position,
                               vector<double> goal_pose) {
  vector<double> x_max{0.01, 0.01, 0.01, 0.01, 0.01};
}
} // namespace IK

#endif
