#include "fk.h"

/**
 * Generate 2d array of Denavit-Hartenberg parameters from alphas, as, ds, and
 * thetas.
 */
std::array<std::array<double, 4>, 6>
FK::generate_parameters(const std::array<double, 5> &thetas) {
  return {{{0.0, 0.0, 0.0, thetas[0] - M_PI},
           {M_PI / 2.0, 0.0, 0.0, thetas[1] - M_PI / 2 - std::sin(50 / 206.16)},
           {0.0, 206.16, 0.0, thetas[2] - 3 * M_PI / 2 + std::sin(50 / 206.16)},
           {0.0, 200.0, 0.0, thetas[3] - 3 * M_PI / 2.0},
           {M_PI / 2.0, 0.0, 0.0, thetas[4] - M_PI},
           {0.0, 0.0, -174.15, 0.0}}};
}

/**
 * Takes alpha_{i-1}, a_{i-1}, d_i, and theta_i in an array
 * Creates Denavit-Hartenberg transformation matrix for give joint paramters
 * Returns transformation matrix for joint i-1 to i
 */
Eigen::Matrix4d
FK::generate_dh_transform(const std::array<double, 4> &joint_parameters) {
  Eigen::Matrix4d T;
  T << cos(joint_parameters[3]), -sin(joint_parameters[3]), 0.0,
      joint_parameters[1], sin(joint_parameters[3]) * cos(joint_parameters[0]),
      cos(joint_parameters[3]) * cos(joint_parameters[0]),
      -sin(joint_parameters[0]),
      -sin(joint_parameters[0]) * joint_parameters[2],
      sin(joint_parameters[3]) * sin(joint_parameters[0]),
      cos(joint_parameters[3]) * sin(joint_parameters[0]),
      cos(joint_parameters[0]), cos(joint_parameters[0]) * joint_parameters[2],
      0.0, 0.0, 0.0, 1.0;

  return T;
}

/**
 * Computes current 3d coordinates of end effector using current joint angles
 */
std::array<double, 3> FK::where(const std::array<double, 5> &joint_angles) {
  // Get parameters for each joint
  std::array<std::array<double, 4>, 6> parameters =
      FK::generate_parameters(joint_angles);

  // Start with transform from world to first join
  Eigen::Matrix4d T_end_effector = FK::generate_dh_transform(parameters[0]);
  // Keep building transforms onto eachother to get the transform from world to
  // end effector
  for (int i = 1; i < parameters.size(); i++)
    T_end_effector *= FK::generate_dh_transform(parameters[i]);

  // Coordinates of end effector come from the third column of the final DH
  // transform matrix
  return {T_end_effector(0, 3), T_end_effector(1, 3), T_end_effector(2, 3)};
}
