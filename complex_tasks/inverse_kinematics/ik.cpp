#include "ik.h"
#include <eigen3/Eigen/src/Jacobi/Jacobi.h>

Matrix5d IK::generate_jacobian(const vector<double> &thetas) {
  Matrix5d J{
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

vector<double> IK::get_next_thetas(const vector<double> &current_thetas,
                                   const Vector5d &goal_pose, FK &fk) {
  vector<double> new_thetas(current_thetas.size());

  // Max change we should see in pose.
  Vector5d x_max{0.01, 0.01, 0.01, 0.01, 0.01};
  x_max *= 10;

  // Get current gripper position to help compute pose (location and orientation
  // of hand; x, y, z, roll, pitch).
  array<double, 3> gripper_position =
      fk.get_end_effector_coordinates(current_thetas);
  Vector5d current_pose{gripper_position[0], gripper_position[1],
                        gripper_position[2], current_thetas[4] - M_PI,
                        current_thetas[1] + current_thetas[2] +
                            current_thetas[3] - 3 * M_PI};

  // # STEP 1 - Determine how far we are from goal.
  Vector5d total_linear_change = goal_pose - current_pose;

  // # STEP 2 - Determine a reasonable small amount to move.
  Vector5d current_linear_change =
      x_max * (total_linear_change / total_linear_change.norm());

  // # STEP 3 - Generate the jacobian for this time step and see if the
  // determinant is reasonable (if it is too small it will eventually reach a
  // singularity).
  Matrix5d J = generate_jacobian(current_thetas);
  double J_det = J.determinant();

  // # CHANGED THIS TO 15000 BECAUSE DETERMINANT IS BIG WHEN IT SPAZZES
  if (abs(J_det) > 1) {
    //rotational_change = J.inverse().dot(current_linear_change);
  }
  // if abs(jdet) > 1:
  //     rotational_change = (np.linalg.inv(j) @ current_linear_change)
  //     # Update thetas to new position
  //     new_thetas = current_thetas + rotational_change
  return {};
}
