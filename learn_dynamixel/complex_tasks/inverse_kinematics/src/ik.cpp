#include "ik.h"
#include "fk.h"

/**
 * Pre-generated Jacobian transform for the widowx 200 robot arm
 */
Matrix5d IK::generate_jacobian(std::array<double, 5> thetas) {
  Matrix5d J{
      {1.0 *
           (-174.15 * cos(thetas[1] + thetas[2] + thetas[3]) +
            200 * cos(thetas[1] + thetas[2]) +
            206.16 * sin(thetas[1] - 0.240159419698206)) *
           sin(thetas[0]),
       1.0 *
           (-174.15 * sin(thetas[1] + thetas[2] + thetas[3]) +
            200 * sin(thetas[1] + thetas[2]) -
            206.16 * cos(thetas[1] - 0.240159419698206)) *
           cos(thetas[0]),
       1.0 *
           (-174.15 * sin(thetas[1] + thetas[2] + thetas[3]) +
            200 * sin(thetas[1] + thetas[2])) *
           cos(thetas[0]),
       1.0 * -174.15 * sin(thetas[1] + thetas[2] + thetas[3]) * cos(thetas[0]),
       0},
      {-1.0 *
           (-174.15 * cos(thetas[1] + thetas[2] + thetas[3]) +
            200 * cos(thetas[1] + thetas[2]) +
            206.16 * sin(thetas[1] - 0.240159419698206)) *
           cos(thetas[0]),
       1.0 *
           (-174.15 * sin(thetas[1] + thetas[2] + thetas[3]) +
            200 * sin(thetas[1] + thetas[2]) -
            206.16 * cos(thetas[1] - 0.240159419698206)) *
           sin(thetas[0]),
       1.0 *
           (-174.15 * sin(thetas[1] + thetas[2] + thetas[3]) +
            200 * sin(thetas[1] + thetas[2])) *
           sin(thetas[0]),
       1.0 * -174.15 * sin(thetas[0]) * sin(thetas[1] + thetas[2] + thetas[3]),
       0},
      {0,
       1.0 * -174.15 * cos(thetas[1] + thetas[2] + thetas[3]) +
           1.0 * 200 * cos(thetas[1] + thetas[2]) +
           1.0 * 206.16 * sin(thetas[1] - 0.240159419698206),
       1.0 * -174.15 * cos(thetas[1] + thetas[2] + thetas[3]) +
           1.0 * 200 * cos(thetas[1] + thetas[2]),
       1.0 * -174.15 * cos(thetas[1] + thetas[2] + thetas[3]), 0},
      {0, 0, 0, 0, 1},
      {0, 1, 1, 1, 0}};

  return J;
}

std::array<double, 5> IK::get_next_thetas(std::array<double, 5> current_thetas,
                                          std::array<double, 5> goal_pose) {
  Vector5d new_thetas =
      Vector5d::Map(current_thetas.begin(), current_thetas.size());

  // Max change we should see in pose.
  Vector5d x_max{0.01, 0.01, 0.01, 0.01, 0.01};
  // x_max *= 10;

  // Get current gripper position to help compute pose (location and orientation
  // of hand; x, y, z, roll, pitch).
  // Adjust joint angles to make them work with DH
  std::array<double, 3> gripper_position = FK::where(current_thetas);
  Vector5d current_pose{gripper_position[0], gripper_position[1],
                        gripper_position[2], current_thetas[4] - M_PI,
                        current_thetas[1] + current_thetas[2] +
                            current_thetas[3] - 3 * M_PI};

  // # STEP 1 - Determine how far we are from goal.
  Vector5d total_linear_change =
      Vector5d::Map(goal_pose.begin(), goal_pose.size()) - current_pose;

  // # STEP 2 - Determine a reasonable small amount to move.
  Vector5d current_linear_change =
      (total_linear_change / total_linear_change.norm()).cwiseProduct(x_max);

  // # STEP 3 - Generate the jacobian for this time step and see if the
  // determinant is reasonable (if it is too small it will eventually reach a
  // singularity).
  Matrix5d J = generate_jacobian(current_thetas);
  double J_det = J.determinant();

  // If a singularity isn't encountered update the 2
  if (abs(J_det) > 1) {
    Vector5d rotational_change = J.inverse() * current_linear_change;
    new_thetas += rotational_change;
  }

  std::array<double, 5> result;
  std::copy(new_thetas.data(), new_thetas.data() + new_thetas.size(), result.begin());

  return result;
}
