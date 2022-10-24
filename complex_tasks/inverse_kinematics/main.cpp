#include "fk.h"
#include "ik.h"
#include <cmath>
#include <iostream>

int main(int argc, char **argv) {
  // Prime forward kinematics specifically for wx200 arm
  vector<double> alpha{0.0, M_PI / 2, 0.0, 0.0, 0.0, M_PI / 2, 0.0};
  vector<double> a{0.0, 0.0, 200.0, 50.0, 200.0, 0.0, 0.0};
  vector<double> d{113.25, 0.0, 0.0, 0.0, 0.0, 0.0, 174.15};
  FK wx200(alpha, a, d);

  // Thetas
  vector<double> initial_thetas{M_PI, M_PI, M_PI, M_PI, M_PI};
  vector<double> current_thetas = initial_thetas;
  // End effector location
  array<double, 3> current_xyz =
      wx200.get_end_effector_coordinates(current_thetas);
  // End effector poses
  vector<double> goal_pose{0.0, 0.0, 600.0, M_PI / 2, -M_PI / 2};
  vector<double> current_pose{
      current_xyz[0], current_xyz[1], current_xyz[2], current_thetas[4] - M_PI,
      current_thetas[1] + current_thetas[3] + current_thetas[4] - 3 * M_PI};

  // Other things
  double moving_status_threshold = 2 * 20 * 0.088 * M_PI / 180.0;

  // Wait until goal is reached
  bool still_moving;
  do {
    still_moving = false;
    for (int i = 0; i < goal_pose.size(); i++)
      still_moving |=
          abs(goal_pose[i] - current_pose[i]) >
          moving_status_threshold;
    cout << "Waiting for arm to reach goal..." << endl;
  } while (still_moving);
  cout << "Finished moving" << endl;

  return 0;
}
