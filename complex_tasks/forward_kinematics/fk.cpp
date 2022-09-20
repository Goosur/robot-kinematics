#include "fk.h"
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <iostream>

///
// Takes in angles in radians of each motor and returns the
// DH parameters for that motor (motors 1, [2 and 3], 4, 5, 6)
///
vector<vector<double>>
forward_kinematics::wx200_parameters(vector<double> thetas) {
  vector<vector<double>> parameters{
      {0.0, 0.0, 113.25, thetas[0]},
      {M_PI / 2, 0.0, 0.0, thetas[1] - M_PI / 2},
      {0.0, 200.0, 0.0, M_PI / 2},
      {0.0, 50.0, 0.0, thetas[2] - M_PI},
      {0.0, 200.0, 0.0, thetas[3] - M_PI / 2},
      {M_PI / 2, 0.0, 0.0, thetas[4]},
      {0.0, 0.0, 174.15, 0.0},
  };

  return parameters;
}

///
// Takes in alpha i-1, a i-1, d i, and theta i and
// returns the transform for joint i-1 to i
//
// parameters = {alpha, a, d, theta}
///
Eigen::Matrix4d forward_kinematics::dh_transform(vector<double> parameters) {
  Eigen::Matrix4d T;
  T << cos(parameters[3]), -sin(parameters[3]), 0.0, parameters[1],
      sin(parameters[3]) * cos(parameters[0]),
      cos(parameters[3]) * cos(parameters[0]), -sin(parameters[0]),
      -sin(parameters[0]) * parameters[2],
      sin(parameters[3]) * sin(parameters[0]),
      cos(parameters[3]) * sin(parameters[0]), cos(parameters[0]),
      cos(parameters[0]) * parameters[2], 0.0, 0.0, 0.0, 1.0;

  return T;
}

///
// Takes in theta of every motor in radians and
// returns the xyz coordinates of the end effector.
///
vector<double> forward_kinematics::get_hand_coordinates(vector<double> angles) {
  // Get the parameters for each joint
  vector<vector<double>> parameters =
      forward_kinematics::wx200_parameters(angles);

  // Start with transform from world to first joint
  Eigen::Matrix4d T_end_effector =
      forward_kinematics::dh_transform(parameters[0]);
  for (int i = 1; i < 7; i++) {
    // Keep building transforms onto eachother like the book
    // to get the transform from world to end effector
    T_end_effector *= forward_kinematics::dh_transform(parameters[i]);
  }

  // Grab xyz from the final transformation matrix
  vector<double> hand_coordinates{T_end_effector(0, 3), T_end_effector(1, 3),
                                  T_end_effector(2, 3)};

  return hand_coordinates;
}
