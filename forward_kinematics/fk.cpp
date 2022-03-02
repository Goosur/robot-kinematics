#include <cmath>
#include <iostream>
#include <math.h>
#include <array>
#include "Eigen/Dense"
#include "fk.h"

///
// Takes in angles in radians of each motor and returns the
// DH parameters for that motor (motors 1, [2 and 3], 4, 5, 6)
///
Eigen::MatrixXd ForwardKinematics::wx200_parameters(double m1, double m2, double m4, double m5, double m6)
{
    Eigen::MatrixXd P(7, 4);
    P <<      0.0,   0.0, 113.25, m1,
         M_PI / 2,   0.0,    0.0, m2 - M_PI / 2,
              0.0, 200.0,    0.0, M_PI / 2,
              0.0,  50.0,    0.0, m4 - M_PI,
              0.0, 200.0,    0.0, m5 - M_PI / 2,
         M_PI / 2,   0.0,    0.0, m6,
              0.0,   0.0, 174.15, 0.0;

    return P;
}

///
// Takes in alpha i-1, a i-1, d i, and theta i and
// returns the transform for joint i-1 to i
///
Eigen::Matrix4d ForwardKinematics::dh_transform(double alpha, double a, double d, double theta)
{
    Eigen::Matrix4d T;
    T <<              cos(theta),             -sin(theta),         0.0,               a,
         sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -sin(alpha) * d,
         sin(theta) * sin(alpha), cos(theta) * sin(alpha),  cos(alpha),  cos(alpha) * d,
                             0.0,                     0.0,         0.0,             1.0; 

    return T;
}

///
// Takes in theta of every motor in radians and
// returns the xyz coordinates of the end effector.
///
std::array<double, 3> ForwardKinematics::get_hand_position(std::array<double, 5> thetas)
{
    // Get the parameters for each joint
    Eigen::MatrixXd P = ForwardKinematics::wx200_parameters(thetas[0], thetas[1], thetas[2], thetas[3], thetas[4]); 

    // Start with transform from world to first joint
    Eigen::Matrix4d T_end_effector = ForwardKinematics::dh_transform(P(0, 0), P(0, 1), P(0, 2), P(0, 3));
    for (int i = 1; i < P.rows(); i++)
    {
            // Keep building transforms onto eachother like the book
            // to get the transform from world to end effector
            T_end_effector *= ForwardKinematics::dh_transform(P(i, 0), P(i, 1), P(i, 2), P(i, 3));
    }

    // Grab xyz from the final transformation matrix
    std::array<double, 3> hand_position = {T_end_effector(0, 3), T_end_effector(1, 3), T_end_effector(2, 3)};

    return hand_position;
}
