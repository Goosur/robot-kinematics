#ifndef IK_H
#define IK_H

#include "fk.h"
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/src/Core/Matrix.h>

using namespace std;

typedef Eigen::Matrix<double, 5, 5> Matrix5d;
typedef Eigen::Vector<double, 5> Vector5d;

namespace IK {
/**
 * @brief Jacobian matrix for wx200 arm.
 * @param thetas [theta1, theta2, theta4, theta5, theta6]
 * @return Resulting jacobian matrix.
 */
Matrix5d generate_jacobian(const vector<double> &thetas);
vector<double> get_next_thetas(const vector<double> &current_thetas,
                               const Vector5d &goal_thetas, FK &fk);
}; // namespace IK

#endif
