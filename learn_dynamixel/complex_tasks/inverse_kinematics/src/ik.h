#ifndef IK_H
#define IK_H

#include <array>
#include <eigen3/Eigen/Eigen>

typedef Eigen::Matrix<double, 5, 5> Matrix5d;
typedef Eigen::Vector<double, 5> Vector5d;

namespace IK {
/**
 * @brief Jacobian matrix for wx200 arm.
 * @param thetas [theta1, theta2, theta3, theta4, theta5]
 * @return Resulting jacobian matrix.
 */
Matrix5d generate_jacobian(std::array<double, 5> thetas);
std::array<double, 5> get_next_thetas(std::array<double, 5> current_thetas,
                                      std::array<double, 5> goal_thetas);
}; // namespace IK

#endif
