#include "fk.h"

Eigen::Matrix<double, 4, 4> FK::generate_transform(double alpha, double a, double d, double theta)
{
    Eigen::Matrix<double, 4, 4> T {
        {cos(theta), -sin(theta), 0, a},
        {sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d},
        {sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d},
        {0, 0, 0, 1}
    };

    return T;
}

vector<double> FK::get_gripper_coords(int n_frames, vector<double> alphas, vector<double> as, vector<double> ds, vector<double> thetas)
{
    Eigen::Matrix<double, 4, 4> T_0N = generate_transform(alphas[0], as[0], ds[0], thetas[0]);

    for (int i = 1; i < n_frames; i++)
        T_0N = T_0N * generate_transform(alphas[i], as[i], ds[i], thetas[i]);

    return vector<double>{T_0N(0, 3), T_0N(1, 3), T_0N(2, 3)};
}
