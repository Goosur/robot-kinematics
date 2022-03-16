#ifndef FK_H
#define FK_H

#include "Eigen/Dense"

namespace forward_kinematics
{
    double** wx200_parameters(double *thetas);
    Eigen::Matrix4d dh_transform(double *parameters);
    double* get_hand_coordinates(double *angles, int angles_size);
}

#endif
