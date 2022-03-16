#ifndef FK_H
#define FK_H

#include "Eigen"

namespace forward_kinematics
{
    double** wx200_parameters(double *thetas);
    Eigen::Matrix4d dh_transform(double *parameters);
    double* get_hand_coordinates(double *angles);
}

#endif
