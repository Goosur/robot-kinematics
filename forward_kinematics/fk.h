#ifndef FK_H
#define FK_H

#include "Eigen/Dense"
#include <array>

class ForwardKinematics {
    public:
        static Eigen::MatrixXd wx200_parameters(double m1, double m2, double m4, double m5, double m6);
        static Eigen::Matrix4d dh_transform(double alpha, double a, double d, double theta);
        static std::array<double, 3> get_hand_position(std::array<double, 5> thetas);
};

#endif
