#ifndef FK_H
#define FK_H

#include <eigen3/Eigen/Dense>
#include <vector>

using std::vector;

namespace FK {
Eigen::Matrix<double, 4, 4> generate_transform(double alpha, double a, double d,
                                               double theta);
vector<double> get_gripper_coords(int n_frames, vector<double> alphas,
                                  vector<double> as, vector<double> ds,
                                  vector<double> thetas);
} // namespace FK

#endif
