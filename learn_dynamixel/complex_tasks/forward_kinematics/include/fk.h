#ifndef FK_H
#define FK_H

#include <array>
#include <eigen3/Eigen/Eigen>
#include <cmath>
#include <vector>

class FK {
public:
  FK(const std::vector<double> &alpha, const std::vector<double> &a,
     const std::vector<double> &d)
      : alpha(alpha), a(a), d(d) {}

  std::array<double, 3>
  get_end_effector_coordinates(const std::vector<double> &joint_angles);

private:
  std::vector<double> alpha;
  std::vector<double> a;
  std::vector<double> d;

  std::vector<std::array<double, 4>> generate_parameters(const std::vector<double> &thetas);
  Eigen::Matrix4d
  generate_dh_transform(const std::array<double, 4> &joint_parameters);
};

#endif
