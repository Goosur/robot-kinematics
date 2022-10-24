#ifndef FK_H
#define FK_H

#include <array>
#include <eigen3/Eigen/Eigen>

using namespace std;

class FK {
public:
  FK(const vector<double> &alpha, const vector<double> &a,
     const vector<double> &d)
      : alpha(alpha), a(a), d(d) {}
  array<double, 3>
  get_end_effector_coordinates(const vector<double> &joint_angles);

private:
  vector<double> alpha;
  vector<double> a;
  vector<double> d;

  vector<array<double, 4>> generate_parameters(const vector<double> &thetas);
  Eigen::Matrix4d
  generate_dh_transform(const array<double, 4> &joint_parameters);
};

#endif
