#ifndef FK_H
#define FK_H

#include <eigen3/Eigen/Eigen>
#include <vector>

using std::vector;

class FK {
public:
  FK(vector<double> &alpha, vector<double> &a, vector<double> &d)
      : alpha(alpha), a(a), d(d) {}

  vector<double> get_();

private:
  vector<double> alpha;
  vector<double> a;
  vector<double> d;
  // Maybe need vector<double> thetas;
};

namespace forward_kinematics {
vector<vector<double>> wx200_parameters(vector<double> thetas);
Eigen::Matrix4d dh_transform(vector<double> parameters);
vector<double> get_hand_coordinates(vector<double> angles);
} // namespace forward_kinematics

#endif
