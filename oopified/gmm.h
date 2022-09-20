/**
 * @file gmm.h
 * @author Caitrin Eaton (ceaton@ncf.edu)
 * @brief Interface for a Gaussian Mixture Model (GMM) class in support of
 * motion primitives.
 * @version 0.1
 * @date 2022-03-05
 *
 * @copyright Copyright (c) 2022
 */
#ifndef GMM_H
#define GMM_H

#include "gaussian.h"
#include <string>
#include <vector>

using std::string;
using std::vector;

/**
 * @brief GMM representation using vector of gaussians.
 */
class GMM : public BasisFunction {
public:
  // constructors
  explicit GMM(int k);
  GMM(vector<Gaussian> gaussians, double offset);

  // accessors
  vector<Gaussian> get_gaussians() const;
  Gaussian get_gaussian(int i) const;
  double at(double x) override;
  vector<double> get_weights() const;
  vector<double> get_means() const;
  vector<double> get_stds() const;
  double get_offset() const;
  int get_k() const;

  // mutators
  void set_means(const vector<double> &means);
  void set_stds(const vector<double> &stds);
  void set_weights(const vector<double> &weights);
  void set_offset(double y);

  // utilities
  void to_string() override;

private:
  vector<Gaussian> gaussians;
  double offset; // weight of GMM
  int k;
};

#endif
