/**
 * @file gmm.cpp
 * @author Caitrin Eaton (ceaton@ncf.edu)
 * @brief Implements a GMM class in support of motion primitives.
 * @version 0.1
 * @date 2022-03-05
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "gmm.h"

GMM::GMM(int k) {
  this->k = k;
  this->gaussians.resize(k);
  this->offset = 0;

  // distribute gaussians across the X axis, range [0.0, 1.0]
  double mean = 0.0;
  double std = 1.0 / ((double)k - 1.0);
  double weight = 1.0 / pow(2.0 * M_PI * pow(std, 2), 0.5);
  for (int i = 0; i < k; i++) {
    this->gaussians[i].set_weight(weight);
    this->gaussians[i].set_mean(mean);
    this->gaussians[i].set_std(std);
    mean += std;
  }
}

GMM::GMM(vector<Gaussian> gaussians, double offset) {
  this->gaussians = gaussians;
  this->offset = offset;
  this->k = gaussians.size();
}

// ACCESSORS

/** @brief return ,a pointer to the vector of all k gaussian kernels
 *	@return vector<Gaussian> of all k gaussian kernels
 */
vector<Gaussian> GMM::get_gaussians() const { return this->gaussians; }

/** @brief return a pointer to the i-th gaussian kernel
 *   @param i (int), index of a specific kernel within the GMM
 *	@return Gaussian at index i in the GMM
 */
Gaussian GMM::get_gaussian(int i) const { return this->gaussians[i]; }

/** @brief return a vector of all k gaussian kernels' weights
 *	@return vector<double> weights of all k gaussian kernels
 */
vector<double> GMM::get_weights() const {
  vector<double> weights(this->k);
  for (int i = 0; i < this->k; i++)
    weights[i] = gaussians[i].get_weight();
  return weights;
}

/** @brief return a vector of all k gaussian kernels' means
 *	@return vector<double> means of all gaussian kernels in the GMM
 */
vector<double> GMM::get_means() const {
  vector<double> means(this->k);
  for (int i = 0; i < this->k; i++)
    means[i] = gaussians[i].get_mean();
  return means;
}

/** @brief return a vector of all k gaussian kernels' stds
 *	@return vector<double> stds of all k gaussian kernels
 */
vector<double> GMM::get_stds() const {
  vector<double> stds(this->k);
  for (int i = 0; i < this->k; i++)
    stds[i] = gaussians[i].get_std();
  return stds;
}

/**
 * @brief return the offset
 * @return double, y-intercept of the GMM model
 */
double GMM::get_offset() const { return this->offset; }

/**
 *  @brief return the y-value for a given x coordinate
 *  @param x (double) point along the horizontal axis to plug into the gaussian
 * equation
 *  @return y (double) height of the gaussian at the given x-coordinate
 */
double GMM::at(double x) {
  double y = this->offset;
  for (int i = 0; i < this->k; i++)
    y += this->gaussians[i].at(x);
  return y;
}

/**
 * @brief return the number of gaussian kernels in the model
 * @return k (int) the number of gaussian kernels in the model
 */
int GMM::get_k() const { return this->k; }

// MUTATORS

/** @brief change the means
 *  @param means, reference to a vector<double> of the gaussian kernels' new
 * centroids
 */
void GMM::set_means(const vector<double> &means) {
  for (int i = 0; i < this->k; i++)
    this->gaussians[i].set_mean(means[i]);
}

/** @brief change the stds
 *  @param stds, reference to a vector<double> of the gaussian kernels' new
 * widths
 */
void GMM::set_stds(const vector<double> &stds) {
  for (int i = 0; i < this->k; i++)
    this->gaussians[i].set_std(stds[i]);
}

/** @brief change the weights
 *  @param weights, reference to a vector<double> of the gaussian kernels' new
 * heights
 */
void GMM::set_weights(const vector<double> &weights) {
  for (int i = 0; i < this->k; i++)
    this->gaussians[i].set_weight(weights[i]);
}

/** @brief change the y-intercept
 *  @param y (double) the GMM's new y-intercept
 */
void GMM::set_offset(double y) { this->offset = y; }

// UTILITIES

/** @brief display gaussian characteristics in the terminal */
void GMM::to_string() {
  string w_str = "\tweights\t= [ ";
  string mean_str = "\tmeans\t= [ ";
  string std_str = "\tstds\t= [ ";

  for (int i = 0; i < this->k; i++) {
    w_str += std::to_string(this->gaussians[i].get_weight());
    mean_str += std::to_string(this->gaussians[i].get_mean());
    std_str += std::to_string(this->gaussians[i].get_std());

    if (i + 1 < this->k) {
      w_str += ", ";
      mean_str += ", ";
      std_str += ", ";
    } else {
      w_str += " ]\n";
      mean_str += " ]\n";
      std_str += " ]\n";
    }
  }

  printf("\nGMM object at location %p\n", this);
  printf("%s", w_str.c_str());
  printf("%s", mean_str.c_str());
  printf("%s", std_str.c_str());
  printf("\toffset\t= %.3f\n", this->offset);
  printf("\tgmm(x)\t= %.3f + Σ[i=1:%d]( w[i] * exp( -(x - mean[i])^2 / (2 * "
         "std[i]^2) )\n",
         this->offset, this->k);
}
