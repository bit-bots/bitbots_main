// #########################################################//
// #                                                       #//
// # gaussian_mixture_models  gaussian.h                   #//
// # Roberto Capobianco  <capobianco@dis.uniroma1.it>      #//
// #                                                       #//
// #########################################################//

#ifndef _GAUSSIAN_HPP_
#define _GAUSSIAN_HPP_

#include <math.h>
#include <stdlib.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <iostream>
#include <particle_filter/defs.hpp>
#include <stdexcept>
#include <string>

namespace gmms {
class Gaussian {
 public:
  // constructors
  Gaussian() {
    dimensionality_ = 0;
    cov_abs_determinant_ = 0.0;
  }
  Gaussian(const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance) { setMeanCovariance(mean, covariance); }

  // useful functions
  double evaluate_point(const Eigen::VectorXd& pt) const;

  // setter & getter
  inline int dimensionality() const { return dimensionality_; }
  inline void setMean(const Eigen::VectorXd& mean) {
    if (mean.size() != dimensionality_) {
      throw std::runtime_error(dimensionality_mismatch());
    }

    mean_ = mean;
  }
  inline Eigen::VectorXd mean() const { return mean_; }
  void setCovariance(const Eigen::MatrixXd& covariance);
  inline Eigen::MatrixXd covariance() const { return covariance_; }
  void setMeanCovariance(const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance);
  std::string toString() const;
  void addToEigenMatrix(Eigen::MatrixXd& matrix, float x0, float y0, float x1, float y1, int stepcount) const;

  double calcDistance(const Eigen::VectorXd& mean) const;

 private:
  int dimensionality_;
  Eigen::VectorXd mean_;
  Eigen::MatrixXd covariance_;
  Eigen::MatrixXd inv_covariance_;
  double cov_abs_determinant_;

  inline static const std::string dimensionality_mismatch() { return "Dimensionality mismatch"; }

  inline static const std::string covariance_not_invertible() { return "Covariance matrix is not invertible"; }
};
}  // namespace gmms

#endif  // _GAUSSIAN_HPP_
