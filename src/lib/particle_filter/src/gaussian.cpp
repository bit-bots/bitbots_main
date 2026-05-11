// #########################################################//
// #                                                       #//
// # gaussian_mixture_models  gaussian.cpp                 #//
// # Roberto Capobianco  <capobianco@dis.uniroma1.it>      #//
// #                                                       #//
// #########################################################//

#include <Eigen/Eigenvalues>
#include <cmath>
#include <particle_filter/gaussian.hpp>

namespace gmms {
double Gaussian::evaluate_point(const Eigen::VectorXd& pt) const {
  if (pt.size() != dimensionality_) {
    throw std::runtime_error(dimensionality_mismatch());
  }

  Eigen::VectorXd dist = pt - mean_;

  double factor = std::sqrt(std::pow(2 * M_PI, dimensionality_) * cov_abs_determinant_);
  double exp = std::exp(-0.5 * dist.transpose() * inv_covariance_ * dist);

  double result = exp / factor;

  if (result == 0) {
    srand(time(NULL));
    double random = ((double)rand() / (RAND_MAX));
    result = random * 1e-15;
  }

  return result;
}

void Gaussian::setCovariance(const Eigen::MatrixXd& covariance) {
  if (covariance.rows() != covariance.cols() || covariance.rows() != dimensionality_) {
    throw std::runtime_error(dimensionality_mismatch());
  }

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(covariance.cols());
  solver.compute(covariance);

  Eigen::VectorXd eigenvalues = solver.eigenvalues();
  Eigen::MatrixXd eigenvectors = solver.eigenvectors();

  for (int i = 0; i < eigenvalues.rows(); ++i) {
    if (eigenvalues(i) < M_EPS) {
      eigenvalues(i) = M_EPS;
    }
  }

  covariance_ = eigenvectors * eigenvalues.asDiagonal() * eigenvectors.inverse();
  double abs_determinant = std::fabs(covariance_.determinant());

  inv_covariance_ = covariance_.inverse();
  cov_abs_determinant_ = abs_determinant;
}

void Gaussian::setMeanCovariance(const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance) {
  int dimensionality = mean.size();
  dimensionality_ = dimensionality;
  setMean(mean);
  setCovariance(covariance);
}

std::string Gaussian::toString() const {
  Eigen::IOFormat mean_format(5, 0, ", ", "\n", "[", "]", "Mean:       ", "\n");
  Eigen::IOFormat cov_format(5, 0, ", ", "\n", "[", "]", "Covariance: ", "\n");
  std::stringstream gaussian_string;
  gaussian_string << "Gaussian: \n";
  gaussian_string << mean_.format(mean_format);
  gaussian_string << covariance_.format(cov_format);
  return gaussian_string.str();
}

void Gaussian::addToEigenMatrix(Eigen::MatrixXd& matrix, float x0, float y0, float x1, float y1, int stepcount) const {
  assert(x0 < x1);
  assert(y0 < y1);
  float x_delta = std::abs(x1 - x0);
  float y_delta = std::abs(y1 - y0);
  float x_mean = mean_(0, 0);
  float y_mean = mean_(1, 0);
  // TODO: save stepsize

  for (int y_step = 0; y_step < stepcount; y_step++) {
    float y = y0 + (y_delta / stepcount * y_step);
    for (int x_step = 0; x_step < stepcount; x_step++) {
      float x = x0 + (x_delta / stepcount * x_step);
      // Function taken from
      // https://en.wikipedia.org/wiki/Gaussian_function#Two-dimensional_Gaussian_function
      // matrix(y_step, x_step) += 1 * std::exp(-(std::pow(x - x_mean, 2)
      // / covariance_(0, 0)  + 2 * (x - x_mean) * (y - y_mean) /
      // covariance_(1, 0) + std::pow(y - y_mean, 2) / covariance_(1,
      // 1)));
      matrix(y_step, x_step) += 1 * std::exp(-((std::pow(x - x_mean, 2) / (2 * covariance_(0, 0))) +
                                               (std::pow(y - y_mean, 2) / (2 * covariance_(1, 1)))));
    }
  }
}

double Gaussian::calcDistance(const Eigen::VectorXd& mean) const {
  if (mean.size() != dimensionality_) {
    throw std::runtime_error(dimensionality_mismatch());
  }
  Eigen::VectorXd diff_vec = mean_ - mean;
  return diff_vec.cwiseAbs().sum() / static_cast<double>(dimensionality_);
}

}  // namespace gmms
