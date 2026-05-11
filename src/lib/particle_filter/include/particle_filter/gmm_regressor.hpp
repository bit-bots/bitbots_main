// #########################################################//
// #                                                       #//
// # gaussian_mixture_models  gmm_regressor.h              #//
// # Roberto Capobianco  <capobianco@dis.uniroma1.it>      #//
// #                                                       #//
// #########################################################//

#ifndef _GMM_REGRESSOR_HPP_
#define _GMM_REGRESSOR_HPP_

#include <Eigen/Core>
#include <memory>
#include <particle_filter/gaussian_mixture_model.hpp>
#include <string>
#include <vector>

namespace gmms {
class GMMRegressor {
 public:
  GMMRegressor() {
    delta_ = 0.01;
    max_iterations_ = 100;
    trained_ = false;
    input_size_ = 0;
  }
  GMMRegressor(const double delta, const int max_iterations) {
    delta_ = delta;
    max_iterations_ = max_iterations;
    trained_ = false;
    input_size_ = 0;
  }

  // dataset with each datapoint per row and each dimension per column
  void train(const Eigen::MatrixXd& dataset, bool evaluate_bic = true, int gmm_components = 10);
  inline Eigen::MatrixXd predict(const Eigen::MatrixXd& dataset) const {
    int diff = input_size_ - dataset.cols();
    Eigen::VectorXi indices = Eigen::VectorXi::LinSpaced(diff, dataset.cols(), input_size_ - 1);

    return predict(dataset, indices);
  }
  Eigen::MatrixXd predict(const Eigen::MatrixXd& dataset, const Eigen::VectorXi& output_indices) const;
  void load(const std::string& filename);
  void save(const std::string& filename);

  // setter & getter
  inline double delta() const { return delta_; }
  inline void setDelta(const double delta) { delta_ = delta; }
  inline int maxIterations() const { return max_iterations_; }
  inline void setMaxIterations(const int max_iterations) { max_iterations_ = max_iterations; }
  inline const std::vector<Eigen::VectorXd> gmmMeans() const { return gmm_->gaussianMeans(); }
  inline const std::vector<Eigen::MatrixXd> gmmCovariances() const { return gmm_->gaussianCovariances(); }

 private:
  bool trained_;
  double delta_;
  int max_iterations_;
  int input_size_;
  std::shared_ptr<GaussianMixtureModel> gmm_;

  inline static const std::string nottrained_() { return "The model has not been trained"; }

  inline static const std::string notconsistent_() {
    return "The requested output size is incosistent with the input data";
  }

  inline static const std::string notvalid_() { return "The requested output size is 0"; }
};
}  // namespace gmms

#endif  // _GMM_REGRESSOR_HPP_
