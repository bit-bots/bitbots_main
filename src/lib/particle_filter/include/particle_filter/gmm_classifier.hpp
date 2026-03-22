// #########################################################//
// #                                                       #//
// # gaussian_mixture_models  gmm_classifier.h             #//
// # Roberto Capobianco  <capobianco@dis.uniroma1.it>      #//
// #                                                       #//
// #########################################################//

#ifndef _GMM_CLASSIFIER_HPP_
#define _GMM_CLASSIFIER_HPP_

#include <Eigen/Core>
#include <memory>
#include <particle_filter/gaussian_mixture_model.hpp>
#include <vector>

namespace gmms {
class GMMClassifier {
 public:
  GMMClassifier() {
    delta_ = 0.01;
    max_iterations_ = 100;
    trained_ = false;
    input_size_ = 0;
  }
  GMMClassifier(const double delta, const int max_iterations) {
    delta_ = delta;
    max_iterations_ = max_iterations;
    trained_ = false;
    input_size_ = 0;
  }

  // dataset with each datapoint per row and each dimension per column
  // label must be in the last column
  void train(const Eigen::MatrixXd& dataset, bool evaluate_bic = true, int gmm_components = 10);
  std::vector<int> predict(const Eigen::MatrixXd& dataset) const;
  void load(const std::string& filename);
  void save(const std::string& filename);

  // setter & getter
  inline double delta() const { return delta_; }
  inline void setDelta(const double delta) { delta_ = delta; }
  inline int maxIterations() const { return max_iterations_; }
  inline void setMaxIterations(const int max_iterations) { max_iterations_ = max_iterations; }

 private:
  bool trained_;
  double delta_;
  int max_iterations_;
  int input_size_;
  std::vector<int> classes_;
  std::vector<std::shared_ptr<GaussianMixtureModel>> gmm_vec_;

  inline static const std::string nottrained_() { return "The model has not been trained"; }

  inline static const std::string dimensionality_mismatch_() { return "Dimensionality mismatch"; }

  inline static const std::string notsufficient_() { return "Class data size not sufficient: skipping class"; }
};
}  // namespace gmms

#endif  // _GMM_CLASSIFIER_HPP_
