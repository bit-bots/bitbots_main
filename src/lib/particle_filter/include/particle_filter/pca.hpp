// #########################################################//
// #                                                       #//
// # gaussian_mixture_models  pca.h                        #//
// # Roberto Capobianco  <capobianco@dis.uniroma1.it>      #//
// #                                                       #//
// #########################################################//

#ifndef _PCA_HPP_
#define _PCA_HPP_

#include <Eigen/Core>
#include <vector>

namespace gmms {
class PCA {
 public:
  // constructors
  PCA() { num_components_ = 0; }
  explicit PCA(const int num_components) { num_components_ = num_components; }

  // setters & getters
  inline void setNumComponents(const int num_components) { num_components_ = num_components; }
  inline int numComponents() const { return num_components_; }

  // useful methods
  // dataset with each datapoint per row and each dimension per column
  inline Eigen::MatrixXd pca(const Eigen::MatrixXd& dataset) {
    double variance_retained = 0.0;
    return pca(dataset, variance_retained);
  }
  Eigen::MatrixXd pca(const Eigen::MatrixXd& dataset, double& retained_variance);

 private:
  int num_components_;
};
}  // namespace gmms

#endif  // _PCA_HPP_
