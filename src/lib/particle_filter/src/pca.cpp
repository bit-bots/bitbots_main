// #########################################################//
// #                                                       #//
// # gaussian_mixture_models  pca.cpp                      #//
// # Roberto Capobianco  <capobianco@dis.uniroma1.it>      #//
// #                                                       #//
// #########################################################//

#include <Eigen/SVD>
#include <iostream>
#include <particle_filter/pca.hpp>
#include <stdexcept>

namespace gmms {
Eigen::MatrixXd PCA::pca(const Eigen::MatrixXd& dataset, double& retained_variance) {
  if (num_components_ > dataset.cols()) {
    throw std::runtime_error("Number of components greater than dataset size");
  }

  Eigen::MatrixXd centered = dataset.rowwise() - dataset.colwise().mean();

  Eigen::RowVectorXd maxValues = centered.colwise().maxCoeff();
  Eigen::RowVectorXd minValues = centered.colwise().minCoeff();
  Eigen::MatrixXd normalized_dataset = centered.array().rowwise() / (maxValues - minValues).array();

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(normalized_dataset, Eigen::ComputeThinV);

  Eigen::VectorXd S = svd.singularValues();
  Eigen::MatrixXd W = svd.matrixV().leftCols(num_components_);

  retained_variance = S.head(num_components_).sum() / S.sum();

  Eigen::MatrixXd projected = normalized_dataset * W;

  return projected;
}
}  // namespace gmms
