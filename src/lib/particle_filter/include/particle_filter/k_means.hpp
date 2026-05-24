// #########################################################//
// #                                                       #//
// # gaussian_mixture_models  k_means.h                    #//
// # Roberto Capobianco  <capobianco@dis.uniroma1.it>      #//
// #                                                       #//
// #########################################################//

#ifndef _K_MEANS_HPP_
#define _K_MEANS_HPP_

#include <Eigen/Core>
#include <vector>

namespace gmms {
class KMeans {
 public:
  // constructors
  KMeans() {
    num_clusters_ = 0;
    termination_flag_ = false;
  }
  explicit KMeans(const int num_clusters) {
    setNumClusters(num_clusters);
    termination_flag_ = false;
  }

  // dataset with each datapoint per row and each dimension per column
  void cluster(const Eigen::MatrixXd& dataset);

  // setter & getter
  inline int numClusters() const { return num_clusters_; }
  inline void setNumClusters(const int num_clusters) {
    num_clusters_ = num_clusters;
    means_.resize(num_clusters_);
    covariances_.resize(num_clusters_);
    cluster_cardinalities_.assign(num_clusters_, 0);
  }
  inline std::vector<Eigen::VectorXd> means() const { return means_; }
  inline std::vector<Eigen::MatrixXd> covariances() const { return covariances_; }
  inline std::vector<int> assignments() const { return assignments_; }
  inline std::vector<int> clusterCardinalities() const { return cluster_cardinalities_; }

 private:
  bool termination_flag_;
  int num_clusters_;
  std::vector<Eigen::VectorXd> means_;
  std::vector<Eigen::MatrixXd> covariances_;
  std::vector<int> assignments_;
  std::vector<int> cluster_cardinalities_;

  void forgyInitialization(const Eigen::MatrixXd& dataset);
  void assign(const Eigen::MatrixXd& dataset);
  void update(const Eigen::MatrixXd& dataset);
  void computeCovariances(const Eigen::MatrixXd& dataset);
};
}  // namespace gmms

#endif  // _K_MEANS_HPP_
