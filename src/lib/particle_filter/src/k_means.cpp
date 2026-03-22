// #########################################################//
// #                                                       #//
// # gaussian_mixture_models  k_means.cpp                  #//
// # Roberto Capobianco  <capobianco@dis.uniroma1.it>      #//
// #                                                       #//
// #########################################################//

#include <cstdlib>
#include <ctime>
#include <iostream>
#include <limits>
#include <particle_filter/k_means.hpp>
#include <set>
#include <stdexcept>

namespace gmms {
// dataset with each datapoint per row
void KMeans::cluster(const Eigen::MatrixXd& dataset) {
  if (num_clusters_ == 0) {
    throw std::runtime_error("Number of clusters not set");
  }

  forgyInitialization(dataset);

  do {
    assign(dataset);
    update(dataset);
  } while (!termination_flag_);

  computeCovariances(dataset);
}

void KMeans::forgyInitialization(const Eigen::MatrixXd& dataset) {
  srand(time(NULL));
  int dataset_size = dataset.rows();
  assignments_.assign(dataset_size, 0);

  std::set<int> assigned_datapoint_idx;

  for (int i = 0; i < num_clusters_; ++i) {
    int random_datapoint_idx;

    do {
      random_datapoint_idx = rand() % dataset_size;
    } while (assigned_datapoint_idx.find(random_datapoint_idx) != assigned_datapoint_idx.end());

    means_[i] = dataset.row(random_datapoint_idx).transpose();
    assigned_datapoint_idx.insert(random_datapoint_idx);
  }
}

void KMeans::assign(const Eigen::MatrixXd& dataset) {
  termination_flag_ = true;

  for (int i = 0; i < dataset.rows(); ++i) {
    double min_distance = std::numeric_limits<int>::max();
    int assignment = 0;

    for (int j = 0; j < num_clusters_; ++j) {
      double distance = (dataset.row(i).transpose() - means_[j]).squaredNorm();

      if (distance < min_distance) {
        min_distance = distance;
        assignment = j;
      }
    }

    if (assignments_[i] != assignment) {
      assignments_[i] = assignment;
      termination_flag_ = false;
    }
  }
}

void KMeans::update(const Eigen::MatrixXd& dataset) {
  int dataset_size = dataset.rows();

  for (int i = 0; i < num_clusters_; ++i) {
    Eigen::VectorXd point_sum;
    cluster_cardinalities_[i] = 0;

    for (int j = 0; j < dataset_size; ++j) {
      if (assignments_[j] == i) {
        if (cluster_cardinalities_[i] == 0) {
          point_sum = dataset.row(j).transpose();
        } else {
          point_sum += dataset.row(j).transpose();
        }

        ++cluster_cardinalities_[i];
      }
    }

    if (cluster_cardinalities_[i] == 0) {
      int random_datapoint_idx = rand() % dataset_size;
      assignments_[random_datapoint_idx] = i;
      point_sum = dataset.row(random_datapoint_idx).transpose();
      cluster_cardinalities_[i] = 1;
    }

    means_[i] = point_sum / cluster_cardinalities_[i];
  }
}

void KMeans::computeCovariances(const Eigen::MatrixXd& dataset) {
  for (int i = 0; i < num_clusters_; ++i) {
    covariances_[i].setZero(dataset.cols(), dataset.cols());

    for (int j = 0; j < dataset.rows(); ++j) {
      if (assignments_[j] == i) {
        Eigen::VectorXd distance_vector = dataset.row(j).transpose() - means_[i];
        covariances_[i] += distance_vector * distance_vector.transpose();
      }
    }

    covariances_[i] = covariances_[i] / (cluster_cardinalities_[i]);
  }
}
}  // namespace gmms
