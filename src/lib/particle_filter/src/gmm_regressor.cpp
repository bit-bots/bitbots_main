// #########################################################//
// #                                                       #//
// # gaussian_mixture_models  gmm_regressor.cpp            #//
// # Roberto Capobianco  <capobianco@dis.uniroma1.it>      #//
// #                                                       #//
// #########################################################//

#include <omp.h>

#include <Eigen/Core>
#include <iostream>
#include <particle_filter/gmm_regressor.hpp>
#include <particle_filter/matrix_io.hpp>
#include <set>
#include <stdexcept>
#include <vector>

namespace gmms {
void GMMRegressor::train(const Eigen::MatrixXd& dataset, bool evaluate_bic, int gmm_components) {
  int num_components;
  double bic = 0.0;
  bool first_trial = true;
  input_size_ = dataset.cols();

  std::cout << "Training regressor" << std::endl;
  std::cout << "\t\tEvaluate BIC: ";

  if (evaluate_bic) {
    std::cout << "ON" << std::endl;
    num_components = 1;
  } else {
    std::cout << "OFF" << std::endl;
    num_components = gmm_components;
  }

  for (int c = num_components; c <= gmm_components; ++c) {
    std::cout << "\t\t\t\tUsing " << c << " GMM components" << std::endl;

    std::shared_ptr<GaussianMixtureModel> model(new GaussianMixtureModel);
    model->setNumComponents(c);
    model->initialize(dataset);
    model->setNumIterations(max_iterations_);
    model->setDelta(delta_);

    try {
      model->expectationMaximization(dataset);
    } catch (const std::runtime_error& e) {
      std::cout << "\t\t\t\t" << c << " components are not usable" << std::endl;
      break;
    }

    double trial_bic = model->bayesianInformationCriterion(dataset);

    std::cout << "\t\t\t\tTrial BIC: " << trial_bic << std::endl;

    if (first_trial || trial_bic < bic) {
      bic = trial_bic;
      gmm_ = model;
      first_trial = false;
    }
  }

  std::cout << "\t\tFinal model with " << gmm_->numComponents() << "; ";
  std::cout << "BIC " << bic << std::endl;

  trained_ = true;
}

Eigen::MatrixXd GMMRegressor::predict(const Eigen::MatrixXd& dataset, const Eigen::VectorXi& output_indices) const {
  if (!trained_ || gmm_ == nullptr) {
    throw std::runtime_error(nottrained_());
  }

  int query_size = dataset.cols();
  int target_size = output_indices.size();

  if (query_size + target_size != input_size_) {
    throw std::runtime_error(notconsistent_());
  }

  if (target_size == 0) {
    throw std::runtime_error(notvalid_());
  }

  int dataset_size = dataset.rows();
  int num_components = gmm_->numComponents();
  Eigen::VectorXi input_indices(query_size);

  int idx = 0;
  for (int i = 0; i < input_size_; ++i) {
    if (!(output_indices.array() == i).any()) {
      input_indices(idx++) = i;
    }
  }

  Eigen::VectorXi indices(input_size_);
  indices.head(query_size) = input_indices;
  indices.tail(target_size) = output_indices;
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, int> P = indices.asPermutation().transpose();

  Eigen::MatrixXd regressions = Eigen::MatrixXd::Zero(dataset_size, target_size);

  std::vector<Gaussian> reduced_gaussians(num_components);
  std::vector<double> normalization_factor(dataset_size, 0.0);

  for (int k = 0; k < num_components; ++k) {
    Eigen::VectorXd mean_k = gmm_->component(k).mean();
    Eigen::MatrixXd covariance_k = gmm_->component(k).covariance();

    mean_k = P * mean_k;
    covariance_k = P * covariance_k * P.transpose();

    Eigen::VectorXd query_mean = mean_k.head(query_size);
    Eigen::VectorXd target_mean = mean_k.tail(target_size);
    Eigen::MatrixXd query_covariance = covariance_k.topLeftCorner(query_size, query_size);
    Eigen::MatrixXd target_covariance = covariance_k.bottomRightCorner(target_size, target_size);
    Eigen::MatrixXd query_target_covariance = covariance_k.topRightCorner(query_size, target_size);
    Eigen::MatrixXd target_query_covariance = query_target_covariance.transpose();
    Eigen::MatrixXd inv_query_covariance = query_covariance.inverse();

    reduced_gaussians[k].setMeanCovariance(query_mean, query_covariance);

    Eigen::MatrixXd conditional_covariance =
        target_covariance - target_query_covariance * inv_query_covariance * query_target_covariance;

#pragma omp parallel for
    for (int i = 0; i < dataset_size; ++i) {
      Eigen::VectorXd query = dataset.row(i);
      double query_probability = reduced_gaussians[k].evaluate_point(query);

      Eigen::VectorXd tmp(target_size);
      tmp = target_mean + target_query_covariance * inv_query_covariance * (query - query_mean);
      tmp *= query_probability;

      regressions.row(i) += tmp;
      normalization_factor[i] += query_probability;

      if (k == num_components - 1) {
        regressions.row(i) /= normalization_factor[i];
      }
    }
  }

  return regressions;
}

void GMMRegressor::load(const std::string& filename) {
  MatrixIO mio;
  Eigen::MatrixXd model;

  mio.readFromFile(filename, model);

  trained_ = model(0, 0);
  delta_ = model(1, 0);
  max_iterations_ = model(2, 0);
  input_size_ = model(3, 0);
  gmm_ = std::shared_ptr<GaussianMixtureModel>(new GaussianMixtureModel);
  gmm_->load(model.block(4, 0, model.rows() - 4, model.cols()));
}

void GMMRegressor::save(const std::string& filename) {
  MatrixIO mio;
  Eigen::MatrixXd gmm_model = gmm_->save();
  Eigen::MatrixXd model = Eigen::MatrixXd::Zero(gmm_model.rows() + 4, gmm_model.cols());

  model(0, 0) = trained_;
  model(1, 0) = delta_;
  model(2, 0) = max_iterations_;
  model(3, 0) = input_size_;
  model.block(4, 0, gmm_model.rows(), gmm_model.cols()) = gmm_model;
  mio.writeToFile(filename, model);
}
}  // namespace gmms
