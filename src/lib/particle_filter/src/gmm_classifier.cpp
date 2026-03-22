// #########################################################//
// #                                                       #//
// # gaussian_mixture_models  gmm_classifier.cpp           #//
// # Roberto Capobianco  <capobianco@dis.uniroma1.it>      #//
// #                                                       #//
// #########################################################//

#include <omp.h>

#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <particle_filter/gmm_classifier.hpp>
#include <particle_filter/matrix_io.hpp>
#include <set>
#include <stdexcept>
#include <vector>

namespace gmms {
void GMMClassifier::train(const Eigen::MatrixXd& dataset, bool evaluate_bic, int gmm_components) {
  Eigen::VectorXi labels = dataset.rightCols(1).cast<int>();
  Eigen::MatrixXd training_data = dataset.leftCols(dataset.cols() - 1);
  std::set<int> classes(labels.data(), labels.data() + labels.size());
  classes_.clear();
  classes_.insert(classes_.begin(), classes.begin(), classes.end());

  gmm_vec_.clear();
  gmm_vec_.resize(classes_.size());

  input_size_ = training_data.cols();
  int class_idx = 0;

  for (std::vector<int>::iterator it = classes_.begin(); it != classes_.end(); ++it) {
    Eigen::MatrixXd class_data((labels.array() == *it).count(), training_data.cols());

    std::cout << "Training class number " << class_idx << ": " << *it << std::endl;
    std::cout << "\t\tClass data size: " << class_data.rows() << std::endl;

    if (class_data.rows() < 10) {
      std::cerr << notsufficient_() << std::endl;
      gmm_vec_[class_idx] = nullptr;
      continue;
    }

    int class_data_idx = 0;

    for (long data_idx = 0; data_idx < labels.size(); ++data_idx) {
      if (labels(data_idx) == *it) {
        class_data.row(class_data_idx++) = training_data.row(data_idx);
      }
    }

    int num_components;
    double bic = 0.0;
    bool first_trial = true;

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
      model->initialize(class_data);
      model->setNumIterations(max_iterations_);
      model->setDelta(delta_);

      try {
        model->expectationMaximization(class_data);
      } catch (const std::runtime_error& e) {
        std::cout << "\t\t\t\t" << c << " components are not usable" << std::endl;
        break;
      }

      double trial_bic = model->bayesianInformationCriterion(class_data);
      std::cout << "\t\t\t\tTrial BIC: " << trial_bic << std::endl;

      if (first_trial || trial_bic < bic) {
        bic = trial_bic;
        gmm_vec_[class_idx] = model;
        first_trial = false;
      }
    }

    std::cout << "\t\tFinal class model with " << gmm_vec_[class_idx]->numComponents() << "; ";
    std::cout << "BIC " << bic << std::endl;
    ++class_idx;
  }

  trained_ = true;
}

std::vector<int> GMMClassifier::predict(const Eigen::MatrixXd& dataset) const {
  if (!trained_) {
    throw std::runtime_error(nottrained_());
  }

  if (dataset.cols() != input_size_) {
    throw std::runtime_error(dimensionality_mismatch_());
  }

  int dataset_size = dataset.rows();
  std::vector<int> assignments(dataset_size);

#pragma omp parallel for
  for (int i = 0; i < dataset_size; ++i) {
    bool first = true;
    double log_likelihood = 0.0;

    for (size_t k = 0; k < gmm_vec_.size(); ++k) {
      if (gmm_vec_[k] == nullptr) {
        continue;
      }

      double class_log_likelihood = gmm_vec_[k]->logLikelihood(dataset.row(i));

      if (first || log_likelihood < class_log_likelihood) {
        assignments[i] = classes_[k];
        log_likelihood = class_log_likelihood;
        first = false;
      }
    }
  }

  return assignments;
}

void GMMClassifier::load(const std::string& filename) {
  MatrixIO mio;
  Eigen::MatrixXd model;

  mio.readFromFile(filename, model);

  trained_ = model(0, 0);
  delta_ = model(1, 0);
  max_iterations_ = model(2, 0);
  input_size_ = model(3, 0);

  classes_.clear();
  gmm_vec_.clear();
  int idx = 4;

  while (idx < model.rows() - 1) {
    classes_.push_back(model(idx, 0));
    int rows = model(idx + 1, 0);
    gmm_vec_.push_back(std::shared_ptr<GaussianMixtureModel>(new GaussianMixtureModel));
    gmm_vec_[gmm_vec_.size() - 1]->load(model.block(idx + 2, 0, rows, model.cols()));

    idx += rows + 2;
  }
}

void GMMClassifier::save(const std::string& filename) {
  MatrixIO mio;
  int rows = 0;
  int cols = 0;
  std::vector<Eigen::MatrixXd> gmm_models;

  for (size_t k = 0; k < gmm_vec_.size(); ++k) {
    if (gmm_vec_[k] == nullptr) {
      continue;
    }

    gmm_models.push_back(gmm_vec_[k]->save());
    rows += gmm_models[k].rows();

    if (cols == 0) {
      cols = gmm_models[k].cols();
    }
  }

  // gmm model + classes + num rows per classes
  Eigen::MatrixXd model = Eigen::MatrixXd::Zero(rows + 2 * classes_.size() + 4, cols);

  model(0, 0) = trained_;
  model(1, 0) = delta_;
  model(2, 0) = max_iterations_;
  model(3, 0) = input_size_;

  int idx = 4;

  for (size_t k = 0; k < gmm_vec_.size(); ++k) {
    if (gmm_vec_[k] == nullptr) {
      continue;
    }

    model(idx, 0) = classes_[k];
    model(idx + 1, 0) = gmm_models[k].rows();
    model.block(idx + 2, 0, gmm_models[k].rows(), gmm_models[k].cols()) = gmm_models[k];
    idx += gmm_models[k].rows() + 2;
  }

  mio.writeToFile(filename, model);
}
}  // namespace gmms
