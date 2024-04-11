//
// Created by judith on 09.03.19.
//

#include <bitbots_localization/ObservationModel.hpp>

namespace bitbots_localization {

RobotPoseObservationModel::RobotPoseObservationModel(std::shared_ptr<Map> map_lines, std::shared_ptr<Map> map_goals,
                                                     std::shared_ptr<Map> map_field_boundary,
                                                     const bitbots_localization::Params &config,
                                                     rclcpp::Node::SharedPtr node)
    : particle_filter::ObservationModel<RobotState>() {
  map_lines_ = map_lines;
  map_goals_ = map_goals;
  map_field_boundary_ = map_field_boundary;
  config_ = config;
  node_ = node;
  // RCLCPP_INFO_STREAM(node_->get_logger(), "a");
  torch::Device device(torch::kCUDA);
  // RCLCPP_INFO_STREAM(node_->get_logger(), "b");
  try {
    // Deserialize the ScriptModule from a file using torch::jit::load().
    RCLCPP_INFO_STREAM(node_->get_logger(), "PATH: " << config_.misc.network_model_path);
    mask_rating_module_ = torch::jit::load(config_.misc.network_model_path, device);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "c");
  } catch (const c10::Error &e) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "error loading the mask rating model\n" << e.what());
  }

  particle_filter::ObservationModel<RobotState>::accumulate_weights_ = true;
  // RCLCPP_INFO_STREAM(node_->get_logger(), "e");
}

double RobotPoseObservationModel::calculate_weight_for_class(
    const RobotState &state, const std::vector<std::pair<double, double>> &last_measurement, std::shared_ptr<Map> map,
    double element_weight) const {
  double particle_weight_for_class;
  if (!last_measurement.empty()) {
    std::vector<double> ratings = map->Map::provideRating(state, last_measurement);
    particle_weight_for_class = std::accumulate(
        ratings.begin(), ratings.end(), 1.0,
        [element_weight](double a, double b) { return a * ((1 - element_weight) + element_weight * (b / 100)); });
  } else {
    particle_weight_for_class = 0;
  }
  return particle_weight_for_class;
}

std::vector<double> RobotPoseObservationModel::measure_bulk(
    std::vector<particle_filter::Particle<RobotState> *> particle_vector) {
  // RCLCPP_INFO_STREAM(node_->get_logger(), "measure_bulk called");
  torch::Device device(torch::kCUDA);

  std::vector<torch::jit::IValue> inputs;
  last_measurement_line_mask_ = last_measurement_line_mask_.to(at::kFloat);
  // RCLCPP_INFO_STREAM(node_->get_logger(), "last_measurement_line_mask_: " << last_measurement_line_mask_.sizes());
  last_measurement_line_mask_ = last_measurement_line_mask_.to(device);
  inputs.push_back(last_measurement_line_mask_);  //.to(device));
  torch::Tensor state_tensor;
  particle_vector[0]->getState().convertParticleListToTorchTensor(particle_vector, state_tensor, false);
  state_tensor = state_tensor.to(at::kFloat).transpose(0, 1);
  state_tensor = state_tensor.reshape({1, 8, -1});
  // RCLCPP_INFO_STREAM(node_->get_logger(), "state_tensor_: " << state_tensor);
  state_tensor = state_tensor.to(device);
  inputs.push_back(state_tensor);  //.to(device));
  // RCLCPP_INFO_STREAM(node_->get_logger(), "state_tensor: " << state_tensor.sizes());
  at::Tensor out_tensor = mask_rating_module_.forward(inputs).toTensor();  // TODO: to cpu or gpu and stuff and dtype
  // RCLCPP_INFO_STREAM(node_->get_logger(), "an");
  out_tensor = out_tensor.toType(at::kDouble).to(torch::kCPU);
  // std::vector<double> out_vector(out_tensor.data_ptr<double>(), out_tensor.data_ptr<double>() + out_tensor.numel());
  // // TODO: make sure dtype fits
  RCLCPP_ERROR_STREAM(node_->get_logger(), "raw: " << out_tensor);
  out_tensor = torch::abs(out_tensor);
  RCLCPP_ERROR_STREAM(node_->get_logger(), "abs: " << out_tensor);
  out_tensor = torch::sum(out_tensor, 1);
  RCLCPP_ERROR_STREAM(node_->get_logger(), "sum: " << out_tensor);
  out_tensor = 1 / out_tensor;
  RCLCPP_ERROR_STREAM(node_->get_logger(), "out_tensor: " << out_tensor);
  std::vector<double> out_vector =
      std::vector<double>(out_tensor.data_ptr<double>(), out_tensor.data_ptr<double>() + out_tensor.numel());
  RCLCPP_ERROR_STREAM(node_->get_logger(), "out_vector: " << out_vector);
  return out_vector;
}

double RobotPoseObservationModel::measure(const RobotState &state) const {
  double particle_weight_lines = calculate_weight_for_class(state, last_measurement_lines_, map_lines_,
                                                            config_.particle_filter.confidences.line_element);
  double particle_weight_goal = calculate_weight_for_class(state, last_measurement_goal_, map_goals_,
                                                           config_.particle_filter.confidences.goal_element);
  double particle_weight_field_boundary =
      calculate_weight_for_class(state, last_measurement_field_boundary_, map_field_boundary_,
                                 config_.particle_filter.confidences.field_boundary_element);

  // Get relevant config values
  auto scoring_config = config_.particle_filter.scoring;

  // Calculate weight for the particle
  double weight = (((1 - scoring_config.lines.factor) + scoring_config.lines.factor * particle_weight_lines) *
                   ((1 - scoring_config.goal.factor) + scoring_config.goal.factor * particle_weight_goal) *
                   ((1 - scoring_config.field_boundary.factor) +
                    scoring_config.field_boundary.factor * particle_weight_field_boundary));

  if (weight < config_.particle_filter.weighting.min_weight) {
    weight = config_.particle_filter.weighting.min_weight;
  }

  // reduce weight if particle is too far outside of the field:
  float range = config_.particle_filter.weighting.out_of_field_range;
  if (state.getXPos() > (config_.field.size.x + config_.field.padding) / 2 + range ||
      state.getXPos() < -(config_.field.size.x + config_.field.padding) / 2 - range ||
      state.getYPos() > (config_.field.size.y + config_.field.padding) / 2 + range ||
      state.getYPos() < -(config_.field.size.y + config_.field.padding) / 2 - range) {
    weight = weight - config_.particle_filter.weighting.out_of_field_weight_decrease;
  }

  return weight;  // exponential?
}

void RobotPoseObservationModel::set_measurement_line_mask(sm::msg::Image measurement) {
  // stolen from: https://github.com/klintan/ros2_pytorch/blob/master/src/ros2_pytorch.cpp
  // convert image to tensor
  std::shared_ptr<cv_bridge::CvImage> image_ = cv_bridge::toCvCopy(measurement, "8UC1");
  // RCLCPP_INFO_STREAM(node_->get_logger(), "image shape: " << image_->image.size());
  cv::Mat image;
  cv::resize(image_->image, image, cv::Size(256, 192));  // TODO change order?
  // RCLCPP_INFO_STREAM(node_->get_logger(), "image sum: " << cv::sum(image));
  at::TensorOptions options(at::ScalarType::Byte);
  std::vector<int64_t> sizes = {1, 1, 256, 192};
  at::Tensor tensor_image = torch::from_blob(image.data, at::IntList(sizes), options);
  tensor_image = tensor_image.transpose(2, 3);  // adapt to pytorch format

  last_measurement_line_mask_ = tensor_image / 255.0;
  RCLCPP_INFO_STREAM(node_->get_logger(), "image input tensor shape: " << last_measurement_line_mask_.sizes());
  RCLCPP_INFO_STREAM(node_->get_logger(), "tensor sum: " << last_measurement_line_mask_.max());
}

void RobotPoseObservationModel::set_measurement_lines_pc(sm::msg::PointCloud2 measurement) {
  for (sm::PointCloud2ConstIterator<float> iter_xyz(measurement, "x"); iter_xyz != iter_xyz.end(); ++iter_xyz) {
    std::pair<double, double> linePolar = cartesianToPolar(iter_xyz[0], iter_xyz[1]);
    last_measurement_lines_.push_back(linePolar);
  }
}

void RobotPoseObservationModel::set_measurement_goalposts(sv3dm::msg::GoalpostArray measurement) {
  // convert to polar
  for (sv3dm::msg::Goalpost &post : measurement.posts) {
    std::pair<double, double> postPolar = cartesianToPolar(post.bb.center.position.x, post.bb.center.position.y);
    last_measurement_goal_.push_back(postPolar);
  }
}

void RobotPoseObservationModel::set_measurement_field_boundary(sv3dm::msg::FieldBoundary measurement) {
  // convert to polar
  for (gm::msg::Point &point : measurement.points) {
    std::pair<double, double> fieldBoundaryPointPolar = cartesianToPolar(point.x, point.y);
    last_measurement_field_boundary_.push_back(fieldBoundaryPointPolar);
  }
}

std::vector<std::pair<double, double>> RobotPoseObservationModel::get_measurement_lines() const {
  return last_measurement_lines_;
}

torch::Tensor RobotPoseObservationModel::get_measurement_line_mask() const {
  // RCLCPP_INFO_STREAM(node_->get_logger(), "fetching line mask");
  return last_measurement_line_mask_;
}

std::vector<std::pair<double, double>> RobotPoseObservationModel::get_measurement_goals() const {
  return last_measurement_goal_;
}

std::vector<std::pair<double, double>> RobotPoseObservationModel::get_measurement_field_boundary() const {
  return last_measurement_field_boundary_;
}

double RobotPoseObservationModel::get_min_weight() const { return config_.particle_filter.weighting.min_weight; }

void RobotPoseObservationModel::clear_measurement() {
  // last_measurement_lines_.clear();
  // last_measurement_goal_.clear();
  // last_measurement_field_boundary_.clear();
  last_measurement_line_mask_ = torch::Tensor();
}

bool RobotPoseObservationModel::measurements_available() {
  bool available = last_measurement_line_mask_.numel() != 0;
  // RCLCPP_INFO_STREAM(node_->get_logger(), "measurements_available: " << available);
  // available |= !last_measurement_lines_.empty();
  // available |= !last_measurement_goal_.empty();
  // available |= !last_measurement_field_boundary_.empty();
  return available;
}
}  // namespace bitbots_localization
