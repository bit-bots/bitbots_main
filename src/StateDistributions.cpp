#include <bitbots_world_model/StateDistributions.h>

LocalObstacleStateDistribution::LocalObstacleStateDistribution(libPF::CRandomNumberGenerator &random_number_generator, std::pair<double, double> initial_robot_pose, std::pair<double, double> field_size) : random_number_generator_(random_number_generator) {
    min_x_ = -field_size.first / 2.0 - initial_robot_pose.first;
    min_y_ = -field_size.second / 2.0 - initial_robot_pose.second;
    max_x_ = field_size.first / 2.0 - initial_robot_pose.first;
    max_y_ = field_size.second / 2.0 - initial_robot_pose.second;
}

const ObstacleState LocalObstacleStateDistribution::draw() const {
    return(ObstacleState(random_number_generator_.getUniform(min_x_, max_x_), random_number_generator_.getUniform(min_y_, max_y_)));
}

LocalObstacleStateWDistribution::LocalObstacleStateWDistribution(libPF::CRandomNumberGenerator &random_number_generator, std::pair<double, double> initial_robot_pose, std::pair<double, double> field_size, double obstacle_min_width, double obstacle_max_width) :
    random_number_generator_(random_number_generator),
    min_width_(obstacle_min_width),
    max_width_(obstacle_max_width) {
    min_x_ = -field_size.first / 2.0 - initial_robot_pose.first;
    min_y_ = -field_size.second / 2.0 - initial_robot_pose.second;
    max_x_ = field_size.first / 2.0 - initial_robot_pose.first;
    max_y_ = field_size.second / 2.0 - initial_robot_pose.second;
}

const ObstacleStateW LocalObstacleStateWDistribution::draw() const {
    return(ObstacleStateW(random_number_generator_.getUniform(min_x_, max_x_), random_number_generator_.getUniform(min_y_, max_y_), random_number_generator_.getUniform(min_width_, max_width_)));
}

