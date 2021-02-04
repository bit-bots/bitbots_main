//
// Created by judith on 09.03.19.
//

#include "../include/bitbots_localization/StateDistribution.h"

RobotStateDistribution::RobotStateDistribution(particle_filter::CRandomNumberGenerator &random_number_generator,
                                               std::pair<double, double> initial_robot_pose,
                                               std::pair<double, double> field_size) : random_number_generator_(
        random_number_generator) {
    //around robot
    min_x_ = -field_size.first / 2.0 - initial_robot_pose.first; //length
    min_y_ = -field_size.second / 2.0 - initial_robot_pose.second; //width
    max_x_ = field_size.first / 2.0 - initial_robot_pose.first;
    max_y_ = field_size.second / 2.0 - initial_robot_pose.second;

}

const RobotState RobotStateDistribution::draw() const {

    return (RobotState(random_number_generator_.getUniform(min_x_, max_x_),
                       random_number_generator_.getUniform(min_y_, max_y_),
                       random_number_generator_.getUniform(-M_PI, M_PI)));

}

RobotStateDistributionStartLeft::RobotStateDistributionStartLeft(
        particle_filter::CRandomNumberGenerator &random_number_generator,
        std::pair<double, double> initial_robot_pose1,
        double initial_theta1,
        std::pair<double, double> initial_robot_pose2,
        double initial_theta2) {

    initial_robot_pose_1 = initial_robot_pose1;
    initial_robot_pose_2 = initial_robot_pose2;
    initial_theta_1 = initial_theta1;
    initial_theta_2 = initial_theta2;

}

const RobotState RobotStateDistributionStartLeft::draw() const {
    if (random_number_generator_.getUniform(0, 1) > 0.5) {
        return (RobotState(random_number_generator_.getGaussian(1) + initial_robot_pose_1.first,
                           random_number_generator_.getGaussian(0.1) + initial_robot_pose_1.second,
                           random_number_generator_.getGaussian(0.1) + initial_theta_1));
    } else {
        return (RobotState(random_number_generator_.getGaussian(1) + initial_robot_pose_2.first,
                           random_number_generator_.getGaussian(0.1) + initial_robot_pose_2.second,
                           random_number_generator_.getGaussian(0.1) + initial_theta_2));
    }
}

RobotStateDistributionStartRight::RobotStateDistributionStartRight(
        particle_filter::CRandomNumberGenerator &random_number_generator,
        std::pair<double, double> initial_robot_pose1,
        double initial_theta1,
        std::pair<double, double> initial_robot_pose2,
        double initial_theta2) {

    initial_robot_pose_1 = initial_robot_pose1;
    initial_robot_pose_2 = initial_robot_pose2;
    initial_theta_1 = initial_theta1;
    initial_theta_2 = initial_theta2;

}

const RobotState RobotStateDistributionStartRight::draw() const {
    if (random_number_generator_.getUniform(0, 1) > 0.5) {
        return (RobotState(random_number_generator_.getGaussian(0.8) - initial_robot_pose_1.first,
                           random_number_generator_.getGaussian(0.1) + initial_robot_pose_1.second,
                           random_number_generator_.getGaussian(0.1) + initial_theta_1));
    } else {
        return (RobotState(random_number_generator_.getGaussian(0.8) - initial_robot_pose_2.first,
                           random_number_generator_.getGaussian(0.1) + initial_robot_pose_2.second,
                           random_number_generator_.getGaussian(0.1) + initial_theta_2));
    }
}

RobotStateDistributionLeftHalf::RobotStateDistributionLeftHalf(particle_filter::CRandomNumberGenerator &random_number_generator,
                                                               std::pair<double, double> field_size) : random_number_generator_(
        random_number_generator) {
    // only own half
    min_x_ = (field_size.first / 2.0) + 0.5;
    min_y_ = (-field_size.second / 2.0) - 0.5;
    max_x_ = 0 - 0.5;
    max_y_ = (field_size.second / 2.0) + 0.5;


}

const RobotState RobotStateDistributionLeftHalf::draw() const {
    return (RobotState(random_number_generator_.getUniform(min_x_, max_x_),
                       random_number_generator_.getUniform(min_y_, max_y_),
                       random_number_generator_.getUniform(-M_PI, M_PI)));
}

RobotStateDistributionRightHalf::RobotStateDistributionRightHalf(particle_filter::CRandomNumberGenerator &random_number_generator,
                                               std::pair<double, double> field_size) : random_number_generator_(
        random_number_generator) {

    // only own half
    min_x_ = -field_size.first / 2.0;
    min_y_ = -field_size.second / 2.0;
    max_x_ = 0;
    max_y_ = field_size.second / 2.0;
}

const RobotState RobotStateDistributionRightHalf::draw() const {
    return (RobotState(random_number_generator_.getUniform(min_x_, max_x_),
                       random_number_generator_.getUniform(min_y_, max_y_),
                       random_number_generator_.getUniform(-M_PI, M_PI)));

}

RobotStateDistributionPosition::RobotStateDistributionPosition(
        particle_filter::CRandomNumberGenerator &random_number_generator,
        double x,
        double y) {

    x_ = x;
    y_ = y;

}

const RobotState RobotStateDistributionPosition::draw() const {

    return (RobotState(random_number_generator_.getGaussian(0.1) + x_,
                       random_number_generator_.getGaussian(0.1) + y_,
                       random_number_generator_.getUniform(-M_PI, M_PI)));
}

RobotStateDistributionPose::RobotStateDistributionPose(
        particle_filter::CRandomNumberGenerator &random_number_generator,
        double x,
        double y,
        double t) {

    x_ = x;
    y_ = y;
    t_ = t;

}

const RobotState RobotStateDistributionPose::draw() const {

    return RobotState(random_number_generator_.getGaussian(0.1) + x_,
                       random_number_generator_.getGaussian(0.1) + y_,
                       random_number_generator_.getGaussian(0.1) + t_);
}





