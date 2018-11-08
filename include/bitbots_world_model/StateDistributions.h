#include <utility>

#include <libPF/CRandomNumberGenerator.h>
#include <libPF/StateDistribution.h>
#include <bitbots_world_model/ObstacleStates.h>

class LocalPositionStateDistribution : public libPF::StateDistribution<PositionState> {
  public:
      LocalPositionStateDistribution(libPF::CRandomNumberGenerator &random_number_generator, std::pair<double, double> initial_robot_pose, std::pair<double, double> field_size);
      // LocalPositionStateDistribution(libPF::CRandomNumberGenerator &random_number_generator, std::pair<double, double> initial_robot_pose, std::pair<double, double> field_size, double obstacle_min_width, double obstacle_max_width);
      ~LocalPositionStateDistribution(){};

      const PositionState draw() const;

 private:
      libPF::CRandomNumberGenerator random_number_generator_;
      double min_x_;
      double max_x_;
      double min_y_;
      double max_y_;
};


class LocalPositionStateWDistribution : public libPF::StateDistribution<PositionStateW> {
  public:
      LocalPositionStateWDistribution(libPF::CRandomNumberGenerator &random_number_generator, std::pair<double, double> initial_robot_pose, std::pair<double, double> field_size, double obstacle_min_width, double obstacle_max_width);
      ~LocalPositionStateWDistribution(){};

      const PositionStateW draw() const;

 private:
      libPF::CRandomNumberGenerator random_number_generator_;
      double min_x_;
      double max_x_;
      double min_y_;
      double max_y_;
      double min_width_;
      double max_width_;
};


