#include <utility>

#include <libPF/CRandomNumberGenerator.h>
#include <libPF/StateDistribution.h>
#include <bitbots_world_model/ObstacleStates.h>

class LocalObstacleStateDistribution : public libPF::StateDistribution<ObstacleState> {
  public:
      LocalObstacleStateDistribution(libPF::CRandomNumberGenerator &random_number_generator, std::pair<double, double> initial_robot_pose, std::pair<double, double> field_size);
      // LocalObstacleStateDistribution(libPF::CRandomNumberGenerator &random_number_generator, std::pair<double, double> initial_robot_pose, std::pair<double, double> field_size, double obstacle_min_width, double obstacle_max_width);
      ~LocalObstacleStateDistribution(){};

      const ObstacleState draw() const;

 private:
      libPF::CRandomNumberGenerator random_number_generator_;
      double min_x_;
      double max_x_;
      double min_y_;
      double max_y_;
};


class LocalObstacleStateWDistribution : public libPF::StateDistribution<ObstacleStateW> {
  public:
      LocalObstacleStateWDistribution(libPF::CRandomNumberGenerator &random_number_generator, std::pair<double, double> initial_robot_pose, std::pair<double, double> field_size, double obstacle_min_width, double obstacle_max_width);
      ~LocalObstacleStateWDistribution(){};

      const ObstacleStateW draw() const;

 private:
      libPF::CRandomNumberGenerator random_number_generator_;
      double min_x_;
      double max_x_;
      double min_y_;
      double max_y_;
      double min_width_;
      double max_width_;
};


