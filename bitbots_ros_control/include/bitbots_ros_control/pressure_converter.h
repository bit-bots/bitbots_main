#include <fstream>
#include <iostream>
#include <numeric>
#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/emitter.h>
#include <yaml-cpp/yaml.h>
#include <std_srvs/Empty.h>
#include <bitbots_msgs/FootPressure.h>
#include <bitbots_msgs/FootScale.h>

class PressureConverter {
 private:
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::NodeHandle pnh_;
  std::vector<double> zero_, scale_;
  std::vector<std::vector<double>> previous_values_, zero_and_scale_values_;
  bool save_zero_and_scale_values_;
  int current_index_;
  int average_, scale_and_zero_average_;
  char side_;
  std::string scale_lr_, zero_lr_;
  ros::ServiceServer zero_service_, scale_service_;
  void pressureCallback(const bitbots_msgs::FootPressureConstPtr &pressure_raw);
  void resetZeroAndScaleValues();
  bool zeroCallback(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &resp);
  bool scaleCallback(bitbots_msgs::FootScaleRequest &req, bitbots_msgs::FootScaleResponse &resp);
  void collectMessages();
  void saveYAML();
 public:
  PressureConverter(ros::NodeHandle &nh, char side);
};


