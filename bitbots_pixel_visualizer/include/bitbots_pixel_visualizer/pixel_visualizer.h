#ifndef PIXEL_VISUALIZER
#define PIXEL_VISUALIZER

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <dynamic_reconfigure/server.h>

#include <bitbots_pixel_visualizer/PixelVisualizerConfig.h>
#include <humanoid_league_msgs/PixelsRelative.h>
#include <humanoid_league_msgs/PixelRelative.h>


class PixelVisualizer {

    public:

        PixelVisualizer();

        // callbacks for ros stuff
        void dynamic_reconfigure_callback(bitbots_pixel_visualizer::PixelVisualizerConfig &config, uint32_t level);
        void pixels_callback(const humanoid_league_msgs::PixelsRelative &msg);


    private:

        ros::NodeHandle nh_;

        ros::Subscriber pixels_subscriber_;

        ros::Publisher marker_publisher_;

        bitbots_pixel_visualizer::PixelVisualizerConfig config_;

        // transformer methods
        visualization_msgs::Marker pixels_to_marker(const humanoid_league_msgs::PixelsRelative &pixels, std::string n_space);
};

#endif
