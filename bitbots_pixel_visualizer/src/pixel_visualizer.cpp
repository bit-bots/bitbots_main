#include <bitbots_pixel_visualizer/pixel_visualizer.h>

PixelVisualizer::PixelVisualizer() : nh_() {
    ROS_INFO_STREAM("Created Bit-Bots image transformer instance.");
}

void PixelVisualizer::pixels_callback(const humanoid_league_msgs::PixelsRelative &msg) {
    ROS_INFO_STREAM("recieved a pixel message!");
    marker_publisher_.publish(pixels_to_marker(msg, config_.pixels_namespace));
}

void PixelVisualizer::dynamic_reconfigure_callback(bitbots_pixel_visualizer::PixelVisualizerConfig &config, uint32_t level) {
    // TODO
    //
    ROS_INFO_STREAM("dynrec!");

    // updating topic names when neccessary
    if (config.pixels_topic != config_.pixels_topic) {
        pixels_subscriber_ = nh_.subscribe(config.pixels_topic.c_str(), 1, &PixelVisualizer::pixels_callback, this);
    }
    if (config.pixels_marker_topic != config_.pixels_marker_topic) {
        marker_publisher_ = nh_.advertise<visualization_msgs::Marker>(config.pixels_marker_topic.c_str(), 1);
    }

    config_ = config;
}

visualization_msgs::Marker PixelVisualizer::pixels_to_marker(const humanoid_league_msgs::PixelsRelative &pixels, std::string n_space) {
    visualization_msgs::Marker pixel_marker;
    pixel_marker.header = pixels.header;
    pixel_marker.ns = n_space;
    pixel_marker.type = visualization_msgs::Marker::POINTS;
    pixel_marker.action = visualization_msgs::Marker::ADD;
    pixel_marker.scale.x = 0.05;
    pixel_marker.scale.y = 0.05;
    pixel_marker.scale.z = 0.05;
    pixel_marker.lifetime = ros::Duration(1); // TODO: evaluate this
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    for (humanoid_league_msgs::PixelRelative pixel : pixels.pixels) {
        pixel_marker.points.push_back(pixel.position);
        // color depending on the value between 0 and 1
        color.r = pixel.value;
        color.g = pixel.value;
        color.b = pixel.value;
        pixel_marker.colors.push_back(color);
    }
    return pixel_marker;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "pixel_visualizer");

    PixelVisualizer pixel_visualizer;

    // dynamic reconfigure
    dynamic_reconfigure::Server<bitbots_pixel_visualizer::PixelVisualizerConfig> dynamic_reconfigure_server;
    dynamic_reconfigure::Server<bitbots_pixel_visualizer::PixelVisualizerConfig>::CallbackType f = boost::bind(&PixelVisualizer::dynamic_reconfigure_callback, &pixel_visualizer, _1, _2);
    dynamic_reconfigure_server.setCallback(f); // automatically calls the callback once


    ros::spin();
    return 0;
}

