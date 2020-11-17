#include <bitbots_splines/pose_spline.h>
#include "bitbots_dynup/visualizer.h"

namespace bitbots_dynup {
Visualizer::Visualizer(const std::string &base_topic) :
    base_topic_(base_topic),
    params_() {
    /* make sure base_topic_ has consistent scheme */
    if (base_topic.compare(base_topic.size() - 1, 1, "/") != 0) {
        base_topic_ += "/";
    }

    /* create necessary publishers */
    spline_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>(base_topic_ + "received_goal",
            /* queue_size */ 5, /* latch */ true);
    }

    void Visualizer::setParams(VisualizationParams params) {
        params_ = params;
    }

    void Visualizer::displaySplines(bitbots_splines::PoseSpline splines,
                                    const std::string &frame) {
        //if (spline_publisher_.getNumSubscribers() == 0)
        //    return;
        visualization_msgs::MarkerArray path = getPath(splines, frame, params_.spline_smoothness);

        spline_publisher_.publish(path);
    }
}