#!/usr/bin/env python3

from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange, ParameterType

class ParameterGenerator:  # TODO own file
    def __init__(self):
        self.param_cache = []

    def declare_params(self, node):
        for param in self.param_cache:
            node.declare_parameter(*param)

    def add(self, param_name, param_type=None, default=None, description=None, min=None, max=None, step=None):
        describtor = ParameterDescriptor()
        describtor.name = param_name
        if description is None:
            describtor.description = param_name
        else:
            describtor.description = description

        if param_type is None and default is not None:
            param_type = type(default)

        py2ros_param_type =  {
            None: ParameterType.PARAMETER_NOT_SET,
            bool: ParameterType.PARAMETER_BOOL,
            int: ParameterType.PARAMETER_INTEGER,
            float: ParameterType.PARAMETER_DOUBLE,
            str: ParameterType.PARAMETER_STRING
        }

        param_type = py2ros_param_type.get(param_type, param_type)

        describtor.type = param_type

        if param_type == ParameterType.PARAMETER_INTEGER:
            if step is None:
                step = 1
            if all(x is not None or isinstance(x, int) for x in [min, max, step]):
                param_range = IntegerRange()
                param_range.from_value = min
                param_range.to_value = max
                param_range.step = step
                describtor.integer_range = [param_range]

        if param_type == ParameterType.PARAMETER_DOUBLE:
            if step is None:
                step = 0.01
            if all(x is not None for x in [min, max]):
                param_range = FloatingPointRange()
                param_range.from_value = float(min)
                param_range.to_value = float(max)
                param_range.step = float(step)
                describtor.floating_point_range = [param_range]

        type2default_default = {
            ParameterType.PARAMETER_NOT_SET: 0,
            ParameterType.PARAMETER_BOOL: False,
            ParameterType.PARAMETER_INTEGER: 0,
            ParameterType.PARAMETER_DOUBLE: 0.0,
            ParameterType.PARAMETER_STRING: ""
        }

        if default is None:
            default = type2default_default[param_type]

        self.param_cache.append((param_name, default, describtor))

gen = ParameterGenerator()

##########
# Params #
##########

gen.add("vision_parallelize", bool, description="Run the neural net and the conventional part in parallel")
gen.add("vision_blind_threshold", int, description="Brightness threshold under which the vision thinks, that someone forgot the camera cap", min=0, max=765)

gen.add("ROS_audio_msg_topic", str, description="ROS topic of the audio message")
gen.add("ROS_img_msg_topic", str, description="ROS topic of the image message")
gen.add("ROS_img_msg_queue_size", int, description="ROS queue size for the image message", min=1, max=20)
gen.add("ROS_field_boundary_msg_topic", str, description="ROS topic of the field boundary message")
gen.add("ROS_ball_msg_topic", str, description="ROS topic of the ball message")
gen.add("ROS_goal_posts_msg_topic", str, description="ROS topic of the goal posts message")
gen.add("ROS_obstacle_msg_topic", str, description="ROS topic of the obstacles message")
gen.add("ROS_line_msg_topic", str, description="ROS topic of the line message")
gen.add("ROS_line_mask_msg_topic", str, description="ROS topic of the line mask message")
gen.add("ROS_debug_image_msg_topic", str, description="ROS topic of the debug image message")
gen.add("ROS_white_HSV_mask_image_msg_topic", str, description="ROS topic of the white HSV color detector mask debug image message")
gen.add("ROS_red_HSV_mask_image_msg_topic", str, description="ROS topic of the red HSV color detector mask debug image message")
gen.add("ROS_blue_HSV_mask_image_msg_topic", str, description="ROS topic of the blue HSV color detector mask debug image message")
gen.add("ROS_field_mask_image_msg_topic", str, description="ROS topic of the field mask debug image message")

gen.add("neural_network_type", str, description="The neural network type that should be used (yolo_opencv, yolo_darknet, yolo_ncs2, yolo_pytorch or dummy)")

#gen.add("yolo_darknet_model_path", str, description="Name of the yolo model")
#gen.add("yolo_openvino_model_path", str, description="Name of the yolo model")
gen.add("yoeo_model_path", str, description="Name of yoeo model")
gen.add("yoeo_nms_threshold", float, description="Yolo: Non maximum suppression threshold", min=0.0, max=1.0)
gen.add("yoeo_conf_threshold", float, description="Yolo: confidence threshold", min=0.0, max=1.0)

gen.add("ball_candidate_field_boundary_y_offset", int, description="Threshold in which ball candidates over the field boundary are allowed.", min=0, max=800)
gen.add("ball_candidate_rating_threshold", float, description="A threshold for the minimum candidate rating", min=0.0, max=1.0)
gen.add("ball_candidate_max_count", int, description="The maximum number of balls that should be published", min=0, max=50)

gen.add("goal_post_field_boundary_y_offset", int, description="Maximum distance between field boundary and goal post", min=1, max=600)

gen.add("field_color_detector_path", str, description="Color lookup table for the field color detector")

gen.add("field_color_detector_use_hsv", bool, description="Should the dummy field HSV detector be used instead")
gen.add("field_color_detector_lower_values_h", int, description="Lower bound for the field color detector hue", min=0, max=255)
gen.add("field_color_detector_lower_values_s", int, description="Lower bound for the field color detector saturation", min=0, max=255)
gen.add("field_color_detector_lower_values_v", int, description="Lower bound for the field color detector value/brightness", min=0, max=255)
gen.add("field_color_detector_upper_values_h", int, description="Upper bound for the field color detector hue", min=0, max=255)
gen.add("field_color_detector_upper_values_s", int, description="Upper bound for the field color detector saturation", min=0, max=255)
gen.add("field_color_detector_upper_values_v", int, description="Upper bound for the field color detector value/brightness", min=0, max=255)

gen.add("white_color_detector_lower_values_h", int, description="Lower bound for the white color detector hue", min=0, max=255)
gen.add("white_color_detector_lower_values_s", int, description="Lower bound for the white color detector saturation", min=0, max=255)
gen.add("white_color_detector_lower_values_v", int, description="Lower bound for the white color detector value/brightness", min=0, max=255)
gen.add("white_color_detector_upper_values_h", int, description="Upper bound for the white color detector hue", min=0, max=255)
gen.add("white_color_detector_upper_values_s", int, description="Upper bound for the white color detector saturation", min=0, max=255)
gen.add("white_color_detector_upper_values_v", int, description="Upper bound for the white color detector value/brightness", min=0, max=255)
gen.add("white_color_detector_use_color_lookup_table", bool, description="Should the white color detector use a color lookup table or a HSV range")
gen.add("white_color_detector_color_lookup_table_path", str, description="Color lookup table for the line color detector")


gen.add("red_color_detector_lower_values_h", int, description="Lower bound for the red color detector hue", min=0, max=255)
gen.add("red_color_detector_lower_values_s", int, description="Lower bound for the red color detector saturation", min=0, max=255)
gen.add("red_color_detector_lower_values_v", int, description="Lower bound for the red color detector value/brightness", min=0, max=255)
gen.add("red_color_detector_upper_values_h", int, description="Upper bound for the red color detector hue", min=0, max=255)
gen.add("red_color_detector_upper_values_s", int, description="Upper bound for the red color detector saturation", min=0, max=255)
gen.add("red_color_detector_upper_values_v", int, description="Upper bound for the red color detector value/brightness", min=0, max=255)

gen.add("blue_color_detector_lower_values_h", int, description="Lower bound for the blue color detector hue", min=0, max=255)
gen.add("blue_color_detector_lower_values_s", int, description="Lower bound for the blue color detector saturation", min=0, max=255)
gen.add("blue_color_detector_lower_values_v", int, description="Lower bound for the blue color detector value/brightness", min=0, max=255)
gen.add("blue_color_detector_upper_values_h", int, description="Upper bound for the blue color detector hue", min=0, max=255)
gen.add("blue_color_detector_upper_values_s", int, description="Upper bound for the blue color detector saturation", min=0, max=255)
gen.add("blue_color_detector_upper_values_v", int, description="Upper bound for the blue color detector value/brightness", min=0, max=255)

gen.add("field_boundary_detector_search_method", str, description="Method for finding the field boundary (iteration, reversed, downsampling_reversed, binary)")
gen.add("field_boundary_detector_vertical_steps", int, description="Number of steps on each scanline", min=1, max=480)
gen.add("field_boundary_detector_horizontal_steps", int, description="Number of scanlines", min=1, max=640)
gen.add("field_boundary_detector_roi_height", int, description="Region Of Interest height in which we are looking for green", min=1, max=100)
gen.add("field_boundary_detector_roi_width", int, description="Region Of Interest width in which we are looking for green", min=1, max=100)
gen.add("field_boundary_detector_roi_increase", float, description="Value that increases the region of interest if it is located lower in the image", min=0, max=1.0)
gen.add("field_boundary_detector_green_threshold", int, description="Threshold of green in the area covered by the kernel", min=0, max=1000)

gen.add("component_camera_cap_check_active", bool)# true
gen.add("component_ball_detection_active", bool)#: true
gen.add("component_obstacle_detection_active", bool)#: true
gen.add("component_goalpost_detection_active", bool)#: true
gen.add("component_line_detection_active", bool)#: true
gen.add("component_field_boundary_detection_active", bool)#: true
gen.add("component_field_detection_active", bool)#: false
gen.add("component_debug_image_active", bool)#: true

gen.add("obstacle_own_team_color", str, description="Team colors")
gen.add("obstacle_active", bool, description="Enables the obstacle detection")
gen.add("obstacle_finder_method", str, description="Method for the obstacle finder (distance, convex or step)")
gen.add("obstacle_color_threshold", int, description="An obstacle is defined as blue/red if it contains more blue or red than this threshold", min=0, max=255)
gen.add("obstacle_white_threshold", int, description="An obstacle that contains more white than this threshold and is not colored, is an goalpost in the conventional approach", min=0, max=255)
gen.add("obstacle_field_boundary_diff_threshold", int, description="Minimal distance between detected and convex field boundary to accept it as obstacle", min=0, max=200)
gen.add("obstacle_candidate_field_boundary_offset", int, description="Fixed height of obstacles above the field boundary", min=0, max=500)
gen.add("obstacle_candidate_min_width", int, description="Minimum width of an obstacle", min=1, max=640)
gen.add("obstacle_candidate_max_width", int, description="Maximum width of an obstacle", min=1, max=640)
gen.add("obstacle_finder_step_length", int, description="Length of an object detection step along the field boundary", min=1, max=640)
gen.add("obstacle_finder_value_increase", float, description="Factor of the impact of the height of the field boundary on the distance threshold", min=0, max=10.0)

gen.add("vision_publish_debug_image", bool, description="Publish debug image message")
#gen.add("vision_publish_HSV_mask_image", bool, description="Publish all three HSV color detector mask image messages for debug purposes")
gen.add("vision_publish_field_mask_image", bool, description="Publish field mask image message for debug purposes")
gen.add("caching", bool, description="Used to deactivate caching for profiling reasons")
