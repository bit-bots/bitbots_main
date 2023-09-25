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

gen.add("component_ball_detection_active", bool, description="Activate/Deactivate the ball detection component")
gen.add("component_camera_cap_check_active", bool, description="Activate/Deactivate the camera cap check component")
gen.add("component_debug_image_active", bool, description="Activate/Deactivate the debug image component")
gen.add("component_field_detection_active", bool, description="Activate/Deactivate the field detection component")
gen.add("component_goalpost_detection_active", bool, description="Activate/Deactivate the goalpost detection component")
gen.add("component_line_detection_active", bool, description="Activate/Deactivate the line detection component")
gen.add("component_obstacle_detection_active", bool, description="Activate/Deactivate the obstacle detection component")

gen.add("vision_blind_threshold", int, description="Brightness threshold under which the vision thinks, that someone forgot the camera cap", min=0, max=765)

gen.add("ROS_audio_msg_topic", str, description="ROS topic of the audio message")
gen.add("ROS_img_msg_topic", str, description="ROS topic of the image message")
gen.add("ROS_ball_msg_topic", str, description="ROS topic of the ball message")
gen.add("ROS_goal_posts_msg_topic", str, description="ROS topic of the goal posts message")
gen.add("ROS_obstacle_msg_topic", str, description="ROS topic of the obstacles message")
gen.add("ROS_line_msg_topic", str, description="ROS topic of the line message")
gen.add("ROS_line_mask_msg_topic", str, description="ROS topic of the line mask message")
gen.add("ROS_debug_image_msg_topic", str, description="ROS topic of the debug image message")
gen.add("ROS_field_mask_image_msg_topic", str, description="ROS topic of the field mask debug image message")

gen.add("yoeo_model_path", str, description="Name of YOEO model")
gen.add("yoeo_nms_threshold", float, description="YOEO Non-maximum suppression threshold", min=0.0, max=1.0)
gen.add("yoeo_conf_threshold", float, description="YOEO confidence threshold", min=0.0, max=1.0)
gen.add("yoeo_framework", str, description="The neural network framework that should be used ['pytorch', 'openvino', 'onnx', 'tvm']")

gen.add("ball_candidate_rating_threshold", float, description="A threshold for the minimum candidate rating", min=0.0, max=1.0)
gen.add("ball_candidate_max_count", int, description="The maximum number of balls that should be published", min=0, max=50)

gen.add("red_color_detector_h_zero_crossing", bool, description="Whether or not the red color detector interval includes the hue zero-crossing")
gen.add("red_color_detector_lower_values_h", int, description="Lower bound for the red color detector hue", min=0, max=255)
gen.add("red_color_detector_lower_values_s", int, description="Lower bound for the red color detector saturation", min=0, max=255)
gen.add("red_color_detector_lower_values_v", int, description="Lower bound for the red color detector value/brightness", min=0, max=255)
gen.add("red_color_detector_upper_values_h", int, description="Upper bound for the red color detector hue", min=0, max=255)
gen.add("red_color_detector_upper_values_s", int, description="Upper bound for the red color detector saturation", min=0, max=255)
gen.add("red_color_detector_upper_values_v", int, description="Upper bound for the red color detector value/brightness", min=0, max=255)

gen.add("blue_color_detector_h_zero_crossing", bool, description="Whether or not the blue color detector interval includes the hue zero-crossing")
gen.add("blue_color_detector_lower_values_h", int, description="Lower bound for the blue color detector hue", min=0, max=255)
gen.add("blue_color_detector_lower_values_s", int, description="Lower bound for the blue color detector saturation", min=0, max=255)
gen.add("blue_color_detector_lower_values_v", int, description="Lower bound for the blue color detector value/brightness", min=0, max=255)
gen.add("blue_color_detector_upper_values_h", int, description="Upper bound for the blue color detector hue", min=0, max=255)
gen.add("blue_color_detector_upper_values_s", int, description="Upper bound for the blue color detector saturation", min=0, max=255)
gen.add("blue_color_detector_upper_values_v", int, description="Upper bound for the blue color detector value/brightness", min=0, max=255)

gen.add("obstacle_finder_method", str, description="Method for the obstacle finder (distance, convex or step)")
gen.add("obstacle_color_threshold", int, description="An obstacle is defined as blue/red if it contains more blue or red than this threshold (amount of pixels in per mille)", min=0, max=255)
gen.add("obstacle_white_threshold", int, description="An obstacle that contains more white than this threshold and is not colored, is an goalpost in the conventional approach", min=0, max=255)
gen.add("obstacle_candidate_min_width", int, description="Minimum width of an obstacle", min=1, max=640)
gen.add("obstacle_candidate_max_width", int, description="Maximum width of an obstacle", min=1, max=640)
gen.add("obstacle_finder_step_length", int, description="Length of an object detection step along the field boundary", min=1, max=640)
gen.add("obstacle_finder_value_increase", float, description="Factor of the impact of the height of the field boundary on the distance threshold", min=0, max=10.0)

gen.add("caching", bool, description="Used to deactivate caching for profiling reasons")
