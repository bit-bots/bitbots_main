# flake8: noqa

# auto-generated DO NOT EDIT

from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import FloatingPointRange, IntegerRange
from rclpy.clock import Clock
from rclpy.exceptions import InvalidParameterValueException
from rclpy.time import Time
import copy
import rclpy
from generate_parameter_library_py.python_validators import ParameterValidators


class BallFilter:
    class Params:
        # for detecting if the parameter struct has been updated
        stamp_ = Time()

        ball_subscribe_topic = "balls_relative"
        ball_position_publish_topic = "ball_position_relative_filtered"
        ball_movement_publish_topic = "ball_relative_movement"
        ball_publish_topic = "ball_relative_filtered"
        ball_filter_reset_service_name = "ball_filter_reset"
        filter_frame = "map"
        filter_rate = 62
        velocity_reduction = 0.6
        process_noise_variance = 0.001
        measurement_certainty = 0.1
        filter_reset_time = 20
        filter_reset_distance = 2
        closest_distance_match = True

    class ParamListener:
        def __init__(self, node, prefix=""):
            self.prefix_ = prefix
            self.params_ = BallFilter.Params()
            self.node_ = node
            self.logger_ = rclpy.logging.get_logger("BallFilter." + prefix)

            self.declare_params()

            self.node_.add_on_set_parameters_callback(self.update)
            self.clock_ = Clock()

        def get_params(self):
            tmp = self.params_.stamp_
            self.params_.stamp_ = None
            paramCopy = copy.deepcopy(self.params_)
            paramCopy.stamp_ = tmp
            self.params_.stamp_ = tmp
            return paramCopy

        def is_old(self, other_param):
            return self.params_.stamp_ != other_param.stamp_

        def refresh_dynamic_parameters(self):
            updated_params = self.get_params()
            # TODO remove any destroyed dynamic parameters

            # declare any new dynamic parameters

        def update(self, parameters):
            updated_params = self.get_params()

            for param in parameters:
                if param.name == self.prefix_ + "ball_subscribe_topic":
                    updated_params.ball_subscribe_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "ball_position_publish_topic":
                    updated_params.ball_position_publish_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "ball_movement_publish_topic":
                    updated_params.ball_movement_publish_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "ball_publish_topic":
                    updated_params.ball_publish_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "ball_filter_reset_service_name":
                    updated_params.ball_filter_reset_service_name = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "filter_frame":
                    validation_result = ParameterValidators.one_of(param, ["map", "odom"])
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.filter_frame = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "filter_rate":
                    validation_result = ParameterValidators.bounds(param, 0, 100)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.filter_rate = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "velocity_reduction":
                    validation_result = ParameterValidators.bounds(param, 0, 1)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.velocity_reduction = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "process_noise_variance":
                    validation_result = ParameterValidators.bounds(param, 0, 1)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.process_noise_variance = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "measurement_certainty":
                    validation_result = ParameterValidators.bounds(param, 0, 1)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.measurement_certainty = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "filter_reset_time":
                    validation_result = ParameterValidators.bounds(param, 0, 100)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.filter_reset_time = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "filter_reset_distance":
                    validation_result = ParameterValidators.bounds(param, 0, 100)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.filter_reset_distance = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "closest_distance_match":
                    updated_params.closest_distance_match = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

            updated_params.stamp_ = self.clock_.now()
            self.update_internal_params(updated_params)
            return SetParametersResult(successful=True)

        def update_internal_params(self, updated_params):
            self.params_ = updated_params

        def declare_params(self):
            updated_params = self.get_params()
            # declare all parameters and give default values to non-required ones
            if not self.node_.has_parameter(self.prefix_ + "ball_subscribe_topic"):
                descriptor = ParameterDescriptor(
                    description="Topic to subscribe to for ball detections", read_only=False
                )
                parameter = updated_params.ball_subscribe_topic
                self.node_.declare_parameter(self.prefix_ + "ball_subscribe_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "ball_position_publish_topic"):
                descriptor = ParameterDescriptor(
                    description="Topic to publish the filtered ball position", read_only=False
                )
                parameter = updated_params.ball_position_publish_topic
                self.node_.declare_parameter(self.prefix_ + "ball_position_publish_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "ball_movement_publish_topic"):
                descriptor = ParameterDescriptor(
                    description="Topic to publish the filtered ball movement", read_only=False
                )
                parameter = updated_params.ball_movement_publish_topic
                self.node_.declare_parameter(self.prefix_ + "ball_movement_publish_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "ball_publish_topic"):
                descriptor = ParameterDescriptor(description="Topic to publish the filtered ball", read_only=False)
                parameter = updated_params.ball_publish_topic
                self.node_.declare_parameter(self.prefix_ + "ball_publish_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "ball_filter_reset_service_name"):
                descriptor = ParameterDescriptor(description="Service to reset the ball filter", read_only=False)
                parameter = updated_params.ball_filter_reset_service_name
                self.node_.declare_parameter(self.prefix_ + "ball_filter_reset_service_name", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "filter_frame"):
                descriptor = ParameterDescriptor(description="Frame to filter the ball in", read_only=False)
                parameter = updated_params.filter_frame
                self.node_.declare_parameter(self.prefix_ + "filter_frame", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "filter_rate"):
                descriptor = ParameterDescriptor(description="Filtering rate in Hz", read_only=True)
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 0
                descriptor.integer_range[-1].to_value = 100
                parameter = updated_params.filter_rate
                self.node_.declare_parameter(self.prefix_ + "filter_rate", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "velocity_reduction"):
                descriptor = ParameterDescriptor(
                    description="Velocity reduction (per axis) factor of the ball per second", read_only=False
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0
                descriptor.floating_point_range[-1].to_value = 1
                parameter = updated_params.velocity_reduction
                self.node_.declare_parameter(self.prefix_ + "velocity_reduction", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "process_noise_variance"):
                descriptor = ParameterDescriptor(
                    description="Noise which is added to the estimated position without new measurements",
                    read_only=False,
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0
                descriptor.floating_point_range[-1].to_value = 1
                parameter = updated_params.process_noise_variance
                self.node_.declare_parameter(self.prefix_ + "process_noise_variance", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "measurement_certainty"):
                descriptor = ParameterDescriptor(description="Ball measurement certainty in filter", read_only=False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0
                descriptor.floating_point_range[-1].to_value = 1
                parameter = updated_params.measurement_certainty
                self.node_.declare_parameter(self.prefix_ + "measurement_certainty", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "filter_reset_time"):
                descriptor = ParameterDescriptor(description="Max ball not seen time in Seconds", read_only=False)
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 0
                descriptor.integer_range[-1].to_value = 100
                parameter = updated_params.filter_reset_time
                self.node_.declare_parameter(self.prefix_ + "filter_reset_time", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "filter_reset_distance"):
                descriptor = ParameterDescriptor(
                    description="Distance to the current estimation causing a filter reset", read_only=False
                )
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 0
                descriptor.integer_range[-1].to_value = 100
                parameter = updated_params.filter_reset_distance
                self.node_.declare_parameter(self.prefix_ + "filter_reset_distance", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "closest_distance_match"):
                descriptor = ParameterDescriptor(
                    description="True if ball should be selected based on distance to filtered ball instead of highest rating",
                    read_only=False,
                )
                parameter = updated_params.closest_distance_match
                self.node_.declare_parameter(self.prefix_ + "closest_distance_match", parameter, descriptor)

            # TODO: need validation
            # get parameters and fill struct fields
            param = self.node_.get_parameter(self.prefix_ + "ball_subscribe_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.ball_subscribe_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "ball_position_publish_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.ball_position_publish_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "ball_movement_publish_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.ball_movement_publish_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "ball_publish_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.ball_publish_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "ball_filter_reset_service_name")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.ball_filter_reset_service_name = param.value
            param = self.node_.get_parameter(self.prefix_ + "filter_frame")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.one_of(param, ["map", "odom"])
            if validation_result:
                raise InvalidParameterValueException(
                    "filter_frame",
                    param.value,
                    "Invalid value set during initialization for parameter filter_frame: " + validation_result,
                )
            updated_params.filter_frame = param.value
            param = self.node_.get_parameter(self.prefix_ + "filter_rate")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0, 100)
            if validation_result:
                raise InvalidParameterValueException(
                    "filter_rate",
                    param.value,
                    "Invalid value set during initialization for parameter filter_rate: " + validation_result,
                )
            updated_params.filter_rate = param.value
            param = self.node_.get_parameter(self.prefix_ + "velocity_reduction")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0, 1)
            if validation_result:
                raise InvalidParameterValueException(
                    "velocity_reduction",
                    param.value,
                    "Invalid value set during initialization for parameter velocity_reduction: " + validation_result,
                )
            updated_params.velocity_reduction = param.value
            param = self.node_.get_parameter(self.prefix_ + "process_noise_variance")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0, 1)
            if validation_result:
                raise InvalidParameterValueException(
                    "process_noise_variance",
                    param.value,
                    "Invalid value set during initialization for parameter process_noise_variance: "
                    + validation_result,
                )
            updated_params.process_noise_variance = param.value
            param = self.node_.get_parameter(self.prefix_ + "measurement_certainty")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0, 1)
            if validation_result:
                raise InvalidParameterValueException(
                    "measurement_certainty",
                    param.value,
                    "Invalid value set during initialization for parameter measurement_certainty: " + validation_result,
                )
            updated_params.measurement_certainty = param.value
            param = self.node_.get_parameter(self.prefix_ + "filter_reset_time")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0, 100)
            if validation_result:
                raise InvalidParameterValueException(
                    "filter_reset_time",
                    param.value,
                    "Invalid value set during initialization for parameter filter_reset_time: " + validation_result,
                )
            updated_params.filter_reset_time = param.value
            param = self.node_.get_parameter(self.prefix_ + "filter_reset_distance")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0, 100)
            if validation_result:
                raise InvalidParameterValueException(
                    "filter_reset_distance",
                    param.value,
                    "Invalid value set during initialization for parameter filter_reset_distance: " + validation_result,
                )
            updated_params.filter_reset_distance = param.value
            param = self.node_.get_parameter(self.prefix_ + "closest_distance_match")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.closest_distance_match = param.value

            self.update_internal_params(updated_params)
