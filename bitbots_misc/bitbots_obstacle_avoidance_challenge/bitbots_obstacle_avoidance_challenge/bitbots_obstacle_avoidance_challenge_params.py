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


class bitbots_obstacle_avoidance_challenge:
    class Params:
        # for detecting if the parameter struct has been updated
        stamp_ = Time()

        blue_lower_h = 89
        blue_upper_h = 145
        blue_lower_s = 144
        blue_upper_s = 231
        blue_lower_v = 31
        blue_upper_v = 194
        red_lower_h = 137
        red_upper_h = 179
        red_lower_s = 177
        red_upper_s = 255
        red_lower_v = 0
        red_upper_v = 255
        min_size = 40
        max_size = 1000
        debug_mode = False

    class ParamListener:
        def __init__(self, node, prefix=""):
            self.prefix_ = prefix
            self.params_ = bitbots_obstacle_avoidance_challenge.Params()
            self.node_ = node
            self.logger_ = rclpy.logging.get_logger("bitbots_obstacle_avoidance_challenge." + prefix)

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
                if param.name == self.prefix_ + "blue_lower_h":
                    validation_result = ParameterValidators.bounds(param, 0, 179)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.blue_lower_h = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "blue_upper_h":
                    validation_result = ParameterValidators.bounds(param, 0, 179)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.blue_upper_h = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "blue_lower_s":
                    validation_result = ParameterValidators.bounds(param, 0, 255)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.blue_lower_s = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "blue_upper_s":
                    validation_result = ParameterValidators.bounds(param, 0, 255)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.blue_upper_s = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "blue_lower_v":
                    validation_result = ParameterValidators.bounds(param, 0, 255)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.blue_lower_v = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "blue_upper_v":
                    validation_result = ParameterValidators.bounds(param, 0, 255)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.blue_upper_v = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "red_lower_h":
                    validation_result = ParameterValidators.bounds(param, 0, 179)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.red_lower_h = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "red_upper_h":
                    validation_result = ParameterValidators.bounds(param, 0, 179)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.red_upper_h = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "red_lower_s":
                    validation_result = ParameterValidators.bounds(param, 0, 255)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.red_lower_s = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "red_upper_s":
                    validation_result = ParameterValidators.bounds(param, 0, 255)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.red_upper_s = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "red_lower_v":
                    validation_result = ParameterValidators.bounds(param, 0, 255)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.red_lower_v = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "red_upper_v":
                    validation_result = ParameterValidators.bounds(param, 0, 255)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.red_upper_v = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "min_size":
                    validation_result = ParameterValidators.gt_eq(param, 0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.min_size = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "max_size":
                    validation_result = ParameterValidators.gt_eq(param, 0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.max_size = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "debug_mode":
                    updated_params.debug_mode = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

            updated_params.stamp_ = self.clock_.now()
            self.update_internal_params(updated_params)
            return SetParametersResult(successful=True)

        def update_internal_params(self, updated_params):
            self.params_ = updated_params

        def declare_params(self):
            updated_params = self.get_params()
            # declare all parameters and give default values to non-required ones
            if not self.node_.has_parameter(self.prefix_ + "blue_lower_h"):
                descriptor = ParameterDescriptor(
                    description="hue value of the lower boundary for blue obstacles", read_only=False
                )
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 0
                descriptor.integer_range[-1].to_value = 179
                parameter = updated_params.blue_lower_h
                self.node_.declare_parameter(self.prefix_ + "blue_lower_h", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "blue_upper_h"):
                descriptor = ParameterDescriptor(
                    description="hue value of the upper boundary for blue obstacles", read_only=False
                )
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 0
                descriptor.integer_range[-1].to_value = 179
                parameter = updated_params.blue_upper_h
                self.node_.declare_parameter(self.prefix_ + "blue_upper_h", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "blue_lower_s"):
                descriptor = ParameterDescriptor(
                    description="separation value of the lower boundary for blue obstacles", read_only=False
                )
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 0
                descriptor.integer_range[-1].to_value = 255
                parameter = updated_params.blue_lower_s
                self.node_.declare_parameter(self.prefix_ + "blue_lower_s", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "blue_upper_s"):
                descriptor = ParameterDescriptor(
                    description="separation value of the upper boundary for blue obstacles", read_only=False
                )
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 0
                descriptor.integer_range[-1].to_value = 255
                parameter = updated_params.blue_upper_s
                self.node_.declare_parameter(self.prefix_ + "blue_upper_s", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "blue_lower_v"):
                descriptor = ParameterDescriptor(
                    description="value value of the lower boundary for blue obstacles", read_only=False
                )
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 0
                descriptor.integer_range[-1].to_value = 255
                parameter = updated_params.blue_lower_v
                self.node_.declare_parameter(self.prefix_ + "blue_lower_v", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "blue_upper_v"):
                descriptor = ParameterDescriptor(
                    description="value value of the upper boundary for blue obstacles", read_only=False
                )
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 0
                descriptor.integer_range[-1].to_value = 255
                parameter = updated_params.blue_upper_v
                self.node_.declare_parameter(self.prefix_ + "blue_upper_v", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "red_lower_h"):
                descriptor = ParameterDescriptor(
                    description="hue value of the lower boundary for red obstacles", read_only=False
                )
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 0
                descriptor.integer_range[-1].to_value = 179
                parameter = updated_params.red_lower_h
                self.node_.declare_parameter(self.prefix_ + "red_lower_h", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "red_upper_h"):
                descriptor = ParameterDescriptor(
                    description="hue value of the upper boundary for red obstacles", read_only=False
                )
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 0
                descriptor.integer_range[-1].to_value = 179
                parameter = updated_params.red_upper_h
                self.node_.declare_parameter(self.prefix_ + "red_upper_h", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "red_lower_s"):
                descriptor = ParameterDescriptor(
                    description="separation value of the lower boundary for red obstacles", read_only=False
                )
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 0
                descriptor.integer_range[-1].to_value = 255
                parameter = updated_params.red_lower_s
                self.node_.declare_parameter(self.prefix_ + "red_lower_s", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "red_upper_s"):
                descriptor = ParameterDescriptor(
                    description="separation value of the upper boundary for red obstacles", read_only=False
                )
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 0
                descriptor.integer_range[-1].to_value = 255
                parameter = updated_params.red_upper_s
                self.node_.declare_parameter(self.prefix_ + "red_upper_s", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "red_lower_v"):
                descriptor = ParameterDescriptor(
                    description="value value of the lower boundary for red obstacles", read_only=False
                )
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 0
                descriptor.integer_range[-1].to_value = 255
                parameter = updated_params.red_lower_v
                self.node_.declare_parameter(self.prefix_ + "red_lower_v", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "red_upper_v"):
                descriptor = ParameterDescriptor(
                    description="value value of the upper boundary for red obstacles", read_only=False
                )
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 0
                descriptor.integer_range[-1].to_value = 255
                parameter = updated_params.red_upper_v
                self.node_.declare_parameter(self.prefix_ + "red_upper_v", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "min_size"):
                descriptor = ParameterDescriptor(
                    description="minimum size of an obstacle to be considered", read_only=False
                )
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 0
                descriptor.integer_range[-1].to_value = 2**31 - 1
                parameter = updated_params.min_size
                self.node_.declare_parameter(self.prefix_ + "min_size", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "max_size"):
                descriptor = ParameterDescriptor(
                    description="maximum size of an obstacle to be considered", read_only=False
                )
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 0
                descriptor.integer_range[-1].to_value = 2**31 - 1
                parameter = updated_params.max_size
                self.node_.declare_parameter(self.prefix_ + "max_size", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "debug_mode"):
                descriptor = ParameterDescriptor(
                    description="set true if debug images should be drawn and published", read_only=False
                )
                parameter = updated_params.debug_mode
                self.node_.declare_parameter(self.prefix_ + "debug_mode", parameter, descriptor)

            # TODO: need validation
            # get parameters and fill struct fields
            param = self.node_.get_parameter(self.prefix_ + "blue_lower_h")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0, 179)
            if validation_result:
                raise InvalidParameterValueException(
                    "blue_lower_h",
                    param.value,
                    "Invalid value set during initialization for parameter blue_lower_h: " + validation_result,
                )
            updated_params.blue_lower_h = param.value
            param = self.node_.get_parameter(self.prefix_ + "blue_upper_h")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0, 179)
            if validation_result:
                raise InvalidParameterValueException(
                    "blue_upper_h",
                    param.value,
                    "Invalid value set during initialization for parameter blue_upper_h: " + validation_result,
                )
            updated_params.blue_upper_h = param.value
            param = self.node_.get_parameter(self.prefix_ + "blue_lower_s")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0, 255)
            if validation_result:
                raise InvalidParameterValueException(
                    "blue_lower_s",
                    param.value,
                    "Invalid value set during initialization for parameter blue_lower_s: " + validation_result,
                )
            updated_params.blue_lower_s = param.value
            param = self.node_.get_parameter(self.prefix_ + "blue_upper_s")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0, 255)
            if validation_result:
                raise InvalidParameterValueException(
                    "blue_upper_s",
                    param.value,
                    "Invalid value set during initialization for parameter blue_upper_s: " + validation_result,
                )
            updated_params.blue_upper_s = param.value
            param = self.node_.get_parameter(self.prefix_ + "blue_lower_v")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0, 255)
            if validation_result:
                raise InvalidParameterValueException(
                    "blue_lower_v",
                    param.value,
                    "Invalid value set during initialization for parameter blue_lower_v: " + validation_result,
                )
            updated_params.blue_lower_v = param.value
            param = self.node_.get_parameter(self.prefix_ + "blue_upper_v")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0, 255)
            if validation_result:
                raise InvalidParameterValueException(
                    "blue_upper_v",
                    param.value,
                    "Invalid value set during initialization for parameter blue_upper_v: " + validation_result,
                )
            updated_params.blue_upper_v = param.value
            param = self.node_.get_parameter(self.prefix_ + "red_lower_h")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0, 179)
            if validation_result:
                raise InvalidParameterValueException(
                    "red_lower_h",
                    param.value,
                    "Invalid value set during initialization for parameter red_lower_h: " + validation_result,
                )
            updated_params.red_lower_h = param.value
            param = self.node_.get_parameter(self.prefix_ + "red_upper_h")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0, 179)
            if validation_result:
                raise InvalidParameterValueException(
                    "red_upper_h",
                    param.value,
                    "Invalid value set during initialization for parameter red_upper_h: " + validation_result,
                )
            updated_params.red_upper_h = param.value
            param = self.node_.get_parameter(self.prefix_ + "red_lower_s")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0, 255)
            if validation_result:
                raise InvalidParameterValueException(
                    "red_lower_s",
                    param.value,
                    "Invalid value set during initialization for parameter red_lower_s: " + validation_result,
                )
            updated_params.red_lower_s = param.value
            param = self.node_.get_parameter(self.prefix_ + "red_upper_s")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0, 255)
            if validation_result:
                raise InvalidParameterValueException(
                    "red_upper_s",
                    param.value,
                    "Invalid value set during initialization for parameter red_upper_s: " + validation_result,
                )
            updated_params.red_upper_s = param.value
            param = self.node_.get_parameter(self.prefix_ + "red_lower_v")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0, 255)
            if validation_result:
                raise InvalidParameterValueException(
                    "red_lower_v",
                    param.value,
                    "Invalid value set during initialization for parameter red_lower_v: " + validation_result,
                )
            updated_params.red_lower_v = param.value
            param = self.node_.get_parameter(self.prefix_ + "red_upper_v")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0, 255)
            if validation_result:
                raise InvalidParameterValueException(
                    "red_upper_v",
                    param.value,
                    "Invalid value set during initialization for parameter red_upper_v: " + validation_result,
                )
            updated_params.red_upper_v = param.value
            param = self.node_.get_parameter(self.prefix_ + "min_size")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt_eq(param, 0)
            if validation_result:
                raise InvalidParameterValueException(
                    "min_size",
                    param.value,
                    "Invalid value set during initialization for parameter min_size: " + validation_result,
                )
            updated_params.min_size = param.value
            param = self.node_.get_parameter(self.prefix_ + "max_size")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt_eq(param, 0)
            if validation_result:
                raise InvalidParameterValueException(
                    "max_size",
                    param.value,
                    "Invalid value set during initialization for parameter max_size: " + validation_result,
                )
            updated_params.max_size = param.value
            param = self.node_.get_parameter(self.prefix_ + "debug_mode")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.debug_mode = param.value

            self.update_internal_params(updated_params)
