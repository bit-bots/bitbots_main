# -*- coding:utf-8 -*-
"""
GoToAbsolutePosition
^^^^^^^^^^^^^^^^^^^^

This Module is responsible for walking to a specific point with a specific
orientation within the absolute coordinate system.

"""
import cmath
import math

from bitbots_common.stackmachine.abstract_init_action_module import AbstractInitActionModule


class GoToAbsolutePosition(AbstractInitActionModule):
    def __init__(self, args=None):
        AbstractInitActionModule.__init__(self, args)

        # Save the target we want to go to
        self.target_x = args[0]
        self.target_y = args[1]
        self.target_o = args[2]

        self.target = args

        self.align_on_zero = False

    def perform(self, connector, reevaluate=False):

        # If there is no current position available we can't go to any target one
        if not connector.localization.has_current_position():
            # TODO We need to consider a certain up-to-dateness
            return self.pop()

        # Else get the current position and orientation and decompose it to the three attributes
        position_triple = connector.localization.get_current_position()
        current_x, current_y, current_o = position_triple

        # Estimate the distance error as radial distance
        radial_error = self.get_radial_error(current_x, current_y)

        radial_tolerance = 500
        angular_tolerance_target = 20

        # When we don't met the radial error we want to turn and go forward
        if radial_error > radial_tolerance:
            # We need to turn towards the center of the target position and then walk forward
            vector_towards_center, radius, angle_to_target = self.determine_vector_and_angle_towards_target(current_x,
                                                                                                            current_y)

            angular_towards_target_error = self.determine_rotation_from_to_angle(current_o, angle_to_target)

            if abs(angular_towards_target_error) > 20:
                if angular_towards_target_error > 0:
                    connector.walking_capsule().start_walking(WalkingCapsule.ZERO, WalkingCapsule.MEDIUM_ANGULAR_LEFT,
                                                              WalkingCapsule.ZERO)
                    return
                else:
                    connector.walking_capsule().start_walking(WalkingCapsule.ZERO, WalkingCapsule.MEDIUM_ANGULAR_RIGHT,
                                                              WalkingCapsule.ZERO)
                    return
            else:
                connector.walking_capsule().start_walking(WalkingCapsule.MEDIUM_FORWARD, WalkingCapsule.ZERO,
                                                          WalkingCapsule.ZERO)
                return
        else:
            # We don't need to get closer to the point but we need to correct for the
            angular_target_error = self.determine_rotation_from_to_angle(current_o, self.target_o)

            if abs(angular_target_error) > angular_tolerance_target:
                if sign(angular_target_error) > 0:
                    connector.walking_capsule().start_walking(WalkingCapsule.ZERO, WalkingCapsule.MEDIUM_ANGULAR_LEFT,
                                                              WalkingCapsule.ZERO)
                    return
                else:
                    connector.walking_capsule().start_walking(WalkingCapsule.ZERO, WalkingCapsule.MEDIUM_ANGULAR_RIGHT,
                                                              WalkingCapsule.ZERO)
                    return
            else:
                connector.walking_capsule().stop_walking()
                self.pop()
                return

    def get_radial_error(self, current_x, current_y):
        distance_error = (self.target_x - current_x) ** 2
        distance_error += (self.target_y - current_y) ** 2
        distance_error = math.sqrt(distance_error)
        return distance_error

    def determine_vector_and_angle_towards_target(self, current_x, current_y):
        vector_towards_center = [self.target_x - current_x, self.target_y - current_y]
        vector_to_center_as_complex = complex(vector_towards_center[0], vector_towards_center[1])
        radius, angle = cmath.polar(vector_to_center_as_complex)
        return vector_towards_center, radius, int(math.degrees(angle) % 360)

    def determine_rotation_from_to_angle(self, from_angle, to_angle):
        rad_f_angle = 2 * math.pi * (from_angle / 360.0)
        rad_t_angle = 2 * math.pi * (to_angle / 360.0)

        erg = math.cos(rad_f_angle) * math.sin(rad_t_angle) - math.cos(rad_t_angle) * math.sin(rad_f_angle)

        if abs(erg) < 1E-12:
            if abs(from_angle - to_angle) <= 1:
                return 0
            else:
                return 180

        if erg > 0:
            return to_angle - from_angle
        elif erg < 0:
            return -1 * ((from_angle - to_angle) % 360)


