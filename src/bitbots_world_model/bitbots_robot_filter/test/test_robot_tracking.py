#!/usr/bin/env python3
import math

import numpy as np
import soccer_vision_attribute_msgs.msg as svam
from geometry_msgs.msg import Vector3
from rclpy.duration import Duration
from rclpy.time import Time

from bitbots_robot_filter.filter import (
    TrackedRobot,
    TrackingConfig,
    angle_difference,
    directional_gain,
    radial_covariance,
)

CONFIG = TrackingConfig(alpha=0.4, beta=0.1, max_measurement_gap=0.5, max_velocity=1.0)

MEASUREMENT_COVARIANCE = np.eye(2) * 0.2

CREATED_AT = Time(seconds=1.0)


def create_robot(position=(2.0, 0.0), covariance=None) -> TrackedRobot:
    covariance = MEASUREMENT_COVARIANCE.copy() if covariance is None else covariance
    return TrackedRobot(
        position=np.array(position, dtype=float),
        covariance=covariance.copy(),
        size=Vector3(x=0.5, y=0.5, z=1.0),
        attributes=svam.Robot(team=svam.Robot.TEAM_OPPONENT),
        confidence=svam.Confidence(confidence=1.0),
        stamp=CREATED_AT,
        measurement_stamp=CREATED_AT,
        measurement_covariance=covariance.copy(),
    )


def observe(robot: TrackedRobot, position, after: float) -> None:
    robot.update(
        np.array(position, dtype=float),
        MEASUREMENT_COVARIANCE.copy(),
        CREATED_AT + Duration(seconds=after),
        CONFIG,
    )


##############
# Correction #
##############


def test_a_detection_moves_the_estimate_and_makes_it_more_certain():
    robot = create_robot(position=(2.0, 0.0))
    uncertainty = robot.uncertainty

    observe(robot, (2.4, 0.0), after=0.1)

    assert 2.0 < robot.position[0] < 2.4
    assert robot.uncertainty < uncertainty


def test_an_imprecise_detection_leaves_us_less_certain():
    precise = create_robot(position=(2.0, 0.0))
    imprecise = create_robot(position=(2.0, 0.0))

    precise.update(np.array([3.0, 0.0]), np.eye(2) * 0.01, CREATED_AT, CONFIG)
    imprecise.update(np.array([3.0, 0.0]), np.eye(2) * 5.0, CREATED_AT, CONFIG)

    assert precise.uncertainty < imprecise.uncertainty


def test_the_covariance_stays_symmetric():
    robot = create_robot(position=(3.0, 0.0))
    for angle in (0.1, -0.2, 0.4, 1.2):
        robot.update(
            np.array([3.0 * math.cos(angle), 3.0 * math.sin(angle)]),
            radial_covariance(angle, across_variance=0.05, along_variance=2.0),
            CREATED_AT,
            CONFIG,
        )
        assert np.allclose(robot.covariance, robot.covariance.T)


def test_a_radial_detection_corrects_the_direction_more_than_the_distance():
    # We track a robot three meters straight ahead and detect it further away and to the side
    robot = create_robot(position=(3.0, 0.0))
    detection = np.array([4.0, 1.0])

    robot.update(detection, radial_covariance(0.0, across_variance=0.05, along_variance=5.0), CREATED_AT, CONFIG)

    # The projection is imprecise along the line of sight, so we follow the sideways part of the
    # detection much more than the part that moves the robot away from us
    assert robot.position[1] / detection[1] > (robot.position[0] - 3.0) / (detection[0] - 3.0)


##########################
# Growth of the estimate #
##########################


def test_the_uncertainty_grows_while_we_do_not_observe_the_robot():
    robot = create_robot()
    uncertainty = robot.uncertainty

    robot.predict(0.05, 0.005)

    assert robot.uncertainty > uncertainty


def test_negative_observations_make_us_less_certain():
    robot = create_robot()
    uncertainty = robot.uncertainty

    robot.add_negative_observation(1.05)

    assert robot.uncertainty > uncertainty


def test_a_detection_we_can_not_project_does_not_move_the_estimate():
    robot = create_robot(position=(2.0, 0.0))
    position = robot.position.copy()
    velocity = robot.velocity.copy()

    robot.add_positive_observation(0.98, CREATED_AT + Duration(seconds=0.1))

    assert np.allclose(robot.position, position)
    assert np.allclose(robot.velocity, velocity)
    # It does tell us that the robot is still there
    assert robot.stamp == CREATED_AT + Duration(seconds=0.1)


def test_our_certainty_decays_slower_while_we_detect_a_robot_we_can_not_project():
    confirmed = create_robot()
    unobserved = create_robot()
    missing = create_robot()

    for step in range(10):
        for robot in (confirmed, unobserved, missing):
            robot.predict(0.05, 0.005)
        confirmed.add_positive_observation(0.98, CREATED_AT + Duration(seconds=0.05 * step))
        missing.add_negative_observation(1.05)

    assert confirmed.uncertainty < unobserved.uncertainty < missing.uncertainty


def test_a_detection_we_can_not_project_never_makes_us_as_certain_as_a_real_one():
    robot = create_robot()
    # An estimate that is exactly as good as the detection it is based on
    robot.covariance = robot.measurement_covariance.copy()

    for _ in range(100):
        robot.add_positive_observation(0.98, CREATED_AT)

    assert np.trace(robot.covariance) >= np.trace(robot.measurement_covariance)


############
# Velocity #
############


def test_consecutive_detections_estimate_the_velocity():
    robot = create_robot(position=(2.0, 0.0))

    observe(robot, (2.4, 0.0), after=0.1)

    # The robot moved 0.4 meters in 0.1 seconds and we follow that with the beta gain
    assert math.isclose(robot.velocity[0], CONFIG.beta * 0.4 / 0.1)
    assert robot.velocity[1] == 0.0


def test_the_velocity_is_lost_if_we_did_not_see_the_robot_for_a_while():
    robot = create_robot(position=(2.0, 0.0))
    observe(robot, (2.4, 0.0), after=0.1)
    assert robot.velocity[0] > 0.0

    observe(robot, (2.8, 0.0), after=0.1 + 2 * CONFIG.max_measurement_gap)

    assert np.allclose(robot.velocity, 0.0)


def test_a_detection_we_can_not_project_does_not_interrupt_the_velocity_estimation():
    robot = create_robot(position=(2.0, 0.0))
    observe(robot, (2.4, 0.0), after=0.1)

    # A detection we could not project arrives in between and must not count as a measurement
    robot.add_positive_observation(0.98, CREATED_AT + Duration(seconds=0.15))
    observe(robot, (2.8, 0.0), after=0.2)

    assert robot.velocity[0] > 0.0


def test_the_velocity_is_limited_to_what_a_robot_can_walk():
    robot = create_robot(position=(2.0, 0.0))

    observe(robot, (12.0, 0.0), after=0.05)

    assert float(np.linalg.norm(robot.velocity)) == CONFIG.max_velocity


def test_a_prediction_follows_the_estimated_velocity():
    robot = create_robot(position=(2.0, 0.0))
    observe(robot, (2.4, 0.0), after=0.1)
    position = robot.position.copy()

    robot.predict(0.5, 0.0)

    assert np.allclose(robot.position, position + robot.velocity * 0.5)


########
# Misc #
########


def test_an_isotropic_detection_uses_the_plain_gain():
    assert np.allclose(directional_gain(0.4, np.eye(2) * 7.0), np.eye(2) * 0.4)


def test_a_tracked_robot_is_converted_to_a_detection_on_the_ground():
    robot = create_robot(position=(1.5, -0.5))
    msg = robot.to_msg()

    assert msg.bb.center.position.x == 1.5
    assert msg.bb.center.position.y == -0.5
    # The bounding box is centered at half the height of the robot
    assert msg.bb.center.position.z == robot.size.z / 2
    assert msg.bb.size == robot.size
    assert msg.attributes.team == svam.Robot.TEAM_OPPONENT


def test_angles_are_compared_across_the_wrap_around():
    assert math.isclose(angle_difference(math.pi - 0.1, -math.pi + 0.1), -0.2, abs_tol=1e-9)
    assert math.isclose(angle_difference(0.2, 0.1), 0.1, abs_tol=1e-9)
