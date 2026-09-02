#!/usr/bin/env python3
import numpy as np
from rclpy.time import Time

from bitbots_ball_filter.ball_filter import AlphaBetaFilter

ALPHA = 0.5
BETA = 0.5
MAX_MEASUREMENT_GAP = 1.0

MEASUREMENT_COVARIANCE = np.diag([0.5, 0.5, 0.0])

OWN_SOURCE = 0
TEAMMATE_SOURCE = 3


def create_filter() -> AlphaBetaFilter:
    ball_filter = AlphaBetaFilter()
    ball_filter.configure(ALPHA, BETA, MAX_MEASUREMENT_GAP)
    return ball_filter


def measure(ball_filter: AlphaBetaFilter, position, source: int, seconds: float) -> None:
    ball_filter.update(np.array(position), MEASUREMENT_COVARIANCE.copy(), source, Time(seconds=seconds))


def test_first_measurement_initializes_the_filter():
    ball_filter = create_filter()
    measure(ball_filter, [1.0, 2.0, 0.0], OWN_SOURCE, 1.0)

    assert np.allclose(ball_filter.position, [1.0, 2.0, 0.0])
    assert np.allclose(ball_filter.velocity, 0.0)
    assert np.allclose(ball_filter.covariance, MEASUREMENT_COVARIANCE)
    assert ball_filter.source == OWN_SOURCE


def test_consecutive_measurements_of_one_source_estimate_the_velocity():
    ball_filter = create_filter()
    measure(ball_filter, [0.0, 0.0, 0.0], OWN_SOURCE, 1.0)
    measure(ball_filter, [1.0, 0.0, 0.0], OWN_SOURCE, 2.0)

    # The ball moved one meter in one second and the filter follows it with the beta gain
    assert ball_filter.velocity[0] == BETA * 1.0
    assert np.allclose(ball_filter.velocity[1:], 0.0)


def test_a_source_switch_does_not_estimate_a_velocity():
    ball_filter = create_filter()
    measure(ball_filter, [0.0, 0.0, 0.0], OWN_SOURCE, 1.0)
    # The same ball, but observed by a teammate that is biased in another direction
    measure(ball_filter, [1.0, 0.0, 0.0], TEAMMATE_SOURCE, 2.0)

    # The position follows the new source, but its offset must not be read as ball movement
    assert ball_filter.position[0] == ALPHA * 1.0
    assert np.allclose(ball_filter.velocity, 0.0)
    assert ball_filter.source == TEAMMATE_SOURCE


def test_the_velocity_is_estimated_again_after_a_source_switch():
    ball_filter = create_filter()
    measure(ball_filter, [0.0, 0.0, 0.0], OWN_SOURCE, 1.0)
    measure(ball_filter, [1.0, 0.0, 0.0], TEAMMATE_SOURCE, 2.0)
    measure(ball_filter, [2.0, 0.0, 0.0], TEAMMATE_SOURCE, 3.0)

    assert ball_filter.velocity[0] > 0.0


def test_the_filter_restarts_after_losing_the_ball():
    ball_filter = create_filter()
    measure(ball_filter, [0.0, 0.0, 0.0], OWN_SOURCE, 1.0)
    measure(ball_filter, [1.0, 0.0, 0.0], OWN_SOURCE, 2.0)
    # We did not observe the ball for longer than the maximum measurement gap
    measure(ball_filter, [3.0, 0.0, 0.0], OWN_SOURCE, 2.0 + 2 * MAX_MEASUREMENT_GAP)

    assert np.allclose(ball_filter.position, [3.0, 0.0, 0.0])
    assert np.allclose(ball_filter.velocity, 0.0)


def test_the_uncertainty_grows_without_measurements():
    ball_filter = create_filter()
    measure(ball_filter, [0.0, 0.0, 0.0], OWN_SOURCE, 1.0)
    certainty = np.diag(ball_filter.covariance)[:2].copy()

    ball_filter.predict(time_step=0.1, process_noise=0.01)
    assert np.all(np.diag(ball_filter.covariance)[:2] > certainty)

    certainty = np.diag(ball_filter.covariance)[:2].copy()
    ball_filter.add_negative_observation(1.5)
    assert np.all(np.diag(ball_filter.covariance)[:2] > certainty)


def test_the_uncertainty_stays_finite():
    ball_filter = create_filter()
    measure(ball_filter, [0.0, 0.0, 0.0], OWN_SOURCE, 1.0)
    for _ in range(1000):
        ball_filter.predict(time_step=0.1, process_noise=0.01)
        ball_filter.add_negative_observation(1.5)

    assert np.all(ball_filter.covariance <= AlphaBetaFilter.UNKNOWN_COVARIANCE)


def test_a_prediction_follows_the_estimated_velocity():
    ball_filter = create_filter()
    measure(ball_filter, [0.0, 0.0, 0.0], OWN_SOURCE, 1.0)
    measure(ball_filter, [1.0, 0.0, 0.0], OWN_SOURCE, 2.0)
    position = ball_filter.position.copy()

    ball_filter.predict(time_step=0.5, process_noise=0.0)
    assert np.allclose(ball_filter.position, position + ball_filter.velocity * 0.5)
