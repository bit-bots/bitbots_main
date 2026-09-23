"""Convert the simulation snapshot into the referee's world-frame observations."""

from bitbots_msgs.msg import SimulationState

from bitbots_auto_referee.core.observations import RobotMotion, SimulationObservation


def decode_observation(message: SimulationState) -> SimulationObservation:
    return SimulationObservation(
        time_ns=message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec,
        step_number=message.step_number,
        ball_position=(message.ball_position.x, message.ball_position.y, message.ball_position.z)
        if message.ball_present
        else None,
        robot_positions={
            robot.robot_index: (robot.position.x, robot.position.y, robot.position.z) for robot in message.robots
        },
        touching_ball=frozenset(robot.robot_index for robot in message.robots if robot.touching_ball),
        teleported_robots=frozenset(message.teleported_robots),
        ball_teleported=message.ball_teleported,
        robot_motion={
            robot.robot_index: RobotMotion(
                robot.linear_speed, robot.angular_speed, robot.body_joint_speed,
                robot.head_joint_speed, robot.upright, robot.relative_height,
            ) for robot in message.robots if robot.motion_valid
        },
    )
