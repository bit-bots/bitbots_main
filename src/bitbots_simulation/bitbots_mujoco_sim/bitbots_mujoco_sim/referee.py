"""Extract referee observations without modifying MuJoCo's physics state."""

import mujoco

from bitbots_msgs.msg import SimulationRobotState, SimulationState


class RefereeObservationBuilder:
    def __init__(self, model: mujoco.MjModel, robot_body_ids: dict[int, int], ball_joint_id: int):
        self.robot_qpos = {
            index: int(model.jnt_qposadr[model.body_jntadr[body]]) for index, body in robot_body_ids.items()
        }
        self.ball_qpos = int(model.jnt_qposadr[ball_joint_id]) if ball_joint_id >= 0 else None
        ball_body = int(model.jnt_bodyid[ball_joint_id]) if ball_joint_id >= 0 else None
        self.ball_geoms = {geom for geom in range(model.ngeom) if model.geom_bodyid[geom] == ball_body}
        roots = {body: index for index, body in robot_body_ids.items()}
        self.geom_robots = {}
        for geom in range(model.ngeom):
            body = int(model.geom_bodyid[geom])
            while body:
                if body in roots:
                    self.geom_robots[geom] = roots[body]
                    break
                body = int(model.body_parentid[body])

    def build(
        self,
        data: mujoco.MjData,
        stamp,
        step_number: int,
        teleported_robots: set[int] | None = None,
        ball_teleported: bool = False,
    ) -> SimulationState:
        """Copy end-of-step positions and contacts from the completed physics solve."""
        touching = set()
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            if contact.dist > 0 or contact.efc_address < 0:
                continue
            if contact.geom1 in self.ball_geoms:
                other = contact.geom2
            elif contact.geom2 in self.ball_geoms:
                other = contact.geom1
            else:
                continue
            if other in self.geom_robots:
                touching.add(self.geom_robots[other])

        message = SimulationState()
        message.header.stamp = stamp
        message.header.frame_id = "world"
        message.step_number = step_number
        message.teleported_robots = sorted(teleported_robots or ())
        message.ball_teleported = ball_teleported
        message.ball_present = self.ball_qpos is not None
        if self.ball_qpos is not None:
            position = data.qpos[self.ball_qpos : self.ball_qpos + 3]
            message.ball_position.x, message.ball_position.y, message.ball_position.z = map(float, position)
        for index, qpos in sorted(self.robot_qpos.items()):
            robot = SimulationRobotState(robot_index=index, touching_ball=index in touching)
            robot.position.x, robot.position.y, robot.position.z = map(float, data.qpos[qpos : qpos + 3])
            message.robots.append(robot)
        return message
