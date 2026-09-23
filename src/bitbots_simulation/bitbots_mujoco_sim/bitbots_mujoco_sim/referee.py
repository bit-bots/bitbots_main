"""Extract referee observations without modifying MuJoCo's physics state."""

import math

import mujoco

from bitbots_msgs.msg import SimulationRobotState, SimulationState


class RefereeObservationBuilder:
    def __init__(self, model: mujoco.MjModel, robot_body_ids: dict[int, int], ball_joint_id: int):
        self.robot_qpos = {
            index: int(model.jnt_qposadr[model.body_jntadr[body]]) for index, body in robot_body_ids.items()
        }
        self.robot_motion_layout = {}
        for index, root in robot_body_ids.items():
            joint = int(model.body_jntadr[root])
            head_dofs, body_dofs = [], []
            for candidate in range(model.njnt):
                if model.jnt_type[candidate] != mujoco.mjtJoint.mjJNT_HINGE:
                    continue
                body = int(model.jnt_bodyid[candidate])
                while body and body != root:
                    body = int(model.body_parentid[body])
                if body == root:
                    name = (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, candidate) or "").lower()
                    target = head_dofs if "head" in name else body_dofs
                    target.append(int(model.jnt_dofadr[candidate]))
            self.robot_motion_layout[index] = (root, int(model.jnt_dofadr[joint]), head_dofs, body_dofs)
        self.model = model
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
            root, dof, head_dofs, body_dofs = self.robot_motion_layout[index]
            robot.linear_speed = math.sqrt(sum(float(v) ** 2 for v in data.qvel[dof : dof + 3]))
            robot.angular_speed = math.sqrt(sum(float(v) ** 2 for v in data.qvel[dof + 3 : dof + 6]))
            robot.head_joint_speed = max((abs(float(data.qvel[d])) for d in head_dofs), default=0.0)
            robot.body_joint_speed = max((abs(float(data.qvel[d])) for d in body_dofs), default=0.0)
            robot.upright = float(data.xmat[root][8])
            initial_height = float(self.model.qpos0[qpos + 2])
            robot.relative_height = float(data.qpos[qpos + 2]) / initial_height if initial_height > 0 else 1.0
            robot.motion_valid = True
            message.robots.append(robot)
        return message
