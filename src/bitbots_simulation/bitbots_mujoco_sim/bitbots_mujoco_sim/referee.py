"""Extract referee observations without modifying MuJoCo's physics state."""

import math

import mujoco
import numpy as np
from transforms3d.euler import mat2euler

from bitbots_msgs.msg import SimulationRobotContact, SimulationRobotState, SimulationState


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
        self.ball_body = ball_body
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

        self.robot_geoms = {
            index: [
                geom
                for geom, owner in self.geom_robots.items()
                if owner == index and (model.geom_contype[geom] or model.geom_conaffinity[geom])
            ]
            for index in robot_body_ids
        }

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
        ball_forces = {}
        robot_contacts = {}
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            if contact.dist > 0 or contact.efc_address < 0:
                continue
            first = self.geom_robots.get(contact.geom1)
            second = self.geom_robots.get(contact.geom2)
            if first is not None and second is not None and first != second:
                force = np.zeros(6)
                mujoco.mj_contactForce(self.model, data, contact_index, force)
                velocities = []
                for geom in (contact.geom1, contact.geom2):
                    velocity = np.zeros(6)
                    mujoco.mj_objectVelocity(self.model, data, mujoco.mjtObj.mjOBJ_GEOM, geom, velocity, 0)
                    velocities.append(velocity[3:] + np.cross(velocity[:3], contact.pos - data.geom_xpos[geom]))
                normal = np.asarray(contact.frame[:3])
                approach_first = max(0.0, float(np.dot(velocities[0], normal)))
                approach_second = max(0.0, float(np.dot(velocities[1], -normal)))
                pair = tuple(sorted((first, second)))
                entry = robot_contacts.setdefault(pair, [0.0, 0.0, 0.0, -1.0, (0.0, 0.0, 0.0)])
                magnitude = float(np.linalg.norm(force[:3]))
                entry[0] += magnitude
                if magnitude > entry[3]:
                    entry[3] = magnitude
                    entry[4] = tuple(float(value) for value in contact.pos)
                approaches = (
                    (approach_first, approach_second) if first == pair[0] else (approach_second, approach_first)
                )
                entry[1] = max(entry[1], approaches[0])
                entry[2] = max(entry[2], approaches[1])
            if contact.geom1 in self.ball_geoms:
                other = contact.geom2
            elif contact.geom2 in self.ball_geoms:
                other = contact.geom1
            else:
                continue
            if other in self.geom_robots:
                robot_index = self.geom_robots[other]
                touching.add(robot_index)
                force = np.zeros(6)
                mujoco.mj_contactForce(self.model, data, contact_index, force)
                ball_forces[robot_index] = ball_forces.get(robot_index, 0.0) + float(np.linalg.norm(force[:3]))

        message = SimulationState()
        message.header.stamp = stamp
        message.header.frame_id = "world"
        message.step_number = step_number
        for pair, values in sorted(robot_contacts.items()):
            contact_message = SimulationRobotContact(
                robot_a=pair[0],
                robot_b=pair[1],
                force=values[0],
                approach_a=values[1],
                approach_b=values[2],
                position_valid=True,
            )
            contact_message.position.x, contact_message.position.y, contact_message.position.z = values[4]
            message.robot_contacts.append(contact_message)
        message.teleported_robots = sorted(teleported_robots or ())
        message.ball_teleported = ball_teleported
        message.ball_present = self.ball_qpos is not None
        if self.ball_qpos is not None:
            position = data.qpos[self.ball_qpos : self.ball_qpos + 3]
            message.ball_position.x, message.ball_position.y, message.ball_position.z = map(float, position)
        for index, qpos in sorted(self.robot_qpos.items()):
            robot = SimulationRobotState(robot_index=index, touching_ball=index in touching)
            robot.ball_contact_force = ball_forces.get(index, 0.0)
            robot.position.x, robot.position.y, robot.position.z = map(float, data.qpos[qpos : qpos + 3])
            root, dof, head_dofs, body_dofs = self.robot_motion_layout[index]
            robot.linear_speed = math.sqrt(sum(float(v) ** 2 for v in data.qvel[dof : dof + 3]))
            robot.angular_speed = math.sqrt(sum(float(v) ** 2 for v in data.qvel[dof + 3 : dof + 6]))
            robot.head_joint_speed = max((abs(float(data.qvel[d])) for d in head_dofs), default=0.0)
            robot.body_joint_speed = max((abs(float(data.qvel[d])) for d in body_dofs), default=0.0)
            robot.yaw = float(mat2euler(np.asarray(data.xmat[root]).reshape(3, 3))[2])
            robot.heading_valid = True
            robot.upright = float(data.xmat[root][8])
            initial_height = float(self.model.qpos0[qpos + 2])
            robot.relative_height = float(data.qpos[qpos + 2]) / initial_height if initial_height > 0 else 1.0
            robot.motion_valid = True
            geoms = self.robot_geoms[index]
            if geoms:
                rotation = data.geom_xmat[geoms].reshape(-1, 3, 3)
                local_bounds = self.model.geom_aabb[geoms]
                centers = data.geom_xpos[geoms] + np.einsum("nij,nj->ni", rotation, local_bounds[:, :3])
                extents = np.einsum("nij,nj->ni", np.abs(rotation), local_bounds[:, 3:])
                lower = np.min(centers - extents, axis=0)
                upper = np.max(centers + extents, axis=0)
                robot.min_x, robot.min_y = float(lower[0]), float(lower[1])
                robot.max_x, robot.max_y = float(upper[0]), float(upper[1])
                robot.bounds_valid = True
            message.robots.append(robot)
        # Only cast rays when an upright robot actually surrounds the ball's horizontal position.
        if self.ball_qpos is not None:
            point = np.asarray(data.qpos[self.ball_qpos : self.ball_qpos + 3], dtype=float)
            candidates = {
                robot.robot_index
                for robot in message.robots
                if robot.bounds_valid
                and robot.upright > 0.85
                and robot.relative_height > 0.8
                and robot.min_x <= point[0] <= robot.max_x
                and robot.min_y <= point[1] <= robot.max_y
            }
            if candidates:
                counts = {index: 0 for index in candidates}
                samples = 16
                geom_id = np.empty(1, dtype=np.int32)
                for angle in np.linspace(0, 2 * math.pi, samples, endpoint=False):
                    direction = np.array([math.cos(angle), math.sin(angle), 0.0])
                    hit = mujoco.mj_ray(self.model, data, point, direction, None, 1, self.ball_body, geom_id, None)
                    owner = self.geom_robots.get(int(geom_id[0]))
                    if 0 <= hit <= 0.35 and owner in candidates:
                        geom = int(geom_id[0])
                        if self.model.geom_contype[geom] or self.model.geom_conaffinity[geom]:
                            counts[owner] += 1
                for robot in message.robots:
                    robot.ball_blockage = counts.get(robot.robot_index, 0) / samples
        return message
