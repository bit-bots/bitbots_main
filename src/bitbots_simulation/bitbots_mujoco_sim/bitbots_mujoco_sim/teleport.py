"""Queue planar teleports for execution by the physics thread."""

import math
from queue import Empty, Full, Queue

import mujoco
from rclpy.task import Future
from transforms3d.euler import euler2quat

from bitbots_msgs.srv import Teleport


class TeleportController:
    def __init__(self, model: mujoco.MjModel, robot_body_ids: dict[int, int], ball_joint_id: int):
        self.model = model
        self.ball_joint_id = ball_joint_id
        self.robot_joints = {index: int(model.body_jntadr[body]) for index, body in robot_body_ids.items()}
        self._queue: Queue = Queue(maxsize=128)

    async def callback(self, request: Teleport.Request, response: Teleport.Response) -> Teleport.Response:
        if not all(math.isfinite(value) for value in (request.x, request.y, request.yaw)):
            response.message = "Teleport coordinates and yaw must be finite"
            return response
        if request.target_type == Teleport.Request.BALL:
            joint = self.ball_joint_id
        elif request.target_type == Teleport.Request.ROBOT:
            joint = self.robot_joints.get(request.robot_index, -1)
        else:
            response.message = "Unknown teleport target type"
            return response
        if joint < 0 or self.model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_FREE:
            response.message = "Teleport target does not exist or has no free joint"
            return response
        completed = Future()
        try:
            self._queue.put_nowait((request, joint, completed))
        except Full:
            response.message = "Teleport queue is full"
            return response
        return await completed

    def apply_pending(self, data: mujoco.MjData, step_number: int) -> tuple[set[int], bool]:
        """Apply a bounded batch before physics; preserve height and articulated pose."""
        teleported_robots: set[int] = set()
        ball_teleported = False
        for _ in range(self._queue.maxsize):
            try:
                request, joint, completed = self._queue.get_nowait()
            except Empty:
                break
            if completed.cancelled():
                continue
            qpos = int(self.model.jnt_qposadr[joint])
            root = int(self.model.jnt_bodyid[joint])
            data.qpos[qpos : qpos + 2] = [request.x, request.y]
            data.qpos[qpos + 3 : qpos + 7] = euler2quat(0.0, 0.0, request.yaw)
            for dof in range(self.model.nv):
                body = int(self.model.dof_bodyid[dof])
                while body and body != root:
                    body = int(self.model.body_parentid[body])
                if body == root:
                    data.qvel[dof] = 0.0
                    data.qacc_warmstart[dof] = 0.0
            if request.target_type == Teleport.Request.BALL:
                ball_teleported = True
            else:
                teleported_robots.add(request.robot_index)
            mujoco.mj_forward(self.model, data)
            completed.set_result(Teleport.Response(success=True, message="Teleport applied", applied_step=step_number))
        return teleported_robots, ball_teleported
