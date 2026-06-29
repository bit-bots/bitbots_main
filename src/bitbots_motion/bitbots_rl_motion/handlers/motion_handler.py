import os

import numpy as np
from ament_index_python import get_package_share_directory

from handlers.handler import Handler


class MotionHandler(Handler):
    """Holds a BeyondMimic reference motion clip (npz) and a finite motion clock.

    Serves the policy ``command`` term (reference joint_pos + joint_vel at the current
    frame) and the reference anchor (base_link) orientation used to build
    ``motion_anchor_ori_b``. Replaces the cyclic ``Phase`` of the walk policy with a
    one-shot clock that the action server starts and that terminates at the end of the
    clip. The clip length is read from the npz, so nothing is hardcoded.
    """

    def __init__(self, node):
        self._node = node

        motion_file = self._node.get_parameter("motion.file").value
        path = os.path.join(get_package_share_directory("bitbots_rl_motion"), "motions", motion_file)
        data = np.load(path)

        self._joint_pos = data["joint_pos"].astype(np.float32)  # (T, 20) policy joint order
        self._joint_vel = data["joint_vel"].astype(np.float32)  # (T, 20)
        # body index 0 == anchor body (base_link); see sim2sim motion_body_index.
        self._ref_anchor_quat_w = data["body_quat_w"][:, 0].astype(np.float32)  # (T, 4) wxyz

        self.num_frames = int(self._joint_pos.shape[0])
        self._t = 0
        self._active = False

    # --- clock control ---
    def start(self) -> None:
        self._t = 0
        self._active = True

    def stop(self) -> None:
        self._active = False

    def advance(self) -> None:
        self._t += 1

    def finished(self) -> bool:
        return self._t >= self.num_frames

    def progress(self) -> float:
        return min(self._t / max(self.num_frames, 1), 1.0)

    def time_step(self) -> int:
        return self._idx()

    def _idx(self) -> int:
        return min(self._t, self.num_frames - 1)

    # --- observation data ---
    def get_command(self) -> np.ndarray:
        i = self._idx()
        return np.concatenate([self._joint_pos[i], self._joint_vel[i]])  # (40,)

    def get_ref_anchor_quat(self) -> np.ndarray:
        return self._ref_anchor_quat_w[self._idx()]  # (4,) wxyz

    def has_data(self) -> bool:
        return True  # motion is loaded at construction time
