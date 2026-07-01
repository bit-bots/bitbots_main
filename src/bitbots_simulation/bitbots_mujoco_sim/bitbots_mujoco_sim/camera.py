import math

import mujoco
import numpy as np

# Camera parameters match the ZED mini config
CAMERA_WIDTH = 672
CAMERA_HEIGHT = 376
CAMERA_FX = 362.6533508300781
CAMERA_FY = 362.6533508300781
CAMERA_CX = 329.59454345703125
CAMERA_CY = 182.3579864501953


class Camera:
    """Represents a camera in the MuJoCo simulation, providing image rendering capabilities."""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        name: str,
        width: int = CAMERA_WIDTH,
        height: int = CAMERA_HEIGHT,
    ):
        self.model: mujoco.MjModel = model
        self.data: mujoco.MjData = data
        self.name: str = name
        self.instance = model.camera(name)
        self.width: int = width
        self.height: int = height
        self.fx: float = CAMERA_FX
        self.fy: float = CAMERA_FY
        self.cx: float = CAMERA_CX
        self.cy: float = CAMERA_CY
        self._fov: float | None = None
        self.renderer = mujoco.Renderer(self.model, height=self.height, width=self.width)

    @property
    def fov(self) -> float:
        """Returns the camera's horizontal field of view in radians."""
        if self._fov is not None:
            return self._fov

        self._fov = 2 * math.atan(self.width / (2 * self.fx))
        return self._fov

    def render(self) -> bytes:
        """
        Renders and returns the camera image as raw bytes (BGRA format).
        Returns Raw image data in BGRA8 format for ROS Image messages.
        """
        # Update renderer with current scene
        self.renderer.update_scene(self.data, camera=self.name)

        # Render the image - returns RGB by default
        rgb_array = self.renderer.render()

        # Convert RGB to BGRA (standard for ROS Image messages)
        # MuJoCo returns RGB uint8 array of shape (height, width, 3)
        # We need to add alpha channel and swap R and B
        height, width, _ = rgb_array.shape
        bgra_array = np.zeros((height, width, 4), dtype=np.uint8)
        bgra_array[:, :, 0] = rgb_array[:, :, 2]  # B
        bgra_array[:, :, 1] = rgb_array[:, :, 1]  # G
        bgra_array[:, :, 2] = rgb_array[:, :, 0]  # R
        bgra_array[:, :, 3] = 255  # A (fully opaque)

        return bgra_array.tobytes()
