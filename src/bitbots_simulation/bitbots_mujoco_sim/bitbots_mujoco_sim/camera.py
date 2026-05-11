import mujoco
import numpy as np


class Camera:
    """Represents a camera in the MuJoCo simulation, providing image rendering capabilities."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, name: str, width: int = 800, height: int = 600):
        self.model: mujoco.MjModel = model
        self.data: mujoco.MjData = data
        self.name: str = name
        self.instance = model.camera(name)
        self.width: int = width
        self.height: int = height
        self.renderer = mujoco.Renderer(self.model, height=self.height, width=self.width)

    @property
    def fov(self) -> float:
        """Returns the camera's horizontal field of view in radians."""
        if hasattr(self, "_fov") and self._fov is not None:
            return self._fov

        # MuJoCo uses fovy (vertical field of view in degrees)
        fovy_deg = self.instance.fovy[0] if hasattr(self.instance.fovy, "__iter__") else self.instance.fovy
        fovy_rad = np.deg2rad(fovy_deg)

        # Convert vertical FOV to horizontal FOV using aspect ratio
        aspect_ratio = self.width / self.height
        fovx_rad = 2 * np.arctan(np.tan(fovy_rad / 2) * aspect_ratio)

        self._fov = fovx_rad
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
