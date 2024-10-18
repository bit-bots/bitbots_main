# This defines an openai gym environment to walk with the wolfgang robot in mujoco.

import math
import mujoco
import gymnasium as gym
import cv2
from tqdm import tqdm
import numpy as np

import uuid


class WolfgangMujocoEnv(gym.Env):
    def __init__(self, render_mode = "human"):
        # Load the model
        self.model = mujoco.MjModel.from_xml_path("/home/florian/Projekt/bitbots/bitbots_main/bitbots_wolfgang/wolfgang_description/urdf/scene.xml")
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model, 415, 256)

        self.render_mode = render_mode

        # Extract joint names from the model (they are non unique!)
        # We do this by reading a null terminated string at the address of the joint name in the joint names buffer
        self.joint_ids_to_names = [self.model.names[self.model.name_jntadr[i]:].decode('utf-8').split('\0')[0] for i in range(self.model.njnt)]

        # Get which values of the observation belong to which joint
        self.joint_id_observation_start = self.model.jnt_qposadr

        # Get the actuator names (they are also not unique)
        self.actuator_ids_to_names = [self.model.names[self.model.name_actuatoradr[i]:].decode('utf-8').split('\0')[0] for i in range(self.model.nu)]


        # Define the action space
        self.action_space = gym.spaces.Box(low=-math.pi, high=math.pi, shape=(20,))
        # Define the observation space
        self.observation_space = gym.spaces.Box(low=-math.pi, high=math.pi, shape=(20,))

        self.max_length = 2

        self.name = "WolfgangMujocoEnv-" + str(uuid.uuid4())

    def reset(self, seed=None):
        super().reset(seed=seed)

        # Reset the simulation
        mujoco.mj_resetData(self.model, self.data)
        return self._get_obs(), {}

    def _get_obs(self):
        return self.data.qpos[self.joint_id_observation_start[1]:]

    def step(self, action):

        # Apply the action
        for i in range(len(action)):
            self.data.ctrl[i] = 0.2 * action[i]

        # Perform an action
        for _ in range(10):
            mujoco.mj_step(self.model, self.data)

        # Debugging
        # Render
        self.renderer.update_scene(self.data)
        img = self.renderer.render()
        cv2.imshow(self.name, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)

        # Calculate the reward
        reward = self.data.xpos[1][0]

        return self._get_obs(), reward, self.data.time >= self.max_length, False, {}

    def render(self):
        # Render the simulation
        self.renderer.update_scene(self.data)
        img = self.renderer.render()
        if self.render_mode == "human":
            cv2.imshow("Wolfgang", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
            cv2.waitKey(1)
        return img

    def close(self):
        super().close()
        # Explicitly delete the objects to free memory and get a more deterministic behavior
        del self.data
        del self.renderer
        del self.model


if __name__ == "__main__":
    env = WolfgangMujocoEnv()
    env.reset()
    last_frame = -1000
    for _ in tqdm(range(1000)):
        print(env.step(env.action_space.sample()))
        time = env.data.time
        if time - last_frame > 1/60:
            last_frame = time
            cv2.imshow("Wolfgang", cv2.cvtColor(env.render(), cv2.COLOR_RGB2BGR))
            cv2.waitKey(1)
    cv2.destroyAllWindows()
    env.close()
