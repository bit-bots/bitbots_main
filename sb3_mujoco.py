from mujoco_env import WolfgangMujocoEnv

import numpy as np

from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import SubprocVecEnv

from stable_baselines3 import PPO


display_env = make_vec_env(WolfgangMujocoEnv, n_envs=1)

model = PPO("MlpPolicy", display_env, verbose=1, n_steps=512)

model.learn(20000, log_interval=1)

display_env = model.get_env()
obs = display_env.reset()

for i in range(1000):
    action, _state = model.predict(obs, deterministic=True)
    obs, reward, done, info = display_env.step(action)
    if done:
        obs = display_env.reset()
    if i % 10 == 0:
        display_env.render("human")

display_env.close()
