#! /usr/bin/env python3

import os
import sys

import rclpy
import yaml
from ament_index_python import get_package_share_directory
from deep_quintic import env
from deep_quintic.ros_runner import ALGOS, create_test_env, get_saved_hyperparams
from rclpy import Parameter
from rclpy.node import Node

if __name__ == "__main__":
    rclpy.init()
    node = Node("rl_walk")
    node.declare_parameter("model_folder", Parameter.Type.STRING)
    model_folder = node.get_parameter("model_folder").get_parameter_value().string_value
    package_path = get_package_share_directory("bitbots_rl_motion")
    model_folder = os.path.join(package_path, "rl_walk_models", model_folder)
    hyperparams, stats_path = get_saved_hyperparams(model_folder, norm_reward=False, test_mode=True)

    # load env_kwargs if existing
    env_kwargs = {}
    args_path = os.path.join(model_folder, "args.yml")
    if os.path.isfile(args_path):
        with open(args_path) as f:
            loaded_args = yaml.load(f, Loader=yaml.UnsafeLoader)  # pytype: disable=module-attr
            if loaded_args["env_kwargs"] is not None:
                env_kwargs = loaded_args["env_kwargs"]
    else:
        node.get_logger().fatal(f"No args.yml found in {args_path}")
        sys.exit()

    env_kwargs["node"] = node
    print(env_kwargs)
    venv = create_test_env(
        "ExecuteEnv-v1",
        n_envs=1,
        stats_path=stats_path,
        log_dir=None,
        should_render=False,
        hyperparams=hyperparams,
        env_kwargs=env_kwargs,
    )

    # direct reference to wolfgang env object
    env = venv.venv.envs[0].env.env  # noqa

    custom_objects = {
        "learning_rate": 0.0,
        "lr_schedule": lambda _: 0.0,
        "clip_range": lambda _: 0.0,
    }
    model_path = os.path.join(model_folder, "model")
    node.get_logger().info(f"Loading model from {model_path}")
    model = ALGOS[loaded_args["algo"]].load(model_path, env=venv, custom_objects=custom_objects)

    env.run_node(model, venv)
