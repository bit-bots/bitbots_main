#! /usr/bin/env python3

import os

import rosparam
import rospkg
import rospy
import deep_quintic
import yaml
from deep_quintic import env
from deep_quintic.ros_runner import ALGOS, create_test_env, get_saved_hyperparams

if __name__ == '__main__':
    model_folder = rosparam.get_param("/rl_walk/model_folder")
    rospack = rospkg.RosPack()
    package_path = rospack.get_path("bitbots_rl_motion")
    model_folder = os.path.join(package_path,"models", model_folder)
    hyperparams, stats_path = get_saved_hyperparams(model_folder, norm_reward=False, test_mode=True)

    # load env_kwargs if existing
    env_kwargs = {}
    args_path = os.path.join(model_folder, "args.yml")
    if os.path.isfile(args_path):
        with open(args_path, "r") as f:
            loaded_args = yaml.load(f, Loader=yaml.UnsafeLoader)  # pytype: disable=module-attr
            if loaded_args["env_kwargs"] is not None:
                env_kwargs = loaded_args["env_kwargs"]
    else:
        print(f"No args.yml found in {args_path}")
        exit()

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
    env = venv.venv.envs[0].env.env
    custom_objects = {
        "learning_rate": 0.0,
        "lr_schedule": lambda _: 0.0,
        "clip_range": lambda _: 0.0,
    }
    model_path = os.path.join(model_folder, "model.zip")
    rospy.loginfo(f"Loading model from {model_path}")
    model = ALGOS[loaded_args['algo']].load(model_path, env=venv, custom_objects=custom_objects)

    env.run_node(model, venv)
