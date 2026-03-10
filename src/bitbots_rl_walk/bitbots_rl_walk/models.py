import os

from ament_index_python import get_package_share_directory
from handler.gyro_handler import GyroHandler

from src.rl_node import RLNode

walk_policy_path = os.path.join(get_package_share_directory("bitbots_rl_walk"), "models", "wolfgang_walk_ppo.onnx")
kick_policy_path = os.path.join(get_package_share_directory("bitbots_rl_walk"), "models", "wolfgang_kick_ppo.onnx")

# Attention: Handler structure should match obs structure!
walk_policy_obs_stack = [
    GyroHandler(),
    # GravityHandler(),
]

walk_policy_publishers = []


walk_node = RLNode(walk_policy_path)
kick_node = RLNode(kick_policy_path)
