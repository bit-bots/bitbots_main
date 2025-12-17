import os
from pathlib import Path

import jax
import orbax.checkpoint as ocp
from brax.training.acme import running_statistics
from brax.training.agents.ppo import networks as ppo_networks
from dotenv import load_dotenv
from jax import tree_util

load_dotenv()

CKPT_PATH = f"{os.getenv('RL_MODELS_PATH')}/walking_without_delay"  # path to checkpoint directory

p = Path(CKPT_PATH)
cp = ocp.PyTreeCheckpointer()

restored = cp.restore(str(p))

normalizer_state = restored[0]
ppo_params = restored[1]

policy_params = ppo_params["policy"]
value_params = ppo_params["value"]

policy_layer_sizes = []

# Size of policy network

for _path, leaf in tree_util.tree_leaves_with_path(policy_params):
    if hasattr(leaf, "shape"):
        if len(leaf.shape) == 2:
            policy_layer_sizes.append(leaf.shape[0])

output_size = list(tree_util.tree_leaves(policy_params))[-1].shape[1]

policy_layer_sizes.append(output_size)

print(f"Policy layer sizes: {policy_layer_sizes}")

# Size of value network
value_layer_sizes = []

for _path, leaf in tree_util.tree_leaves_with_path(value_params):
    if hasattr(leaf, "shape"):
        if len(leaf.shape) == 2:
            value_layer_sizes.append(leaf.shape[0])

value_layer_sizes.append(list(tree_util.tree_leaves(value_params))[-1].shape[1])

print(f"Value layer sizes: {value_layer_sizes}")


# Defining the PPO network
def preprocess_observations(obs):
    normalized_obs, _ = running_statistics.normalize(obs, normalizer_state)
    return normalized_obs


ppo_net = ppo_networks.make_ppo_networks(
    observation_size=policy_layer_sizes[0],
    action_size=policy_layer_sizes[-1],
    preprocess_observations_fn=preprocess_observations,
    policy_hidden_layer_sizes=policy_layer_sizes[1:-1],
    value_hidden_layer_sizes=value_layer_sizes[1:-1],
)

make_inference = ppo_networks.make_inference_fn(ppo_net)

policy_fn = make_inference(policy_params, deterministic=True)

policy_fn = jax.jit(policy_fn)
