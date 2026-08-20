=================================================
Reinforcement Learning of Policies and Deployment
=================================================

We use several reinforcement learning frameworks to train policies for our robots.
After the models have been trained we export them as onnx files which describe the neural network structure and weights.
We use the bitbots_rl_motion policy execution framework to deploy the models on our robots.

Policy Training
===============

Currently (August 2026) we use the following frameworks to train our policies:

- Walking: `mujoco_playground <https://github.com/Flova/mujoco_playground/tree/kick>`_
- Standup: mjlab (currently used) , `HoST <https://github.com/bit-bots/HoST>`_ (not in use currently), and `BeyondMimic <https://github.com/bit-bots/BeyondMimic>`_ (not in use currently)
- Kick: model with proprietary training code from hightorque
- Path following: `mjlab <https://github.com/bit-bots/mjlab_piplus>`_ (not in use currently)
- Fun stuff like cartwheel: `BeyondMimic <https://github.com/bit-bots/BeyondMimic>`_

Going forward we prefer using mjlab as, while it is slightly slower than mujoco_playground,
it uses manager based reinforcement learning.
Manager based reinforcement learning allows defining the RL problem as a set of configurations which are read by their specific managers.
For example, the reward manager reads its configuration to call the appropriate reward functions and combine their output.

We train the RL policies mostly on the GPU machines with the RTX 4090 and 5090 cards in our lab (cl05 and cl06).
Training requires a lot of parameter tuning of the reward function and some other parameters of the environment like observation noise and domain randomization.


Policy Deployment
=================

Policies are deployed on the robots using the `bitbots_rl_motion <https://github.com/bit-bots/bitbots_main/tree/main/src/bitbots_motion/bitbots_rl_motion>`_ package.
To add a new policy, firstly add a node and configuration and add it to the launch file.
To build the observation and process the action several handlers already exists but you may have to add new ones for your policy.
The readme of the repository contains further information on the structure of the framework.

An important aspect is the 

.. code-block:: python

    def allowed_states(self) -> bool:
        """
        Returns whether the policy is allowed to be executed in the current state.
        This is used to prevent the robot from falling over when executing a policy.
        """
        raise NotImplementedError

function. It defines in which states the policy is executed.

To integrate the policy in the code, use ros topics, services, or actions which can be triggered from other components such as the HCM, behavior, or path planning.
