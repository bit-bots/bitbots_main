====================================
Pi Plus AMP Walk and Kick Policies
====================================

This page describes the deployment of two policies for the HighTorque Pi Plus.
Both policies were trained with mjlab and an AMP (adversarial motion prior) style reward.
The ``bitbots_rl_motion`` package runs them with the same ``RLNode`` framework as the other policies
(see :doc:`rl_models`).

.. warning::
   No part of this integration was tested on the real robot.
   The tests check the shapes, the observation order, the joint order and the safety clamps.
   They do not show that the policies walk or kick on the hardware.


Components
==========

.. list-table::
   :header-rows: 1

   * - Policy
     - Node (executable)
     - Config
     - Model
   * - Walk
     - ``piplus_amp_walk_node``
     - ``configs/piplus_amp_walk_model.yaml``
     - ``models/piplus_amp_walk.onnx``
   * - Kick
     - ``piplus_amp_kick_node``
     - ``configs/piplus_amp_kick_model.yaml``
     - ``models/piplus_amp_kick.onnx``

The launch file ``rl_motion.launch`` starts these nodes as the default walk and kick (arguments ``walk`` and ``kick``),
in place of the previous ``walk_node`` and ``kick_ball_node``.
All walk policies publish to ``walking_motor_goals``, and all kick policies serve the ``rl_kick`` action,
so only one walk policy should be enabled at a time (``walk`` or ``mjlab_walk``).

The walk node uses the same interfaces as ``mjlab_walk_node``: ``cmd_vel``, ``imu/data``, ``joint_states`` and ``robot_state``.
The kick node uses the same ``rl_kick`` action (``bitbots_msgs/Kick``) as ``kick_ball_node``.
It also reads ``ball_position_relative_filtered`` and the transforms ``odom`` to ``base_footprint`` and the ball frame to ``torso_link``.


Provenance
==========

.. list-table::
   :header-rows: 1

   * - Policy
     - Training repository and task
     - Run directory
     - Checkpoint
   * - Walk
     - ``piplus_amp_mjlab`` (branch ``main``), task ``Mjlab-Velocity-Flat-Amp-DA-HighTorque-PiPlus``
     - ``logs/rsl_rl/piplus_velocity_amp_symmetric/2026-11-04_08-26-04_gait_pause_double_support``
       (resumed from ``model_22050.pt`` of ``2026-11-03_23-19-04_gait_pause_double_support``)
     - ``model_29999.pt`` (iteration 29999)
   * - Kick
     - ``AMP_mjlab`` (worktree ``AMP_mjlab-kick-noballvel``, branch ``amp-soccer-kick``), task ``PiPlus-AMP-Kick-DA``
     - ``logs/rsl_rl/piplus_amp_kick_symmetric/2026-11-02_21-25-35_kick_sym_huber``
     - ``model_29000.pt`` (iteration 29000)

Do not use ``model_29999.pt`` of the kick run.
The kick training became numerically unstable at approximately iteration 29467.

A new kick run (``2026-11-04_08-11-39_kick_sym_huber_coeff01_rest05_tanh``) observes the ball position with a tanh bound
(see `Ball position bound`_). It is not complete. Its ONNX model is not in the package yet.
When its model replaces the current model, export it from the final checkpoint and set ``ball.pos_tanh_scale``
to the value of its training. The current model must not get the tanh bound.

The ONNX files were exported on the CPU.
The walk model was exported with the ``MLPModel.as_onnx()`` function of the rsl_rl version of the training repository.
This is the same function that ``OnPolicyRunner.export_policy_to_onnx`` uses.
The kick model was rebuilt from the checkpoint (MLP with ELU and empirical normalization) and exported.
For both models, the ONNX output was compared with the PyTorch actor on random inputs.
The maximum relative difference was less than 1e-3.
The kick export is also equal to the export that the ``play.py`` script of the training repository made for the same checkpoint.

Both ONNX models contain the observation normalization of the training.
Thus, the nodes give the raw observation to the model.

The joint order, default pose, gains, action scales, joint ranges, history length, gait frequency range,
command ranges and control period in the configs come from the ``params/env.yaml`` file of each run and
from the robot model of the training (``piplus.xml``).
Both policies run at the control period of the training (``phase.control_dt``).


Joint order
===========

Both policies use the joint order of the training robot model (``piplus.xml``).
The config parameter ``joints.ordered_relevant_joint_names`` contains this order.
The ``JointHandler`` reads and writes the joints by name, so the order of ``joint_states`` on the robot is not important.

.. list-table::
   :header-rows: 1

   * - Policy index
     - Joint (training model and robot)
   * - 0 - 3
     - ``r_shoulder_pitch_joint``, ``r_shoulder_roll_joint``, ``r_upper_arm_joint``, ``r_elbow_joint``
   * - 4 - 7
     - ``l_shoulder_pitch_joint``, ``l_shoulder_roll_joint``, ``l_upper_arm_joint``, ``l_elbow_joint``
   * - 8 - 13
     - ``r_hip_pitch_joint``, ``r_hip_roll_joint``, ``r_thigh_joint``, ``r_calf_joint``, ``r_ankle_pitch_joint``, ``r_ankle_roll_joint``
   * - 14 - 19
     - ``l_hip_pitch_joint``, ``l_hip_roll_joint``, ``l_thigh_joint``, ``l_calf_joint``, ``l_ankle_pitch_joint``, ``l_ankle_roll_joint``

The training model, ``bitbots_mujoco_sim/xml/pi_plus.xml`` and the ``piplus_description`` URDF use the same joint names and the same joint axes.
The ``ref`` values of the shoulder joints in the MuJoCo models are equal to the origin rotations of the shoulder joints in the URDF.
Thus, the joint angle of the training model is equal to the joint angle of the robot, and all joint signs are +1 (``joints.joint_signs`` is not set).
The head joints are not part of the policies. The nodes do not send commands to the head joints.

The joint ranges in the configs are the ranges of the training model.
For the hip roll joints, these ranges are smaller than the URDF ranges.
The test makes sure that the config ranges are inside the URDF ranges.


Walk policy
===========

The actor observation is one frame. There is no history.

.. list-table::
   :header-rows: 1

   * - Index
     - Term
     - Size
     - Source on the robot
   * - 0 - 2
     - Base angular velocity
     - 3
     - ``GyroHandler`` (``imu/data``)
   * - 3 - 5
     - Projected gravity
     - 3
     - ``GravityHandler`` (``imu/data`` orientation)
   * - 6 - 25
     - Joint positions minus the default pose
     - 20
     - ``JointHandler``
   * - 26 - 45
     - Joint velocities
     - 20
     - ``JointHandler``
   * - 46 - 66
     - Last action (20 joint actions and the gait frequency action)
     - 21
     - Previous output of the policy
   * - 67 - 69
     - Velocity command (vx, vy, wz)
     - 3
     - ``cmd_vel``, clipped to the training range
   * - 70 - 71
     - Gait phase (sin, cos)
     - 2
     - ``GaitClockHandler``
   * - 72
     - Stand flag
     - 1
     - 1 if all three commands are zero, else 0

The output has 21 values.
The first 20 values are the joint actions. The joint target is the default pose plus the action times the action scale of the joint.
The last value sets the gait clock frequency.

Gait clock
----------

During training, the gait clock runs inside the environment. The node runs the same clock (``GaitClockHandler``):

#. The observation of a step contains (sin, cos) of 2 pi times the phase.
#. The policy gives the frequency action.
#. The node calculates the frequency as the middle of the frequency range plus half of the range times tanh of the frequency action.
#. The node adds the frequency times the control period to the phase.
#. If the robot stands (all three velocity commands are zero), the phase does not go past the next
   double support phase (the next multiple of 0.5 at or after the current phase).
#. The node takes the phase modulo 1.

The next observation uses the new phase. This is the same order as in the training (``GaitPhaseAction.process_actions``).
Thus, at a zero command the clock runs on to the next double support phase and stops there.
When a non-zero command comes back, the clock runs on from that phase.
The stand flag and the clock pause use the same threshold (``STAND_COMMAND_THRESHOLD`` in ``piplus_amp_utils.py``).
The node uses the clipped command of the same step for the observation and for the clock.
At each start of the policy, the phase is set to ``gait.initial_phase`` and the last action is set to zero.
The training starts each episode with a random phase.

Start and stop
--------------

The policy runs in all robot states that allow walking, also at a zero command.
At a zero command, the stand flag is 1 and the policy stands still, as in the training.
This is the same behavior as ``walk_node``. ``mjlab_walk_node`` stops at a zero command instead.

The HCM controls the hand-over to other modules:
when the walk goals stop to move by more than a small threshold for ``standing_transition_delay``,
the HCM changes the robot state from WALKING to CONTROLLABLE.
Recent kick goals take precedence over walk goals, so the first goals of a kick switch the state to KICKING
even while the robot walks.
The HCM forwards walk goals only in the states CONTROLLABLE and WALKING.
In other states (for example KICKING or an animation), the node stops the policy.
When the node starts the policy again, it resets the last action and the gait clock.


Kick policy
===========

The actor observation is a history of frames.
The history length is ``obs.history_length``.
The order is time-major: all terms of the oldest frame come first, and all terms of the newest frame come last.
This is the ``history_ordering="time"`` setting of the training task.
The node uses a ``HistoryBuffer``. After each start of the policy, the first frame fills the full history, as in the training.

.. list-table::
   :header-rows: 1

   * - Index in a frame
     - Term
     - Size
     - Source on the robot
   * - 0 - 2
     - Base angular velocity
     - 3
     - ``GyroHandler`` (``imu/data``)
   * - 3 - 5
     - Projected gravity
     - 3
     - ``GravityHandler`` (``imu/data`` orientation)
   * - 6 - 25
     - Joint positions minus the default pose
     - 20
     - ``JointHandler``
   * - 26 - 45
     - Joint velocities
     - 20
     - ``JointHandler``
   * - 46 - 65
     - Last action
     - 20
     - Previous output of the policy (not clipped)
   * - 66 - 68
     - Ball position in the torso frame (x, y, z)
     - 3
     - ``BallHandler``: ball filter output transformed to ``torso_link``, each axis clipped,
       optional tanh bound
   * - 69 - 70
     - Kick direction (cos, sin) in the yaw frame of the robot
     - 2
     - ``AmpKickHandler``: direction of the kick goal, anchored in ``odom``
   * - 71
     - Kick intensity in [0, 1]
     - 1
     - ``AmpKickHandler``: from the strength of the kick goal

The output has 20 values, one action for each joint.
The joint target is the default pose plus the action times the action scale.

Kick command
------------

The ``rl_kick`` goal gives the kick direction as (x, y) in the body frame when the goal arrives.
The node anchors this direction in the ``odom`` frame.
At each step, it expresses the direction in the current yaw frame of ``base_footprint``.

The strength of the goal is the target ball speed in m/s.
The training maps the intensity x to the target ball speed 2 / (1 - x) m/s.
The node uses the inverse of this map and clips the result to ``command.max_kick_intensity``.
A strength of 2 m/s or less gives the intensity 0.

If the node did not receive a ball estimate since it started, it rejects the kick goal and writes a warning to the log.
Thus the policy does not run and the node sends no joint goals for this goal.

The policy runs while the kick goal is active and the robot state allows a kick.
Unlike the other kick policies, the kick may also start while the robot walks, so the robot does not have to
stand still before kicking. Its first joint goals switch the HCM to KICKING, which pauses the walk.
The policy has no "kick done" signal. The behavior starts a kick as soon as the ball is closer than
``rl_kick_max_ball_distance`` (body behavior config), without waiting until the robot is aligned to the ball,
and the kick runs as long as the ball stays that close:

* The node stops the kick if the ball is farther away than ``ball.max_kick_distance`` (keep it equal to the
  behavior value), or if the ball estimate is too old or its covariance is too large.
* The behavior cancels the kick when it decides to do something else, e.g. because the ball is too far away
  or lost. It does not forget the ball after a kick, so the ball can be tracked and approached again.
* There is no time limit by default: a goal timeout <= 0 uses ``command.kick_timeout``, and a
  ``command.kick_timeout`` <= 0 means no time limit.

Ball position bound
-------------------

Each axis of the ball position is clipped to ``ball.position_clip``.
If ``ball.pos_tanh_scale`` is a number s, each axis then becomes s * tanh(x / s).
In the training, this transform comes after the noise, the delay and the transform to the anchor frame
(``object_pos_b_noisy_delayed_world`` with ``tanh_scale``).
If ``ball.pos_tanh_scale`` is null, the node does not apply the transform.
The value must agree with the training of the loaded model.
The current model was trained without the transform, so its config sets null.


Safety clamps
=============

The actor output of both policies has no bounds.
During the kick training, actions of more than 1000 occurred before the training added a clip.

Both nodes apply these limits (``ActionLimiter``):

* The joint action is clipped so that the joint target stays inside the soft joint limits.
  The soft limits are the joint ranges in the config, shrunk around their centers by ``joints.soft_limit_factor``.
  The kick training uses the same clip (``LimitedJointPositionAction``).
  The walk training does not clip the action. For the walk, the clip is a deployment safety limit only.
* The last action in the observation is clipped to plus or minus ``joints.action_limit``.
  The training does not clip this value. In the kick training, the "last action" observation is the raw policy output
  (``ActionManager.action``), not the clipped action.
  Thus, the limit is much larger than the actions of the trained policies. It only stops a feedback loop that diverges.

The ``RLNode`` also skips steps with observations or actions that are not finite.


Known differences and limits
============================

* The kick policy was trained in simulation only. It has no guard against extreme poses.
* Neither policy was tested on the robot.
* The default poses of the two policies are not equal to each other and not equal to the ``walkready`` animation.
  A policy observes the joints relative to its own default pose.
  If the robot is far from this pose when the policy starts, the observation is outside the training data.
* The walk training removed the passive joint damping of the robot model. The real robot has this damping.
  The MuJoCo simulation (``bitbots_mujoco_sim``) also runs without passive joint damping, like the training.
* The training adds observation noise, delays and randomization of the IMU mounting. The nodes do not add noise.
* The kick training delays the ball position and the kick direction by up to five steps and adds noise to both.
  The real ball filter has its own delay and noise.
* The training observes the ball center. The node uses the position from the ball filter without a height correction.
* The kick can start from a walking robot. The kick training may not cover starting from a walking gait,
  so the first moments of the kick can be outside the training data. This was not tested.
