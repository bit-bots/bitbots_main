#!/usr/bin/env python3
import json
import sys
import traceback
from typing import Optional

import numpy as np
import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_srvs.srv import SetBool

from bitbots_animation_server.animation import Animation, parse
from bitbots_animation_server.resource_manager import ResourceManager
from bitbots_animation_server.spline_animator import SplineAnimator
from bitbots_msgs.action import PlayAnimation
from bitbots_msgs.msg import Animation as AnimationMsg
from bitbots_msgs.msg import JointCommand
from bitbots_msgs.srv import AddAnimation

# A hashable definition for the UUID of the goals
UUID = tuple[int]


class AnimationNode(Node):
    """This node provides an action server for playing animations."""

    def __init__(self):
        """Starts a simple action server and waits for requests."""
        super().__init__("animation_server")
        self.get_logger().debug("Starting Animation Server")

        self.current_joint_states: Optional[JointState] = None
        self.current_animation = None
        self.animation_cache: dict[str, Animation] = {}
        self.running_goals: set[UUID] = set()
        self.goals_to_abort: set[UUID] = set()

        # Get robot type and create resource manager
        self.declare_parameter("robot_type", "wolfgang")
        self.resource_manager = ResourceManager(self.get_parameter("robot_type").value)

        # Load all animations into memory
        all_animations = self.resource_manager.find_all_animations_by_name(self)
        for animation_name, animation_file in all_animations.items():
            try:
                with open(animation_file) as fp:
                    self.animation_cache[animation_name] = parse(json.load(fp))
            except OSError:
                self.get_logger().error(f"Animation '{animation_name}' could not be loaded")
            except ValueError:
                self.get_logger().error(
                    f"Animation '{animation_name}' had a ValueError. "
                    "Probably there is a syntax error in the animation file. "
                    "See traceback"
                )
                traceback.print_exc()

        callback_group = ReentrantCallbackGroup()

        # Subscribers
        self.create_subscription(JointState, "joint_states", self.update_current_pose, 1, callback_group=callback_group)

        # Service clients
        self.hcm_animation_mode = self.create_client(SetBool, "play_animation_mode")

        # Publisher for sending joint states to the HCM
        self.hcm_publisher = self.create_publisher(AnimationMsg, "animation", 1)

        # Action server for playing animations
        self._action_server = ActionServer(
            self, PlayAnimation, "animation", self.execute_cb, callback_group=callback_group, goal_callback=self.goal_cb
        )

        # Service to temporarily add an animation to the cache
        self.add_animation_service = self.create_service(AddAnimation, "add_temporary_animation", self.add_animation)

    def goal_cb(self, request: PlayAnimation.Goal) -> GoalResponse:
        """This checks whether the goal is acceptable."""

        # Check if we know the animation
        if request.animation not in self.animation_cache:
            self.get_logger().error(f"Animation '{request.animation}' not found")
            return GoalResponse.REJECT

        # Check if another animation is currently running
        if len(self.running_goals):
            if request.hcm:
                self.get_logger().warn(
                    "Another animation is currently running, but it is from the HCM. Aborting the old one..."
                )
                # Cancel the running goals
                self.goals_to_abort = self.goals_to_abort | self.running_goals
            else:
                self.get_logger().error("Another animation is currently running. Rejecting...")
                return GoalResponse.REJECT

        # Check if bounds are valid
        if request.bounds:
            # Get the length of the animation
            animation_length = len(self.animation_cache[request.animation].keyframes)
            # Check if the bounds are valid
            if request.start < 0 or request.start >= animation_length:
                self.get_logger().error(
                    f"Start index {request.start} out of bounds for animation '{request.animation}'"
                )
                return GoalResponse.REJECT
            if request.end < 0 or request.end > animation_length:
                self.get_logger().error(f"End index {request.end} out of bounds for animation '{request.animation}'")
                return GoalResponse.REJECT
            if request.start >= request.end:
                self.get_logger().error(f"Start index {request.start} must be smaller than end index {request.end}")
                return GoalResponse.REJECT

        # Everything is fine we are good to go
        return GoalResponse.ACCEPT

    def execute_cb(self, goal: ServerGoalHandle) -> PlayAnimation.Result:
        """This is called, when the action should be executed."""

        # Convert goal id uuid to hashable tuple (custom UUID type)
        goal_id: UUID = tuple(goal.goal_id.uuid)

        # Store the that the animation is running / the goal is active
        self.running_goals.add(goal_id)

        # Define a function to finish the action and return the resource
        def finish(successful: bool) -> PlayAnimation.Result:
            """This is called when the action is finished."""
            # Remove references to the goal
            self.running_goals.remove(goal_id)
            self.goals_to_abort.discard(goal_id)  # We use discard here, because the goal might not be in the set
            # Set the goal as succeeded or aborted
            if successful:
                goal.succeed()
            else:
                goal.abort()
            # Create the appropriate result object
            return PlayAnimation.Result(successful=successful)

        # Set type for request
        request: PlayAnimation.Goal = goal.request

        self.current_animation = request.animation

        # publish info to the console for the user
        self.get_logger().info(f"Request to play animation {self.current_animation}")

        # If the request is not from the HCM we might need to wait for the HCM to be controllable
        if not request.hcm:
            # Wait for HCM to be up and running
            if not self.hcm_animation_mode.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("HCM not available. Aborting animation...")
                return finish(successful=False)

            # Send request to make the HCM to go into animation play mode
            num_tries = 0
            while rclpy.ok() and (not self.hcm_animation_mode.call(SetBool.Request(data=True)).success):
                if num_tries >= 10:
                    self.get_logger().error("Failed to request HCM to go into animation play mode")
                    return finish(successful=False)
                num_tries += 1
                self.get_clock().sleep_for(Duration(seconds=0.1))

        # Create splines
        animator = SplineAnimator(
            self.animation_cache[self.current_animation], self.current_joint_states, self.get_logger(), self.get_clock()
        )

        # Flag that determines that we have send something once
        once = False

        # Loop to play the animation
        while rclpy.ok() and animator and goal_id not in self.goals_to_abort:
            try:
                last_time = self.get_clock().now()

                # if we're here we want to play the next keyframe, cause there is no other goal
                # compute next pose
                t = (
                    self.get_clock().now().nanoseconds / 1e9 - animator.get_start_time()
                )  # time since start of animation
                # Start later if we have set bounds and only play the part of the animation
                if request.bounds and request.start > 0:
                    t += animator.get_keyframe_times()[request.start]

                # Get the robot pose at the current time from the spline interpolator
                pose = animator.get_positions_rad(t)

                # Finish if spline ends or we reaches the explicitly defined premature end
                if pose is None or (request.bounds and once and t > animator.get_keyframe_times()[request.end - 1]):
                    # Animation is finished, tell it to the hcm (except if it is from the hcm)
                    if not request.hcm:
                        hcm_result = self.hcm_animation_mode.call(SetBool.Request(data=False))
                        if not hcm_result.success:
                            self.get_logger().error(f"Failed to finish animation on HCM. Reason: {hcm_result.message}")

                    # We give a positive result
                    goal.publish_feedback(PlayAnimation.Feedback(percent_done=100))
                    return finish(successful=True)

                self.send_animation(from_hcm=request.hcm, pose=pose, torque=animator.get_torque(t))

                perc_done = int(
                    ((self.get_clock().now().nanoseconds / 1e9 - animator.get_start_time()) / animator.get_duration())
                    * 100
                )
                perc_done = max(0, min(perc_done, 100))

                goal.publish_feedback(PlayAnimation.Feedback(percent_done=perc_done))

                once = True

                self.get_clock().sleep_until(last_time + Duration(seconds=0.02))
            except (ExternalShutdownException, KeyboardInterrupt):
                sys.exit(0)
        return finish(successful=False)

    def update_current_pose(self, msg):
        """Gets the current motor positions and updates the representing pose accordingly."""
        self.current_joint_states = msg

    def send_animation(self, from_hcm: bool, pose: dict, torque: Optional[dict]):
        """Sends an animation to the hcm"""
        self.hcm_publisher.publish(
            AnimationMsg(
                from_hcm=from_hcm,
                joint_command=JointCommand(
                    header=Header(
                        stamp=self.get_clock().now().to_msg(),
                    ),
                    joint_names=pose.keys(),
                    positions=pose.values(),
                    velocities=[-1.0] * len(pose),
                    accelerations=[-1.0] * len(pose),
                    max_currents=[np.clip((torque[joint]), 0.0, 1.0) for joint in pose]
                    if torque and False  # TODO
                    else [-1.0] * len(pose),  # fmt: skip
                ),
            )
        )

    def add_animation(self, request: AddAnimation.Request, response: AddAnimation.Response) -> AddAnimation.Response:
        """
        Adds an animation to the cache (non persistent).
        This is useful if e.g. the recording GUI wants to play an uncommitted animation.
        """
        try:
            # Parse the animation
            new_animation = parse(json.loads(request.json))
            # Get the name of the animation and add it to the cache
            self.animation_cache[new_animation.name] = new_animation
        except ValueError:
            self.get_logger().error(
                "Animation provided by service call had a ValueError. "
                "Probably there is a syntax error in the animation file. "
                "See traceback"
            )
            traceback.print_exc()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AnimationNode()
    ex = MultiThreadedExecutor(num_threads=10)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
