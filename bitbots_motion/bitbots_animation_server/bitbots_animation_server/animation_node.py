#!/usr/bin/env python3
import json
import traceback

import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from bitbots_animation_server.animation import Animation, parse
from bitbots_animation_server.resource_manager import ResourceManager
from bitbots_animation_server.spline_animator import SplineAnimator
from bitbots_msgs.action import PlayAnimation
from bitbots_msgs.msg import Animation as AnimationMsg
from bitbots_msgs.msg import RobotControlState
from bitbots_msgs.srv import AddAnimation


class AnimationNode(Node):
    """This node provides an action server for playing animations."""

    def __init__(self):
        """Starts a simple action server and waits for requests."""
        super().__init__("animation_server")
        self.get_logger().debug("Starting Animation Server")

        self.current_joint_states = None
        self.hcm_state = 0
        self.current_animation = None
        self.animation_cache: dict[str, Animation] = {}

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
        self.create_subscription(
            RobotControlState, "robot_state", self.update_hcm_state, 1, callback_group=callback_group
        )

        # Publisher for sending joint states to the HCM
        self.hcm_publisher = self.create_publisher(AnimationMsg, "animation", 1)

        # Action server for playing animations
        self._as = ActionServer(self, PlayAnimation, "animation", self.execute_cb, callback_group=callback_group)

        # Service to temporarily add an animation to the cache
        self.add_animation_service = self.create_service(AddAnimation, "add_temporary_animation", self.add_animation)

    def execute_cb(self, goal: ServerGoalHandle):
        """This is called, when someone calls the animation action"""

        # Set type for request
        request: PlayAnimation.Goal = goal.request

        first = True
        self.current_animation = request.animation

        # publish info to the console for the user
        self.get_logger().info(f"Request to play animation {request.animation}")

        # These are the states in which we can play an animation without thinking twice
        animatable_states = [RobotControlState.CONTROLLABLE, RobotControlState.RECORD]

        # Check if we can play the animation right now or if we have to wait
        if self.hcm_state not in animatable_states and not request.hcm:
            # we cant play an animation right now
            # but we send a request, so that we may can soon
            self.send_animation_request()

            # Wait for the hcm to be controllable
            num_tries = 0
            while rclpy.ok() and self.hcm_state not in animatable_states and num_tries < 10:
                num_tries += 1
                self.get_logger().info(f"HCM not controllable. Waiting... (try {num_tries})")
                self.get_clock().sleep_until(self.get_clock().now() + Duration(seconds=0.1))

            if self.hcm_state not in animatable_states:
                self.get_logger().info(
                    "HCM not controllable. Only sent request to make it come controllable, "
                    "but it was not successful until timeout"
                )
                goal.abort()
                return PlayAnimation.Result(successful=False)

        animator = self.get_animation_splines(self.current_animation)

        # Flag that determines that we have send something once
        once = False

        # Loop to play the animation
        while rclpy.ok() and animator:
            try:
                last_time = self.get_clock().now()

                # if we're here we want to play the next keyframe, cause there is no other goal
                # compute next pose
                t = (
                    self.get_clock().now().nanoseconds / 1e9 - animator.get_start_time()
                )  # time since start of animation
                # Start later if we have set bounds and only play the part of the animation
                if request.bounds:
                    t += animator.get_keyframe_times()[request.start]

                self.get_logger().info(f"Playing animation at time {t}")
                self.get_logger().info(f"Using bounds {request.bounds} and start {request.start} and end {request.end}")
                self.get_logger().info(
                    f"Using start time {animator.get_keyframe_times()[request.start]} and end time {animator.get_keyframe_times()[request.end]}"
                )

                # Get the robot pose at the current time from the spline interpolator
                pose = animator.get_positions_rad(t)

                # Finish if spline ends or we reaches the explicitly defined premature end
                assert request.end >= 0 and request.end <= len(animator.get_keyframe_times())
                if pose is None or (request.bounds and once and t > animator.get_keyframe_times()[request.end]):
                    # animation is finished
                    # tell it to the hcm
                    self.send_animation(first=False, last=True, hcm=request.hcm, pose=None, torque=None)
                    goal.publish_feedback(PlayAnimation.Feedback(percent_done=100))
                    # we give a positive result
                    goal.succeed()
                    return PlayAnimation.Result(successful=True)

                self.send_animation(first=first, last=False, hcm=request.hcm, pose=pose, torque=animator.get_torque(t))

                first = False  # we have sent the first frame, all frames after this can't be the first
                perc_done = int(
                    ((self.get_clock().now().nanoseconds / 1e9 - animator.get_start_time()) / animator.get_duration())
                    * 100
                )
                perc_done = max(0, min(perc_done, 100))

                goal.publish_feedback(PlayAnimation.Feedback(percent_done=perc_done))

                once = True

                self.get_clock().sleep_until(last_time + Duration(seconds=0.02))
            except (ExternalShutdownException, KeyboardInterrupt):
                exit()
        return PlayAnimation.Result(successful=False)

    def get_animation_splines(self, animation_name: str):
        if animation_name not in self.animation_cache:
            self.get_logger().error(f"Animation '{animation_name}' not found")
            return
        return SplineAnimator(
            self.animation_cache[animation_name], self.current_joint_states, self.get_logger(), self.get_clock()
        )

    def update_current_pose(self, msg):
        """Gets the current motor positions and updates the representing pose accordingly."""
        self.current_joint_states = msg

    def update_hcm_state(self, msg: RobotControlState):
        self.hcm_state = msg.state

    def send_animation_request(self):
        anim_msg = AnimationMsg()
        anim_msg.request = True
        anim_msg.header.stamp = self.get_clock().now().to_msg()
        self.hcm_publisher.publish(anim_msg)

    def send_animation(self, first: bool, last: bool, hcm: bool, pose: dict, torque: dict):
        """Sends an animation to the hcm"""
        anim_msg = AnimationMsg()
        anim_msg.request = False
        anim_msg.first = first
        anim_msg.last = last
        anim_msg.hcm = hcm
        if pose is not None:  #
            traj_msg = JointTrajectory()
            traj_msg.joint_names = []
            traj_msg.points = [JointTrajectoryPoint()]
            # We are only using a single point in the trajectory message, since we only want to send a single joint goal
            traj_msg.points[0].positions = []
            traj_msg.points[0].effort = []
            for joint in pose:
                traj_msg.joint_names.append(joint)
                traj_msg.points[0].positions.append(pose[joint])
                if torque:
                    # 1 and 2 should be mapped to 1
                    traj_msg.points[0].effort.append(np.clip((torque[joint]), 0, 1))
            anim_msg.position = traj_msg
        anim_msg.header.stamp = self.get_clock().now().to_msg()
        self.hcm_publisher.publish(anim_msg)

    def add_animation(self, request: AddAnimation.Request, response: AddAnimation.Response) -> AddAnimation.Response:
        """
        Adds an animation to the cache (non persistent).
        This is usefull if e.g. the recording GUI wants to play an uncommited animation.
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
