#!/usr/bin/env python3
# todo
# copy paste of frames, from one animation to another
# record button next to frame name filed
# set min max for joint value fields
import json
import math
import os
from copy import deepcopy
from datetime import datetime
from typing import Optional

from rclpy.action import ActionClient
from rclpy.node import Node

from bitbots_msgs.action import PlayAnimation
from bitbots_msgs.srv import AddAnimation


class AnimationData:
    """
    Defines a current status of the recorded Animation
    """

    def __init__(self):
        self.key_frames: list[dict] = []
        self.author: str = "Unknown"
        self.description: str = "Edit me!"
        self.last_edited: datetime = datetime.isoformat(datetime.now(), " ")
        self.name: str = "None yet"
        self.version: int = 0


class Recorder:
    def __init__(self, node: Node) -> None:
        self._node = node
        self.steps = []
        self.redo_steps = []
        self.current_state = AnimationData()
        self.animation_client: ActionClient = ActionClient(self._node, PlayAnimation, "animation")
        self.add_animation_client = self._node.create_client(AddAnimation, "add_temporary_animation")
        self.save_step("Initial step")

    def get_keyframes(self) -> list[dict]:
        """
        Gets the keyframes of the current animation

        :return: the keyframes of the current animation
        """
        return self.current_state.key_frames

    def get_keyframe(self, name: str) -> Optional[dict]:
        """
        Gets the keyframe with the given name

        :param name: the name of the keyframe
        :return: the keyframe with the given name
        """
        # Get the first keyframe with the given name or None
        return next(filter(lambda key_frame: key_frame["name"] == name, self.get_keyframes()), None)

    def get_keyframe_index(self, name: str) -> Optional[int]:
        """
        Gets the index of the keyframe with the given name

        :param name: the name of the keyframe
        :return: the index of the keyframe with the given name
        """
        # Get the index of the first keyframe with the given name or None
        key_frame = self.get_keyframe(name)
        if key_frame is None:
            return None
        return self.get_keyframes().index(key_frame)

    def get_meta_data(self) -> tuple[str, int, str, str]:
        """
        Returns the meta data of the current animation
        """
        data = self.current_state
        return data.name, data.version, data.author, data.description

    def set_meta_data(self, name: str, version: int, author: str, description: str) -> None:
        self.current_state.name = name
        self.current_state.version = version
        self.current_state.author = author
        self.current_state.description = description

    def save_step(self, description, state=None):
        """Save the current state of the Animation
        for later restoration by the undo-command

        (Yes we might save only a diff, but the Memory consumption
        should still be relatively low this way and saving / undoing
        is really cheap in terms of CPU and effort spent programming)

        :param description: A string describing the saved action for the user
        :param state: a AnimState can be given otherwise the current one is used
        """

        self._node.get_logger().debug(f"Saving step: {description}")
        if not state:
            state = deepcopy(self.current_state)
        self.steps.append((state, description))

    def undo(self, amount: int = 1) -> str:
        """Undo <amount> of steps or the last Step if omitted"""
        if amount > len(self.steps) or self.steps[-1][1] == "Initial step":
            self._node.get_logger().warn("I cannot undo what did not happen!")
            return "I cannot undo what did not happen!"
        if amount == 1:
            state, description = self.steps.pop()
            self.redo_steps.append((state, description, self.current_state))
            self._node.get_logger().info(f"Undoing: {description}")
            if self.steps:
                state, description = self.steps[-1]
                self.current_state = state
                self._node.get_logger().info(f"Last noted action: {description}")
                return f"Undoing. Last noted action: {description}"
            else:
                self._node.get_logger().info("There are no previously noted steps")
                return "Undoing. There are no more previous steps."
        else:
            self._node.get_logger().info(f"Undoing {amount} steps")
            state, description = self.steps[-amount]
            self.current_state = state
            self.redo_steps = self.steps[-amount:].reverse()
            self.steps = self.steps[:-amount]
            return f"Undoing {amount} steps"

    def redo(self, amount: int = 1) -> str:
        """Redo <amount> of steps, or the last step if omitted"""
        post_state = None
        if not self.redo_steps:
            self._node.get_logger().warn("Cannot redo what was not undone!")
            return "Cannot redo what was not undone!"
        if amount < 0:
            self._node.get_logger().warn("Amount cannot be negative! (What where you even thinking?)")
            return "Amount cannot be negative! (What where you even thinking?)"
        while amount and self.redo_steps:
            pre_state, description, post_state = self.redo_steps.pop()
            self.steps.append((pre_state, description))
            amount -= 1
        self.current_state = post_state
        self._node.get_logger().info(f"Last noted step is now: {self.steps[-1][1]}")
        return f"Last noted step is now: {self.steps[-1][1]}"

    def record(
        self,
        motor_pos: dict[str, float],
        motor_torque: dict[str, int],  # TODO: check if we can use bool instead of int
        frame_name: str,
        duration: float,
        pause: float,
        seq_pos: Optional[int] = None,
        override: bool = False,
    ):
        """Record Command, save current keyframe-data

        :param motor_pos: A position for each motor we want to control
        :param motor_torque: Wether or not the motor should apply torque
        :param frame_name: The name of the frame
        :param duration: The duration of the frame
        :param pause: We pause for this amount of time after the frame
        :param seq_pos: The position in the sequence where the frame should be inserted
        :param override: Wether or not to override the frame at the given position
        """
        frame = {"name": frame_name, "duration": duration, "pause": pause, "goals": motor_pos, "torque": motor_torque}
        new_frame = deepcopy(frame)
        if seq_pos is None:
            self.current_state.key_frames.append(new_frame)
            self.save_step(f"Appending new keyframe '{frame_name}'")
        elif not override:
            self.current_state.key_frames.insert(seq_pos, new_frame)
            self.save_step(f"Inserting new keyframe '{frame_name}' at position {seq_pos}")
        else:
            self.current_state.key_frames[seq_pos] = new_frame
            self.save_step(f"Overriding keyframe at position {seq_pos} with '{frame_name}'")
        return True

    def save_animation(self, path: str, file_name: Optional[str] = None) -> None:
        """Dumps all keyframe data to an animation .json file

        :param path: The folder where the file should be saved
        :param file_name: The name of the file
        """
        if not self.current_state.key_frames:
            self._node.get_logger().info("There is nothing to save.")
            return "There is nothing to save."

        # Use the animation name as file name if none is given
        if not file_name:
            file_name = self.current_state.name

        path = os.path.join(path, f"{file_name}.json")
        self._node.get_logger().debug(f"Saving to '{path}'")

        # Convert the angles from radians to degrees
        keyframes = deepcopy(self.current_state.key_frames)
        for kf in keyframes:
            for k, v in kf["goals"].items():
                kf["goals"][k] = math.degrees(v)

        # Construct the animation dictionary with meta data and keyframes
        animation_dict = {
            "name": self.current_state.name,
            "version": self.current_state.version,
            "last_edited": datetime.isoformat(datetime.now(), " "),
            "author": self.current_state.author,
            "description": self.current_state.description,
            "keyframes": keyframes,
        }

        # Save the animation to a file
        with open(path, "w") as fp:
            json.dump(animation_dict, fp, sort_keys=True, indent=4)

        return "Saving to '%s'" % path + ". Done."

    def load_animation(self, path: str) -> None:
        """Record command, load a animation '.json' file

        :param path: path of the animation to load
        """
        # Load the json file
        with open(path) as fp:
            data = json.load(fp)
        # Check if the file is a valid animation and convert the angles from degrees to radians
        try:
            for keyframe in data["keyframes"]:
                for k, v in keyframe["goals"].items():
                    keyframe["goals"][k] = math.radians(v)
        except ValueError as e:
            self._node.get_logger().error("Animation {} is corrupt:\n {}".format(path, e.message.partition("\n")[0]))
            return "Animation {} is corrupt:\n {}".format(path, e.message.partition("\n")[0])

        # Set the current state to the loaded animation
        self.current_state.key_frames = data["keyframes"]

        # Save the current state
        self.save_step(f"Loading of animation named '{data['name']}'")

    def play(self, until_frame: Optional[int] = None) -> str:
        """Record command, start playing an animation

        Can play a certain (named) animation or the current one by default.
        Also can play only a part of an animation if end is defined

        :param until_frame: the frame until which the animation should be played
        """
        if not self.current_state.key_frames:
            msg = "Refusing to play, because nothing to play exists!"
            self._node.get_logger().warn(msg)
            return msg

        if until_frame is None:
            # Play complete animation
            until_frame = len(self.current_state.key_frames)
        else:
            # Check if the given frame id is in bounds
            assert until_frame > 0, "Upper bound must be positive"
            assert until_frame <= len(
                self.current_state.key_frames
            ), "Upper bound must be less than or equal to the number of frames"

        # Create name for the temporary animation that is send to the animation server
        # We can call this animation by name to play it
        # We do not want to overwrite the current animation
        tmp_animation_name = f"{self.current_state.name}_tmp"

        # Create a dictionary with the animation data
        animation_dict = {
            "name": tmp_animation_name,
            "version": self.current_state.version,
            "last_edited": datetime.isoformat(datetime.now(), " "),
            "author": self.current_state.author,
            "description": self.current_state.description,
            "keyframes": deepcopy(self.current_state.key_frames[0:until_frame]),
        }

        # Convert the angles from radians to degrees
        for key_frame in animation_dict["keyframes"]:
            for joint, v in key_frame["goals"].items():
                key_frame["goals"][joint] = math.degrees(v)

        self._node.get_logger().debug(f"Playing {len(animation_dict['keyframes'])} frames...")

        # Wait for the service to be available before sending the animation
        if not self.add_animation_client.wait_for_service(timeout_sec=2):
            self._node.get_logger().error("Add Animation Service not available! Is the animation server running?")
            return "Add Animation Service not available! Is the animation server running?"

        # Create a request to add the animation
        request = AddAnimation.Request()
        request.json = json.dumps(animation_dict, sort_keys=True, indent=4)

        # Send the request to the service (blocking to make sure the animation is added before playing it)
        self.add_animation_client.call(request)

        # Wait for animation action server to be available
        if not self.animation_client.wait_for_server(timeout_sec=2):
            self._node.get_logger().error("Animation Action Server not available! Is the animation server running?")
            return "Animation Action Server not available! Is the animation server running?"

        # Create a goal to play the animation
        goal = PlayAnimation.Goal()
        goal.animation = tmp_animation_name
        goal.hcm = True  # force TODO check that
        self.animation_client.send_goal_async(goal)  # TODO maybe handle result or status

        return f"Playing {len(animation_dict['keyframes'])} frames..."

    def change_frame_order(self, new_order: list[str]) -> None:
        """Changes the order of the frames given an array of frame names"""
        new_ordered_frames = [self.get_keyframe(frame_name) for frame_name in new_order]
        assert None not in new_ordered_frames, "One of the given frame names does not exist"
        self.current_state.key_frames = new_ordered_frames
        self.save_step("Reordered frames")

    def duplicate(self, frame_name: str) -> None:
        """
        Duplicates a frame
        """
        index = self.get_keyframe_index(frame_name)
        assert index is not None, "The given frame name does not exist"
        self.current_state.key_frames.insert(index + 1, deepcopy(self.get_keyframe(frame_name)))
        self.save_step(f"Duplicated Frame '{frame_name}'")

    def delete(self, frame_name: str) -> None:
        """
        Deletes a frame
        """
        index = self.get_keyframe_index(frame_name)
        assert index is not None, "The given frame name does not exist"
        self.current_state.key_frames.pop(index)
        self.save_step(f"Deleted Frame '{frame_name}'")
