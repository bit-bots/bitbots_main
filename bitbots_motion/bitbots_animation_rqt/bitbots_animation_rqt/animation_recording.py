#!/usr/bin/env python3

import json
import math
import os
from copy import deepcopy
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Optional

import regex as re
from rclpy.action import ActionClient
from rclpy.client import Client as ServiceClient
from rclpy.node import Node
from std_srvs.srv import SetBool

from bitbots_msgs.action import PlayAnimation
from bitbots_msgs.srv import AddAnimation


@dataclass
class Keyframe:
    """
    Defines a keyframe of the recorded Animation
    """

    name: str
    duration: float
    pause: float
    goals: dict[str, float]
    torque: dict[str, int]

    def as_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "duration": self.duration,
            "pause": self.pause,
            "goals": self.goals,
            "torque": self.torque,
        }


@dataclass
class AnimationData:
    """
    Defines a current status of the recorded Animation
    """

    author: str = "Unknown"
    key_frames: list[Keyframe] = field(default_factory=list)
    last_edited: str = datetime.now().isoformat(" ")
    description: str = "Edit me!"
    version: str = "0"
    name: str = "None yet"


class Recorder:
    def __init__(
        self,
        node: Node,
        animation_action_client: ActionClient,
        add_animation_client: ServiceClient,
        hcm_record_mode_client: ServiceClient,
    ):
        self._node = node
        self.steps: list[tuple[AnimationData, str, bool]] = []
        self.redo_steps: list[tuple[AnimationData, str, AnimationData]] = []
        self.current_state = AnimationData()
        self.animation_client = animation_action_client
        self.add_animation_client = add_animation_client
        self.hcm_record_mode_client = hcm_record_mode_client
        self.save_step("Initial step", frozen=True)

    def get_keyframes(self) -> list[Keyframe]:
        """
        Gets the keyframes of the current animation

        :return: the keyframes of the current animation
        """
        return self.current_state.key_frames

    def get_keyframe(self, name: str) -> Optional[Keyframe]:
        """
        Gets the keyframe with the given name

        :param name: the name of the keyframe
        :return: the keyframe with the given name
        """
        # Get the first keyframe with the given name or None
        return next(filter(lambda key_frame: key_frame.name == name, self.get_keyframes()), None)  # type: ignore[arg-type, union-attr]

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

    def get_meta_data(self) -> tuple[str, str, str, str]:
        """
        Returns the meta data of the current animation
        """
        data = self.current_state
        return data.name, data.version, data.author, data.description

    def set_meta_data(self, name: str, version: str, author: str, description: str) -> None:
        self.current_state.name = name
        self.current_state.version = version
        self.current_state.author = author
        self.current_state.description = description

    def save_step(self, description: str, state: Optional[AnimationData] = None, frozen: bool = False) -> None:
        """Save the current state of the Animation
        for later restoration by the undo-command

        (Yes we might save only a diff, but the Memory consumption
        should still be relatively low this way and saving / undoing
        is really cheap in terms of CPU and effort spent programming)

        :param description: A string describing the saved action for the user
        :param state: a AnimState can be given otherwise the current one is used
        :param frozen: if True, the step cannot be undone
        """

        self._node.get_logger().debug(f"Saving step: {description}")
        if not state:
            state = deepcopy(self.current_state)
        self.steps.append((state, description, frozen))

    def undo(self) -> str:
        """Undo the last Step if omitted"""
        if self.steps[-1][2]:
            self._node.get_logger().warn("Reaching frozen step (e.g. the initial state). Cannot undo further!")
            return "Reaching frozen step (e.g. the initial state). Cannot undo further!"
        state, description, _ = self.steps.pop()
        self.redo_steps.append((state, description, self.current_state))
        self._node.get_logger().info(f"Undoing: {description}")
        if self.steps:
            state, description, _ = self.steps[-1]
            self.current_state = state
            self._node.get_logger().info(f"Last noted action: {description}")
            return f"Undoing. Last noted action: {description}"
        else:
            self._node.get_logger().info("There are no previously noted steps")
            return "Undoing. There are no more previous steps."

    def redo(self) -> str:
        """Redo the last step if omitted"""
        post_state = None
        if not self.redo_steps:
            self._node.get_logger().warn("Cannot redo what was not undone!")
            return "Cannot redo what was not undone!"
        pre_state, description, post_state = self.redo_steps.pop()
        self.steps.append((pre_state, description, False))
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
        frozen: bool = False,
    ):
        """Record Command, save current keyframe-data

        :param motor_pos: A position for each motor we want to control
        :param motor_torque: Wether or not the motor should apply torque
        :param frame_name: The name of the frame
        :param duration: The duration of the frame
        :param pause: We pause for this amount of time after the frame
        :param seq_pos: The position in the sequence where the frame should be inserted
        :param override: Wether or not to override the frame at the given position
        :param frozen: if True, the step cannot be undone
        """
        new_frame = deepcopy(
            Keyframe(
                name=frame_name,
                duration=duration,
                pause=pause,
                goals=motor_pos,
                torque=motor_torque,
            )
        )
        if seq_pos is None:
            self.current_state.key_frames.append(new_frame)
            self.save_step(f"Appending new keyframe '{frame_name}'", frozen=frozen)
        elif not override:
            self.current_state.key_frames.insert(seq_pos, new_frame)
            self.save_step(f"Inserting new keyframe '{frame_name}' at position {seq_pos}", frozen=frozen)
        else:
            self.current_state.key_frames[seq_pos] = new_frame
            self.save_step(f"Overriding keyframe at position {seq_pos} with '{frame_name}'", frozen=frozen)

    def save_animation(self, path: str, file_name: Optional[str] = None) -> str:
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
            for k, v in kf.goals.items():
                kf.goals[k] = math.degrees(v)

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

    def load_animation(self, path: str) -> Optional[str]:
        """Record command, load a animation '.json' file

        :param path: path of the animation to load
        """
        # Load the json file
        with open(path) as fp:
            data = json.load(fp)
        # Check if the file is a valid animation and convert the angles from degrees to radians
        try:
            keyframes: list[Keyframe] = []
            for keyframe in data["keyframes"]:
                keyframes.append(
                    Keyframe(
                        name=keyframe["name"],
                        duration=keyframe["duration"],
                        pause=keyframe["pause"],
                        goals={k: math.radians(v) for k, v in keyframe["goals"].items()},
                        torque=keyframe["torque"],
                    )
                )
        except ValueError as e:
            self._node.get_logger().error("Animation {} is corrupt:\n {}".format(path, e.message.partition("\n")[0]))
            return "Animation {} is corrupt:\n {}".format(path, e.message.partition("\n")[0])

        # Set the current state to the loaded animation
        self.current_state = AnimationData(
            key_frames=keyframes,
            name=data["name"],
            version=data["version"] if "version" in data else "0",
            last_edited=data["last_edited"] if "last_edited" in data else datetime.now().isoformat(" "),
            author=data["author"] if "author" in data else "Unknown",
            description=data["description"] if "description" in data else "",
        )

        # Save the current state
        self.save_step(f"Loading of animation named '{data['name']}'")
        return None

    def play(self, from_frame: int = 0, until_frame: int = -1) -> tuple[str, bool]:
        """Plays (a range of) the current animation using the animation server

        :param from_frame: the keyframe from which the animation should be played
        :param until_frame: the keyframe until which the animation should be played
        """
        if not self.current_state.key_frames:
            msg = "Refusing to play, because nothing to play exists!"
            self._node.get_logger().warn(msg)
            return msg, False

        if until_frame == -1:
            # Play complete animation
            until_frame = len(self.current_state.key_frames)
        else:
            # Check if the given frame id is in bounds
            assert until_frame > 0, "Upper bound must be positive"
            assert until_frame <= len(
                self.current_state.key_frames
            ), "Upper bound must be less than or equal to the number of frames"
        assert from_frame >= 0, "Lower bound must be positive"

        # Create name for the temporary animation that is send to the animation server
        # We can call this animation by name to play it
        # We do not want to overwrite the current animation
        tmp_animation_name = "bitbots_animation_rqt_tmp"

        # Create a dictionary with the animation data
        animation_dict = {
            "name": tmp_animation_name,
            "version": self.current_state.version,
            "last_edited": datetime.isoformat(datetime.now(), " "),
            "author": self.current_state.author,
            "description": self.current_state.description,
            "keyframes": deepcopy(list(map(Keyframe.as_dict, self.current_state.key_frames))),
        }

        # Convert the angles from radians to degrees
        for key_frame in animation_dict["keyframes"]:
            for joint, v in key_frame["goals"].items():  # type: ignore[index]
                key_frame["goals"][joint] = math.degrees(v)  # type: ignore[index]

        self._node.get_logger().debug(f"Playing {len(animation_dict['keyframes'])} frames...")

        # Wait for the service to be available before sending the animation
        if not self.add_animation_client.wait_for_service(timeout_sec=2):
            self._node.get_logger().error("Add Animation Service not available! Is the animation server running?")
            return "Add Animation Service not available! Is the animation server running?", False

        # Send request to the service (blocking to make sure the animation is added before playing it)
        self.add_animation_client.call(AddAnimation.Request(json=json.dumps(animation_dict, sort_keys=True, indent=4)))

        # Set the HCM into record mode
        if self.hcm_record_mode_client.wait_for_service(timeout_sec=2):
            self.hcm_record_mode_client.call(SetBool.Request(data=True))
        else:
            self._node.get_logger().error(
                "HCM Record Mode Service not available! Is the HCM running? The animation might not be played if the HCM is not in record mode."
            )

        # Wait for animation action server to be available
        if not self.animation_client.wait_for_server(timeout_sec=2):
            self._node.get_logger().error("Animation Action Server not available! Is the animation server running?")
            return "Animation Action Server not available! Is the animation server running?", False

        # Send a goal to play the animation
        self.animation_client.send_goal_async(
            PlayAnimation.Goal(animation=tmp_animation_name, bounds=True, start=from_frame, end=until_frame)
        )  # TODO maybe blocking or something else

        return f"Playing {len(animation_dict['keyframes'])} frames...", True

    def change_frame_order(self, new_order: list[str]) -> None:
        """Changes the order of the frames given an array of frame names"""
        new_ordered_frames = [self.get_keyframe(frame_name) for frame_name in new_order]
        assert None not in new_ordered_frames, "One of the given frame names does not exist"
        self.current_state.key_frames = new_ordered_frames  # type: ignore[assignment]
        self.save_step("Reordered frames")

    def duplicate(self, frame_name: str) -> None:
        """
        Duplicates a frame
        """
        # Get the index of the frame
        index = self.get_keyframe_index(frame_name)
        assert index is not None, "The given frame name does not exist"
        # Create a deep copy of the frame
        new_frame = deepcopy(self.get_keyframe(frame_name))
        assert new_frame is not None, "The given frame name does not exist (even less possible here)"
        # Add suffix to the frame name if it already exists and increment the number instead of making it longer
        while new_frame.name in [frame.name for frame in self.current_state.key_frames]:
            # Check if the frame name ends with "_copy_" and a number, if so increment the number
            if re.match(r".*?_copy_(\d+)", new_frame.name):
                new_frame.name = re.sub(
                    r"(_copy_)(\d+)", lambda m: f"{m.group(1)}{int(m.group(2)) + 1}", new_frame.name
                )
            else:
                new_frame.name = f"{frame_name}_copy_1"
        # Insert the new frame after the original frame
        self.current_state.key_frames.insert(index + 1, new_frame)
        self.save_step(f"Duplicated Frame '{frame_name}'")

    def delete(self, frame_name: str) -> None:
        """
        Deletes a frame
        """
        index = self.get_keyframe_index(frame_name)
        assert index is not None, "The given frame name does not exist"
        self.current_state.key_frames.pop(index)
        self.save_step(f"Deleted Frame '{frame_name}'")
