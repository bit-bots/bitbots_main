#!/usr/bin/env python3
import datetime

# todo
# copy paste of frames, from one animation to another
# record button next to frame name filed
# set min max for joint value fields
import json
import math
import os
from copy import deepcopy

from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

from bitbots_msgs.action import PlayAnimation


class AnimationData:
    """
    Defines a current status of the recorded Animation
    """

    def __init__(self):
        self.anim_steps = []
        self.name = "None yet"
        self.version = 0
        self.last_edited = datetime.datetime.isoformat(datetime.datetime.now(), " ")
        self.author = "Unknown"
        self.description = "Edit me!"


class Recorder:
    def __init__(self, node: Node):
        self._node = node
        self.steps = []
        self.redo_steps = []
        self.current_state = AnimationData()
        self.animation_client: ActionClient = ActionClient(self._node, PlayAnimation, "animation")
        self.save_step("Initial step")

    def get_animation_state(self):
        return self.current_state.anim_steps

    def get_meta_data(self):
        data = self.current_state
        return data.name, data.version, data.author, data.description

    def set_meta_data(self, name, version, author, description):
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

        self._node.get_logger().debug("Saving step: %s" % description)
        if not state:
            state = deepcopy(self.current_state)
        self.steps.append((state, description))
        self.save_animation("backup")

    def undo(self, amount=1):
        """Undo <amount> of steps or the last Step if omitted"""
        if amount > len(self.steps) or self.steps[-1][1] == "Initial step":
            self._node.get_logger().warn("I cannot undo what did not happen!")
            return "I cannot undo what did not happen!"
        if amount == 1:
            state, description = self.steps.pop()
            self.redo_steps.append((state, description, self.current_state))
            self._node.get_logger().info("Undoing: %s" % description)
            if self.steps:
                state, description = self.steps[-1]
                self.current_state = state
                self._node.get_logger().info("Last noted action: %s" % description)
                return "Undoing. Last noted action: %s" % description
            else:
                self._node.get_logger().info("There are no previously noted steps")
                return "Undoing. There are no more previous steps."
        else:
            self._node.get_logger().info("Undoing %i steps" % amount)
            state, description = self.steps[-amount]
            self.current_state = state
            self.redo_steps = self.steps[-amount:].reverse()
            self.steps = self.steps[:-amount]
            return "Undoing %i steps" % amount

    def redo(self, amount=1):
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
        self._node.get_logger().info("Last noted step is now: %s " % self.steps[-1][1])
        return "Last noted step is now: %s " % self.steps[-1][1]

    def record(self, motor_pos, motor_torque, frame_name, duration, pause, seq_pos=None, override=False):
        """Record Command, save current keyframe-data"""
        frame = {"name": frame_name, "duration": duration, "pause": pause, "goals": motor_pos, "torque": motor_torque}
        new_frame = deepcopy(frame)
        if seq_pos is None:
            self.current_state.anim_steps.append(new_frame)
            self.save_step("Appending new keyframe " + frame_name)
        elif not override:
            self.current_state.anim_steps.insert(seq_pos, new_frame)
            self.save_step("Inserting new keyframe " + frame_name + " to position " + str(seq_pos))
        else:
            self.current_state.anim_steps[seq_pos] = new_frame
            self.save_step("overriding keyframe " + frame_name + " at position " + str(seq_pos))
        return True

    def clear(self):
        """Record Command, clear all keyframe-data"""
        newsteps = []
        for i in self.steps:
            if i[1] == "Initial step":
                newsteps.append(i)
        self.steps = deepcopy(newsteps)
        self.current_state.anim_steps = []
        return True

    def save_animation(self, path, file_name=None, save_checkbox=None):
        """Record Command, dump all keyframedata to an animation .json file

        :param file_name: what name the new file should receive
        """
        if not self.current_state.anim_steps:
            self._node.get_logger().info("There is nothing to save.")
            return "There is nothing to save."

        if not file_name:
            file_name = self.current_state.name

        if not os.path.isdir(path):
            path = os.path.expanduser("~")
        path = os.path.join(path, file_name + ".json")
        self._node.get_logger().debug("Saving to '%s'" % path)

        saved_keyframes = deepcopy(self.current_state.anim_steps)

        for kf in saved_keyframes:
            if save_checkbox is not None:
                if kf["name"] in save_checkbox:
                    for k, v in save_checkbox[kf["name"]].items():
                        if v == 0:
                            kf["goals"].pop(k)

        anim = {
            "name": self.current_state.name,
            "version": self.current_state.version,
            "last_edited": datetime.datetime.isoformat(datetime.datetime.now(), " "),
            "author": self.current_state.author,
            "description": self.current_state.description,
            "keyframes": saved_keyframes,
        }

        for kf in anim["keyframes"]:
            for k, v in kf["goals"].items():
                kf["goals"][k] = int(math.degrees(v))

        with open(path, "w") as fp:
            json.dump(anim, fp, sort_keys=True, indent=4)
        return "Saving to '%s'" % path + ". Done."

    def remove(self, framenumber=None):
        """Record Command, remove the last keyframedata

        :param framenumber: The Number of frame to remove. default is last
        """
        if not framenumber:
            if not self.current_state.anim_steps:
                self._node.get_logger().warn("Nothing to revert, framelist is empty!")
                return False
            self.save_step("Reverting the last Keyframe (#%i)" % len(self.current_state.anim_steps))
            self.current_state.anim_steps.pop()
            return True
        else:
            try:
                framenumber = int(framenumber)
            except TypeError:
                self._node.get_logger().warn("Optional framenumber must be Integer! (got %s)" % framenumber)
                return False
            if len(self.current_state.anim_steps) < framenumber:
                self._node.get_logger().warn("Invalid framenumber: %i" % framenumber)
                return False
            self.save_step("Reverting keyframe #%i" % framenumber)
            framenumber -= 1  # Frameindices in the GUI are starting with 1, not 0
            self.current_state.anim_steps.pop(framenumber)
        return True

    def load_animation(self, path):
        """Record command, load a animation '.json' file

        :param path: path of the animation to load
        """
        data = []
        with open(path) as fp:
            try:
                data = json.load(fp)
                i = 0
                for kf in data["keyframes"]:
                    if "name" not in kf:
                        kf["name"] = "frame" + str(i)
                        i += 1
                for kf in data["keyframes"]:
                    for k, v in kf["goals"].items():
                        kf["goals"][k] = math.radians(v)
            except ValueError as e:
                self._node.get_logger().error(
                    "Animation {} is corrupt:\n {}".format(path, e.message.partition("\n")[0])
                )
                return "Animation {} is corrupt:\n {}".format(path, e.message.partition("\n")[0])

        # Ensure Data retrieval was a success
        if not data:
            return False

        self.save_step("Loading of animation named %s" % path)

        self.current_state.anim_steps = data["keyframes"]

    def play(self, anim_path, until_frame=None):
        """Record command, start playing an animation

        Can play a certain (named) animation or the current one by default.
        Also can play only a part of an animation if end is defined

        :param until_frame: the frame until which the animation should be played
        """
        if not self.current_state.anim_steps:
            self._node.get_logger().info("Refusing to play, because nothing to play exists!")
            return "Refusing to play, because nothing to play exists!"
        if not until_frame:
            # play complete animation
            n = len(self.current_state.anim_steps)
        else:
            # play the given number if not higher than current steps
            n = min(until_frame, len(self.current_state.anim_steps))

        anim_dict = {"name": "Record-play", "keyframes": deepcopy(self.current_state.anim_steps[0:n])}
        for kf in anim_dict["keyframes"]:
            for k, v in kf["goals"].items():
                kf["goals"][k] = int(math.degrees(v))

        self._node.get_logger().info("playing %d frames..." % len(anim_dict["keyframes"]))

        path = os.path.join(anim_path, f"record_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.json")

        with open(path, "w") as fp:
            json.dump(anim_dict, fp, sort_keys=True, indent=4)

        self._node.get_logger().info("Storing animation in %s" % path)

        # TODO send animation to animation server using service

        self.play_animation("record")

        return "playing %d frames..." % len(anim_dict["keyframes"])

    def play_animation(self, name):
        """Sends the animation to the animation server"""
        first_try = self.animation_client.wait_for_server(Duration(seconds=10))
        if not first_try:
            self._node.get_logger().error(
                "Animation Action Server not running! Will now wait until server is accessible!"
            )
            self.animation_client.wait_for_server()
            self._node.get_logger().warn("Animation server now running, hcm will go on.")
        goal = PlayAnimation.Goal()
        goal.animation = name
        goal.hcm = True  # force TODO check that
        self.animation_client.send_goal_async(goal)  # TODO maybe handle result or status

    def change_frame_order(self, new_order):
        """Changes the order of the frames given an array of frame names"""
        new_ordered_frames = []
        for frame_name in new_order:
            for frame in self.current_state.anim_steps:
                if frame_name == frame["name"]:
                    new_ordered_frames.append(frame)

        self.current_state.anim_steps = new_ordered_frames
        self.save_step("Reordered frames")

    def duplicate(self, frame_name):
        """
        Duplicates a frame
        """
        new_frames = []
        for frame in self.current_state.anim_steps:
            new_frames.append(frame)
            if frame_name == frame["name"]:
                duplicate = deepcopy(frame)
                newname = frame_name + "d"
                x = True
                n = 0
                for frame in self.current_state.anim_steps:
                    if newname == frame["name"]:
                        while x:
                            x = False
                            for frame in self.current_state.anim_steps:
                                if newname + str(n) == frame["name"]:
                                    n += 1
                                    x = True
                        newname = newname + str(n)

                duplicate["name"] = newname
                new_frames.append(duplicate)
        self.current_state.anim_steps = new_frames
        self.save_step("Duplicated Frame " + frame_name)

    def delete(self, frame_name):
        """
        Deletes a frame
        """
        new_frames = []
        for frame in self.current_state.anim_steps:
            if not frame_name == frame["name"]:
                new_frames.append(frame)
        self.current_state.anim_steps = new_frames
        self.save_step("Duplicated Frame " + frame_name)
