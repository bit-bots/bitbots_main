#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import datetime

# todo
# klickbar für welche motoren werte in den step gespeichert werden
# copy past of frames, from one animation to another
# duplicate frame
# button for playing just current frmae
# button to set robot in init pose
# record button next to frame name filed
import json
import os

import actionlib
import humanoid_league_msgs
import rosparam
import rospy
from copy import deepcopy
from socket import gethostname
import rospkg
from humanoid_league_msgs.msg import PlayAnimationAction


class AnimationData(object):
    """ Defines a current status of the recorded Animation
    """

    def __init__(self):
        self.anim_steps = []
        self.name = "None yet"
        self.version = 0
        self.last_edited = datetime.datetime.isoformat(datetime.datetime.now(), ' ')
        self.author = "Unknown"
        self.last_hostname = "Unknown"
        self.description = "Edit me!"


class Recorder(object):
    """ Recorder Methods are gathered in this class

    :param ipc: Shared Memory provider to set data
    :param gui: urwid-gui responsible for displaying this reocrd-instance
    :param logger:
        the logger to use, defaults to 'record-gui' the logger is
        important for the commuication with the gui-console
    """

    def __init__(self):
        self.steps = []
        self.redo_steps = []
        self.current_state = AnimationData()
        self.anim_client = actionlib.SimpleActionClient('animation', PlayAnimationAction)

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
        """ Save the current state of the Animation
        for later restoration by the undo-command

        (Yes we might save only a diff, but the Memory consumption
        should still be relatively low this way and saving / undoing
        is really cheap in terms of CPU and effort spent programming)

        :param description: A string describing the saved action for the user
        :param state: a AnimState can be given otherwise the current one is used
        """

        rospy.logdebug("Saving step: %s" % description)
        if not state:
            state = deepcopy(self.current_state)
        self.steps.append((state, description))
        self.save_animation("backup")

    def undo(self, amount=1):
        """ Undo <amount> of steps or the last Step if not given
        """
        if amount > len(self.steps):
            rospy.logwarn("I cannot undo what did not happen!")
            return False
        if amount == 1:
            state, description = self.steps.pop()
            if state.anim_steps == self.current_state.anim_steps:
                state, description = self.steps.pop()
            self.redo_steps.append((state, description, self.current_state))
            self.current_state = state
            rospy.loginfo("Undoing: %s" % description)
            if self.steps:
                state, description = self.steps[-1]
                rospy.loginfo("Last noted action: %s" % description)
            else:
                rospy.loginfo("There are no previously noted steps")
            return True
        else:
            rospy.loginfo("Undoing %i steps" % amount)
            state, description = self.steps[-amount]
            self.current_state = state
            self.redo_steps = self.steps[-amount:].reverse()
            self.steps = self.steps[:-amount]
            return True

    def redo(self, amount=1):
        """ Redo <amount> of steps, or the last step if omitted
        """
        post_state = None
        if not self.redo_steps:
            rospy.logwarn("Cannot redo what was not undone!")
            return False
        if amount < 0:
            rospy.logwarn("Amount cannot be negative! (What where you even thinking?)")
            return False
        while amount and self.redo_steps:
            pre_state, description, post_state = self.redo_steps.pop()
            self.steps.append((pre_state, description))
            amount -= 1
        self.current_state = post_state
        rospy.loginfo("Last noted step is now: %s " % self.steps[-1][1])
        return True

    def record(self, motor_pos, frame_name, duration, pause, seq_pos=None, override=False):
        """ Record Command, save current keyframe-data
        """
        frame = {
            "name": frame_name,
            "duration": duration,
            "pause": pause,
            "goals": motor_pos
        }
        new_frame = deepcopy(frame)
        if not seq_pos:
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
        """ Record Command, clear all keyframe-data
        """
        self.save_step("Clearing all keyframe data")
        self.current_state.anim_steps = []
        return True

    def save_animation(self, path, file_name=None, force=False):
        """ Record Command, dump all keyframedata to an animation .json file

        The GUI is asked for validity of the data, because the GUI keeps track
        of that anyway. If the data is not valid, saving the changes is refused
        to avoid putting defect files into our repository or even overriding
        functional ones. Forcing this method to dump results in a '_defective'
        postfix of the filename. The force feature is intended to allow "emergency"
        saves when the Programm itself is forced to exit in an inconsistent state.

        :param file_name: what name the new file should receive
        :param force: set True, for saving even inconsistent states.
        """
        if not self.current_state.anim_steps:
            rospy.loginfo("There is nothing to save.")
            # todo display in rqt
            return False

        if not file_name:
            file_name = self.current_state.name

        if not os.path.isdir(path):
            path = os.path.expanduser('~')
        path = os.path.join(path, file_name + '.json')
        rospy.logdebug("Saving to '%s'" % path)

        anim = {
            "name": self.current_state.name,
            "version": self.current_state.version,
            "last_edited": datetime.datetime.isoformat(datetime.datetime.now(), ' '),
            "author": self.current_state.author,
            "description": self.current_state.description,
            "keyframes": self.current_state.anim_steps,
            "hostname": gethostname()
        }

        with open(path, "w") as fp:
            json.dump(anim, fp, sort_keys=True, indent=4)
        return True

    def remove(self, framenumber=None):
        """ Record Command, remove the last keyframedata

        :param framenumber: The Number of frame to remove. default is last
        """
        if not framenumber:
            if not self.current_state.anim_steps:
                rospy.logwarn("Nothing to revert, framelist is empty!")
                return False
            self.save_step("Reverting the last Keyframe (#%i)" % len(self.current_state.anim_steps))
            self.current_state.anim_steps.pop()
            return True
        else:
            try:
                framenumber = int(framenumber)
            except TypeError:
                rospy.logwarn("Optional framenumber must be Integer! (got %s)" % framenumber)
                return False
            if len(self.current_state.anim_steps) < framenumber:
                rospy.logwarn("Invalid framenumber: %i" % framenumber)
                return False
            self.save_step("Reverting keyframe #%i" % framenumber)
            framenumber -= 1  # Frameindices in the GUI are starting with 1, not 0
            self.current_state.anim_steps.pop(framenumber)
        return True

    def mirror(self, selector, tag):
        """Mirrors a Motor-Group to its opposing motors
        """
        indices = index_select(self.anim_steps, selector)
        if not indices:
            return False

        # temp-safe current state
        state = deepcopy(self.current_state)

        for index in indices:
            if not self.mirror_single_frame(index, tag):
                self.log.warn("Mirror failed on Keyframe %i, reverting!")
                self.current_state = state
                return False

        # finally save after we know the mirror is a success
        self.save_step("Mirror of Motor-Group '%s' in Keyframes %s" % (tag, selector), state)
        self.gui.display_keyframes(self.current_state.anim)
        return True

    def mirror_single_frame(self, framenumber, tag):
        selected = joints.get_joints(tag)
        error = []
        for joint in selected:
            try:
                opposing = joints.get_joint_by_cid(joint.opposing)
                if opposing in selected:
                    error.append((joint, opposing))
            except KeyError:
                self.log.debug("Got a Key error for joint %i - probably there is just no opposing joint." % joint.cid)
        if error:
            err_str = "Cannot Mirror, because the following motors are ambigous: "
            for joint, opposing in error:
                err_str += " " + str(joint.cid) + "<=>" + str(opposing.cid)
            self.log.warn(err_str)
            return False

        try:
            for joint in selected:
                if joint.opposing:
                    opposing_joint = joints.get_joint_by_cid(joint.opposing)
                    new_value = self.current_state.anim_steps[framenumber]['goals'][joint.name]
                    if joint.inverted:
                        new_value *= -1
                    self.current_state.anim_steps[framenumber]["goals"][opposing_joint.name] = new_value
        except KeyError:
            self.log.warning("I am missing a Joint in my animation that should be mirrored, aborting")
            return False
        except IndexError:
            self.log.warning("Keyframe %i does not appear to exist!" % framenumber)
            return False
        return True

    def load_animation(self, path):
        """ Record command, load a animation '.json' file

        :param name: name of the animation to load
        """
        data = []
        with open(path) as fp:
            try:
                data = json.load(fp)
            except ValueError as e:
                rospy.logerr("Animation %s is corrupt:\n %s" %
                             (path, e.message.partition('\n')[0]))

        # Ensure Data retrieval was a success
        if not data:
            return False

        self.save_step("Loading of animation named %s" % path)

        self.current_state.anim_steps = data[u'keyframes']

        # get metadata from the file, if specified
        def get_meta(key, default="Unknown"):
            """ Retrieve the meta-information for given key from data
            :param str key:
            :param default: returned if key does not exist
            :type default: any
            """
            if not key in data:
                msg = "key %s not found in the animation" % key
                rospy.logdebug(msg)
                return default
            return data[key]

        self.current_state.name = get_meta('name', 'NONAME')
        self.current_state.description = get_meta('description', "Edit me!")
        self.current_state.version = get_meta('version', 0)
        self.current_state.last_edited = get_meta('last_edited')
        self.current_state.author = get_meta('author')
        self.current_state.last_hostname = get_meta('hostname')

        rospy.loginfo("Animation with %d frames loaded" % len(self.current_state.anim_steps))
        return True

    def play(self, until_frame=None):
        """ Record command, start playing an animation

        Can play a certain (named) animation or the current one by default.
        Also can play only a part of an animation if *start* and/or *end* are defined

        :param until_frame:
        """

        if not self.current_state.anim_steps:
            rospy.loginfo("Refusing to play, because nothing to play exists!")
            return False
        if not until_frame:
            # play complete animation
            n = len(self.current_state.anim_steps)
        else:
            # play the given number if not higher than current steps
            n = min(until_frame, len(self.current_state.anim_steps))

        anim_dict = {
            "name": "Record-play",
            "keyframes": self.current_state.anim_steps[0:n]
        }

        rospy.loginfo("playing %d frames..." % len(anim_dict['keyframes']))
        return self.execute_play(anim_dict)

    def execute_play(self, anim_dict):
        """ We make a temporary copy of the animation and call the animation play action to play it"""
        anim_package = rosparam.get_param("robot_type_name").lower() + "_animations"
        rospack = rospkg.RosPack()
        path = rospack.get_path(anim_package)
        path = os.path.join(path, "record" + '.json')

        with open(path, "w") as fp:
            json.dump(anim_dict, fp, sort_keys=True, indent=4)

        self.play_animation("record")

    def play_animation(self, name):

        first_try = self.anim_client.wait_for_server(
            rospy.Duration(rospy.get_param("hcm/anim_server_wait_time", 10)))
        if not first_try:
            rospy.logerr(
                "Animation Action Server not running! Will now wait until server is accessible!")
            self.anim_client.wait_for_server()
            rospy.logwarn("Animation server now running, hcm will go on.")
        goal = humanoid_league_msgs.msg.PlayAnimationGoal()
        goal.animation = name
        goal.hcm = True  # force

    def change_frame_order(self, new_order):
        """ Changes the order of the frames given an array of frame names"""
        new_ordered_frames = []
        for frame_name in new_order:
            for frame in self.current_state.anim_steps:
                if frame_name == frame["name"]:
                    new_ordered_frames.append(frame)
        self.current_state.anim_steps = new_ordered_frames
        self.save_step("Reordered frames")

    def duplicate(self, frame_name):
        new_frames = []
        for frame in self.current_state.anim_steps:
            new_frames.append(frame)
            if frame_name == frame["name"]:
                duplicate = deepcopy(frame)
                duplicate["name"] = frame_name + "duplicate"
                new_frames.append(duplicate)
        self.current_state.anim_steps = new_frames
        self.save_step("Duplicated Frame " + frame_name)