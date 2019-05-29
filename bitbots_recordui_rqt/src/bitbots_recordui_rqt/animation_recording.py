#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import datetime

# todo
# copy past of frames, from one animation to another
# record button next to frame name filed
# set min max for joint value fields

import json
import os
import math

import actionlib
import humanoid_league_msgs
import rosparam
from rosgraph import MasterException
import rospy
from copy import deepcopy
from socket import gethostname
import rospkg
from humanoid_league_msgs.msg import PlayAnimationAction

import subprocess


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
        self.save_step('Initial step')

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
        if amount > len(self.steps) or self.steps[-1][1] == "Initial step":
            rospy.logwarn("I cannot undo what did not happen!")
            return "I cannot undo what did not happen!"
        if amount == 1:
            state, description = self.steps.pop()
            #if state.anim_steps == self.current_state.anim_steps:
            #    state, description = self.steps.pop()
            self.redo_steps.append((state, description, self.current_state))
            rospy.loginfo("Undoing: %s" % description)
            if self.steps:
                state, description = self.steps[-1]
                self.current_state = state
                rospy.loginfo("Last noted action: %s" % description)
                return "Undoing. Last noted action: %s" % description
            else:
                rospy.loginfo("There are no previously noted steps")
                return "Undoing. There are no more previous steps."
        else:
            rospy.loginfo("Undoing %i steps" % amount)
            state, description = self.steps[-amount]
            self.current_state = state
            self.redo_steps = self.steps[-amount:].reverse()
            self.steps = self.steps[:-amount]
            return "Undoing %i steps" % amount

    def redo(self, amount=1):
        """ Redo <amount> of steps, or the last step if omitted
        """
        post_state = None
        if not self.redo_steps:
            rospy.logwarn("Cannot redo what was not undone!")
            return "Cannot redo what was not undone!"
        if amount < 0:
            rospy.logwarn("Amount cannot be negative! (What where you even thinking?)")
            return "Amount cannot be negative! (What where you even thinking?)"
        while amount and self.redo_steps:
            pre_state, description, post_state = self.redo_steps.pop()
            self.steps.append((pre_state, description))
            amount -= 1
        self.current_state = post_state
        rospy.loginfo("Last noted step is now: %s " % self.steps[-1][1])
        return "Last noted step is now: %s " % self.steps[-1][1]

    def record(self, motor_pos, motor_torque, frame_name, duration, pause, seq_pos=None, override=False):
        """ Record Command, save current keyframe-data
        """
        frame = {
            "name": frame_name,
            "duration": duration,
            "pause": pause,
            "goals": motor_pos,
            "torque": motor_torque
        }
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
        """ Record Command, clear all keyframe-data
        """
        newsteps = []
        for i in self.steps:
            if i[1] == 'Initial step':
                newsteps.append(i)
        self.steps = deepcopy(newsteps)
        #self.save_step("Clearing all keyframe data")
        self.current_state.anim_steps = []
        return True

    def save_animation(self, path, file_name=None, save_checkbox=None, force=False):
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
            return "There is nothing to save."

        if not file_name:
            file_name = self.current_state.name

        if not os.path.isdir(path):
            path = os.path.expanduser('~')
        path = os.path.join(path, file_name + '.json')
        rospy.logdebug("Saving to '%s'" % path)

        savedKeyframes = deepcopy(self.current_state.anim_steps)

        for kf in savedKeyframes:
            if not save_checkbox == None:
                if kf["name"] in save_checkbox:
                    for k, v in save_checkbox[kf["name"]].items():
                        if v == 0:
                            kf["goals"].pop(k)

        anim = {
            "name": self.current_state.name,
            "version": self.current_state.version,
            "last_edited": datetime.datetime.isoformat(datetime.datetime.now(), ' '),
            "author": self.current_state.author,
            "description": self.current_state.description,
            "keyframes": savedKeyframes,
            "hostname": gethostname()
        }

        for kf in anim['keyframes']:
            for k, v in kf['goals'].iteritems():
                kf['goals'][k] = int(math.degrees(v))

        with open(path, "w") as fp:
            json.dump(anim, fp, sort_keys=True, indent=4)
        return ("Saving to '%s'" % path + ". Done.")

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

    def load_animation(self, path):
        """ Record command, load a animation '.json' file
        :param name: name of the animation to load
        """
        data = []
        with open(path) as fp:
            try:
                data = json.load(fp)
                i = 0
                for kf in data['keyframes']:
                    if not 'name' in kf:
                        kf['name'] = 'frame'+str(i)
                        i += 1
                for kf in data['keyframes']:
                    for k, v in kf['goals'].iteritems():
                        kf['goals'][k] = math.radians(v)
            except ValueError as e:
                rospy.logerr("Animation %s is corrupt:\n %s" %
                             (path, e.message.partition('\n')[0]))
                return ("Animation %s is corrupt:\n %s" % (path, e.message.partition('\n')[0]))


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

    def play(self, anim_path, until_frame=None):
        """ Record command, start playing an animation
        Can play a certain (named) animation or the current one by default.
        Also can play only a part of an animation if *start* and/or *end* are defined
        :param until_frame:
        """
        try:
            if not self.current_state.anim_steps:
                rospy.loginfo("Refusing to play, because nothing to play exists!")
                return "Refusing to play, because nothing to play exists!"
            if not until_frame:
                # play complete animation
                n = len(self.current_state.anim_steps)
            else:
                # play the given number if not higher than current steps
                n = min(until_frame, len(self.current_state.anim_steps))

            anim_dict = {
                "name": "Record-play",
                "keyframes": deepcopy(self.current_state.anim_steps[0:n])
            }
            for kf in anim_dict['keyframes']:
                for k, v in kf['goals'].iteritems():
                    kf['goals'][k] = int(math.degrees(v))

            rospy.loginfo("playing %d frames..." % len(anim_dict['keyframes']))
            self.execute_play(anim_dict, anim_path)
            return "playing %d frames..." % len(anim_dict['keyframes'])
        except MasterException:
            rospy.logwarn("There is no Robot! Can't play Animation!")
            return "There is no Robot! Can't play Animation!"

    def execute_play(self, anim_dict, anim_path):
        """ We make a temporary copy of the animation and call the animation play action to play it"""
        anim_package = rosparam.get_param("robot_type_name").lower() + "_animations"
        rospack = rospkg.RosPack()
        path = rospack.get_path(anim_package)
        path = os.path.join(path, "record" + '.json')


        with open(path, "w") as fp:
            json.dump(anim_dict, fp, sort_keys=True, indent=4)
        p = subprocess.Popen(["scp", path, anim_path]).wait()
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
        self.anim_client.send_goal_and_wait(goal)
        rospy.sleep(0.5)

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
        '''
        Duplicates a frame
        '''
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
                        while(x):
                            x = False
                            for frame in self.current_state.anim_steps:
                                if newname + str(n) == frame["name"]:
                                    n+=1
                                    x = True
                        newname = newname+str(n)

                duplicate["name"] = newname
                new_frames.append(duplicate)
        self.current_state.anim_steps = new_frames
        self.save_step("Duplicated Frame " + frame_name)

    def delete(self, frame_name):
        '''
        Deletes a frame
        '''
        new_frames = []
        for frame in self.current_state.anim_steps:
            if not frame_name == frame["name"]:
                new_frames.append(frame)
        self.current_state.anim_steps = new_frames
        self.save_step("Duplicated Frame " + frame_name)
