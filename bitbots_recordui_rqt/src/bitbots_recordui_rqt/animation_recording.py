#!/usr/bin/env python3
import datetime

# todo
# meta daten der animation
# klickbar für welche motoren werte in den step gespeichert werden
# leiste mit keyframe, die namen haben. letzter is current keyframe

import rospy
from copy import deepcopy


class AnimState(object):
    """ Defines a current status of the recorded Animation
    """
    def __init__(self):
        self.anim = []
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

    # Recorder propagates some properties of the
    # AnimState it holds, as if they where its own
    # using a nifty little piece of utility code
    anim = property_proxy('current', 'anim')
    name = property_proxy('current', 'name')
    version = property_proxy('current', 'version')
    last_edited = property_proxy('current', 'last_edited')
    author = property_proxy('current', 'author')
    last_hostname = property_proxy('current', 'last_hostname')
    description = property_proxy('current', 'description')

    def __init__(self, ipc, gui, logger=None):
        self.steps = []
        self.redo_steps = []
        self.current = AnimState()

    def save_step(self, description, state=None):
        """ Save the current state of the Animation
        for later restoration by the undo-command

        (Yes we might save only a diff, but the Memory consumption
        should still be relatively low this way and saving / undoing
        is really cheap in terms of CPU and effort spent programming)

        :param description: A string describing the saved action for the user
        :param state: a AnimStep can be given otherwise the current one is used
        """

        rospy.logdebug("Saving step: %s" % description)
        if not state:
            state = deepcopy(self.current)
        self.steps.append((state, description))
        self.dump("backup")

    def undo(self, amount=1):
        """ Undo <amount> of steps or the last Step if not given
        """
        if amount > len(self.steps):
            rospy.logwarn("I cannot undo what did not happen!")
            #todo display in GUI
            return False
        if amount == 1:
            state, description = self.steps.pop()
            self.redo_steps = [(state, description, self.current)]
            self.current = state
            rospy.loginfo("Undoing: %s" % description)
            if self.steps:
                state, description = self.steps[-1]
                rospy.loginfo("Last noted action: %s" % description)
            else:
                rospy.loginfo("There are no previously noted steps")
            #todo if this is called, the gui has to update the motor values
            #self.gui.display_keyframes(self.current.anim)
            return True
        else:
            rospy.loginfo("Undoing %i steps" % amount)
            state, description = self.steps[-amount]
            self.current = state
            self.redo_steps = self.steps[-amount:].reverse()
            self.steps = self.steps[:-amount]
            #todo if this is called, the gui has to update the motor values
            #self.gui.display_keyframes(self.current.anim)
            return True

    def redo(self, amount=1):
        """ Redo <amount> of steps, or the last step if omitted
        """
        post_state = None
        if not self.redo_steps:
            rospy.logwarn("Cannot redo what was not undone!")
            #todo display in GUI
            return False
        if amount < 0:
            rospy.logwarn("Amount cannot be negative! (What where you even thinking?)")
            return False
        while amount and self.redo_steps:
            pre_state, description, post_state = self.redo_steps.pop()
            self.steps.append((pre_state, description))
            amount -= 1
        self.current = post_state
        #todo
        #self.gui.display_keyframes(self.current.anim)
        rospy.loginfo("Last noted step is now: %s " % self.steps[-1][1])
        return True

    def record(self, motor_pos, seq_pos=None):
        """ Record Command, save current keyframe-data
        """
        frame = {
            "duration": 1,
            "pause": 0.0,
            "goals": dict(motor_pos)
        }
        if not seq_pos:
            self.save_step("Appending new keyframe #%i" % len(self.anim))
            self.current.anim.append(frame)
            #self.gui.append_keyframe(frame)  # performanter als alle keyframes neu anzeigen
        else:
            self.save_step("Inserting new keyframe to position %s" % seq_pos)
            self.current.anim.insert(seq_pos, frame)
            #self.gui.display_keyframes(self.current.anim)
        #todo the gui has to display the right keyframe
        return True

    def clear(self):
        """ Record Command, clear all keyframe-data
        """
        self.save_step("Clearing all keyframe data")
        self.current.anim = []
        #todo gui has to update
        return True

    def dump(self, name, force=False):
        """ Record Command, dump all keyframedata to an animation .json file

        The GUI is asked for validity of the data, because the GUI keeps track
        of that anyway. If the data is not valid, saving the changes is refused
        to avoid putting defect files into our repository or even overriding
        functional ones. Forcing this method to dump results in a '_defective'
        postfix of the filename. The force feature is intended to allow "emergency"
        saves when the Programm itself is forced to exit in an inconsistent state.

        :param name: what name the new file should receive
        :param force: set True, for saving even inconsistent states.
        """
        if not self.anim:
            self.log.info("There is nothing to save.")
            return False
        if not self.gui.is_valid():
            self.log.warn("Errors exist, cannot save!")
            if force:
                name += '_defective'
                self.log.warn(
                    "But I was forced to save anyway, as ~/%s.json" % name)
            else:
                return False
        path = os.path.normpath('/home/darwin/')
        if not os.path.isdir(path):
            path = os.path.expanduser('~')
        path = os.path.join(path, name + '.json')
        self.log.debug("Speichere Abfolge nach '%s'" % path)

        self.current.version += 1
        anim = {
            "name": name,
            "version": self.current.version,
            "last_edited": datetime.datetime.isoformat(datetime.datetime.now(), ' '),
            "author": self.current.author,
            "description": self.current.description,
            "keyframes": self.current.anim,
            "hostname": gethostname()
        }

        with open(path, "w") as fp:
            json.dump(anim, fp, sort_keys=True, indent=4)
        return True

    def revert(self, framenumber=None):
        """ Record Command, remove the last keyframedata

        :param framenumber: The Number of frame to remove. default is last
        """
        if not framenumber:
            if not self.anim:
                self.log.warning("Nothing to revert, framelist is empty!")
                return False
            self.save_step("Reverting the last Keyframe (#%i)" % len(self.anim))
            self.current.anim.pop()
            self.gui.pop_keyframe()
            return True
        else:
            try:
                framenumber = int(framenumber)
            except TypeError:
                self.log.warn("Optional framenumber must be Integer! (got %s)" % framenumber)
                return False
            if len(self.anim) < framenumber:
                self.log.warn("Invalid framenumber: %i" % framenumber)
                return False
            self.save_step("Reverting keyframe #%i" % framenumber)
            framenumber -= 1  # Frameindices in the GUI are starting with 1, not 0
            self.current.anim.pop(framenumber)
            self.gui.pop_keyframe(framenumber)
            # If another element but the last was removed, we have to
            # recalculate the framenumber. I am lazy and simply let the gui
            # load a new set of keyframe elements from our animation status
            # here. Making the GUI able to change framenumbers of the keyframes
            # would give a performance-boost here.
            self.gui.display_keyframes(self.anim)
        return True

    def mirror(self, selector, tag):
        """Mirrors a Motor-Group to its opposing motors
        """
        indices = index_select(self.anim, selector)
        if not indices:
            return False

        # temp-safe current state
        state = deepcopy(self.current)

        for index in indices:
            if not self.mirror_single_frame(index, tag):
                self.log.warn("Mirror failed on Keyframe %i, reverting!")
                self.current = state
                return False

        # finally save after we know the mirror is a success
        self.save_step("Mirror of Motor-Group '%s' in Keyframes %s" % (tag, selector), state)
        self.gui.display_keyframes(self.current.anim)
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
                    new_value = self.current.anim[framenumber]['goals'][joint.name]
                    if joint.inverted:
                        new_value *= -1
                    self.current.anim[framenumber]["goals"][opposing_joint.name] = new_value
        except KeyError:
            self.log.warning("I am missing a Joint in my animation that should be mirrored, aborting")
            return False
        except IndexError:
            self.log.warning("Keyframe %i does not appear to exist!" % framenumber)
            return False
        return True

    def retrieve_animation(self, name):
        """ Function to locate and extract animation-data

        First tries locate the file,
        then extracts and returns the data if possible

        :param name: name of the animation
        :return: The retrieved data, empty list if nothing could be retrieved.
        """
        try:
            search = "/home/darwin/%s.json" % name
            filename = find(search)
        except IOError:
            self.log.debug('animation not found in %s ' % search)
            try:
                search = os.path.join(os.path.expanduser('~'), name + ".json")
                filename = find(search)
            except IOError:
                self.log.debug('animation not found in %s ' % search)
                try:
                    filename = find_animation(name)
                except IOError:
                    self.log.warn(
                        "Animation %s konnte nirgendwo gefunden werden!" % name)
                    return False
        self.log.info("Animation unter %s gefunden" % filename)

        data = []
        with open(filename) as fp:
            try:
                data = json.load(fp)
            except ValueError as e:
                self.log.error("Animation %s ist fehlerhaft:\n %s" %
                               (filename, e.message.partition('\n')[0]))
        return data

    def load(self, name):
        """ Record command, load a animation '.json' file

        :param name: name of the animation to load
        """
        start = time.time()
        data = self.retrieve_animation(name)

        # Ensure Data retrieval was a success
        if not data:
            return False

        self.save_step("Loading of animation named %s" % name)

        self.current.anim = data[u'keyframes']
        end = (time.time() - start) * 1000
        self.log.info("Loaded in %s ms" % end)
        self.current.name = name

        # get metadata from the file, if specified
        def get_meta(key, default="Unknown"):
            """ Retrieve the meta-information for given key from data
            :param str key:
            :param default: returned if key does not exist
            :type default: any
            """
            if not key in data:
                msg = "key %s not found in the animation %s" % (key, animation)
                self.log.debug(msg)
                return default
            return data[key]
        self.description = get_meta('description', "Edit me!")
        self.version = get_meta('version', 0)
        self.last_edited = get_meta('last_edited')
        self.author = get_meta('author')
        self.last_hostname = get_meta('hostname')

        self.gui.display_keyframes(self.current.anim)
        self.log.info("Abfolge von %d Stellungen geladen" % len(self.current.anim))
        return True

    def append(self, name):
        """ Record command, append keyframes of an animation to the current one

        :param name: name of the animation to append
        """
        data = self.retrieve_animation(name)
        if not data:
            return False
        self.save_step("Appending animation named %s" % name)
        self.current.anim.extend(data[u'keyframes'])
        self.gui.display_keyframes(self.current.anim)
        self.log.info(
            u"Abfolge von %d Stellungen angefügt" % len(data[u'keyframes']))
        return True

    def play(self, anim=None, selector=None):
        """ Record command, start playing an animation

        Can play a certain (named) animation or the current one by default.
        Also can play only a part of an animation if *start* and/or *end* are defined

        :param anim: Animation-Name, defaults to the current animation
        :type anim: str or None
        :param selector: A selector specifying what keyframes of the Animation should be included
        :type selector: str
        :return: Success of the operation
        """

        # If no anim given, we use the current one
        if not anim:
            if not self.current.anim:
                self.log.info("Refusing to play, because nothing to play exists!")
                return False
            anim_dict = {
                "name": "Record-play",
                "keyframes": self.current.anim
            }
            self.log.debug("playing current animation")
        # If anim given, we load it
        else:
            self.log.debug("loading animation %s" % anim)
            anim_dict = self.retrieve_animation(anim)

            # ensure the retrieval was a success
            if not anim_dict:
                self.log.warning("Retrieval of animation %s failed!" % anim)
                return False

        # Filter the Keyframelist when a selector is given
        if selector:
            self.log.debug("Filtering the Keyframelist based on: '%s' ..." % selector)
            cropped_list = list_select(anim_dict['keyframes'], selector)
            if not cropped_list:
                self.log.warning("not playing, because something went wrong selecting the keyframes")
                return False
            anim_dict['keyframes'] = cropped_list

        self.log.info("Spiele %d Stellungen ab..." % len(anim_dict['keyframes']))
        return self.execute_play(anim_dict)

    def execute_play(self, anim_dict):
        """ Try to move the robot according to given data

        The operation might fail
        if it is not possible to control the Robot

        :param anim_dict: Animation data (as dict) :return: Success of the operation """
        success = True
        try:
            animation.Animator(
                animation.parse(anim_dict),
                self.ipc.get_pose()).play(self.ipc, recordflag=True)
        except NotControlableError:
            self.log.warn("Motion meldete keine Kontrolle!")
            success = False
        return success

    def init(self):
        """ Record command, set the robot back to his init-pose
        Shortcut for 'play walkready'
        """
        self.log.info("Gehe in initpose")
        if not self.play_animation(self.ipc, "walkready"):
            self.log.warn(
                "Konnte init nicht korrekt ausführen (siehe motion debug)")
            return False
        return True

    def play_animation(self, ipc, name):
        """ Spiel die Animation *name* ab. Stopt den Roboter vorher, falls nötig"""
        self.log.info("Spiele Animation '%s'" % name)

        filename = find_animation(name)
        with open(filename) as fp:
            info = json.load(fp)

        anim = animation.parse(info)
        try:
            animation.Animator(anim, ipc.get_pose()).play(ipc, recordflag=True)
        except NotControlableError:
            self.log.warn("Motion meldete keine Kontrolle!")
            return False
        self.log.info("Beende Animation '%s'" % name)
        return True

    def copy(self, frm, to=None):
        """ copy a keyframe
        :param frm: position to copy from
        :param to: position to copy to, defaults to frm
        """
        frm -= 1  # adapt for index difference
        if not to:
            to = frm
        else:
            to -= 1  # adapt for index difference
        try:
            self.current.anim.insert(to, self.current.anim[frm])
        except IndexError:
            self.log.warn("The Keyframe number %s does not exist!" % frm)
            return False
        self.gui.display_keyframes(self.current.anim)
        return True

    def move(self, frm, to):
        """ move a keyframe
        :param frm: position to move from
        :param to: position to move to
        """
        assert frm > 0
        assert to > 0
        # adapt for index difference
        frm -= 1
        to -= 1
        orig_state = deepcopy(self.current)
        # calculate index shift by pop
        if frm <= to:
            to -= 1
        try:
            item = self.current.anim.pop(frm)
        except IndexError:
            self.log.warn("The Keyframe number %s does not exist!" % frm)
            return False
        self.save_step('moving Keyframe #%i to #%i' % (frm + 1, to + 1), orig_state)
        self.current.anim.insert(to, item)
        self.gui.display_keyframes(self.current.anim)
        return True

    def pose(self, frame_id):
        """ Take pose of a specific keyframe
        :param frame_id: id of the frame
        """
        assert frame_id > 0
        frame_id -= 1
        try:
            frame = copy.deepcopy(self.current.anim[frame_id])
        except IndexError:
            self.log.warn("Keyframe %s existiert nicht!" % (frame_id + 1))
            return False
        frame['duration'] = 1.0
        frame['pause'] = 0.0
        anim = {'name': 'record-pose',
                'keyframes': [frame]
                }
        self.log.info("Trying to pose Keyframe %s" % (frame_id + 1))
        try:
            animation.Animator(
                animation.parse(anim),
                self.ipc.get_pose()).play(self.ipc, recordflag=True)
        except NotControlableError:
            self.log.warn("Motion meldete keine Kontrolle!")
            return False
        return True
