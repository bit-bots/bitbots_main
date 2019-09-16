#-*- coding:utf-8 -*-
"""
ResourceManager
^^^^^^^^^^^^^^^

The ResourceManager module provides functions for file searching in a
Darwin Project. Thus, it is possible to find resources without knowing
the currend location in the file system.

This module provides the global methods :func:`find_resource`,
:func:`find_anim` and :func:`find` which use a single global instance
of the :class:`ResourceManager`. Thereby, files that have once been
discovered do not have to be searched again.
"""

import os.path
from os.path import abspath, dirname, exists, join, normpath
from os import walk
# get an instance of RosPack with the default search paths
import rospkg as rospkg
import rospy



class ResourceManager(object):

    def __init__(self):
        if not rospy.has_param("robot_type_name"):
            rospy.logwarn("Robot type name parameter was not set. I assume that you want to use Wolfgang")
        anim_package = rospy.get_param("robot_type_name", "wolfgang").lower() + "_animations"

        rospack = rospkg.RosPack()
        path = rospack.get_path(anim_package)
        self.basepath = abspath(path + "/animations")

        self.cache = {}
        self.files = []  # Animations cached for find_all_animations
        self.names = []  # Plain animation names, without filename-extension
        self.animpath = self._get_animpath()

    def search(self, path, folders, filename=""):
        """
        :param path: path to search in
        :type path: String
        :param folders: folder or file to search for
        :type folders: String or List of Strings
        :param filename: will be appended to each element in `folders` to complete it
        :type filename: String
        :raises: IOError
        :return: absolute path to the file
        :returntype: String

        This method searches in all folders in `path` recursively for the file
        specified in folders + filename. If folders is a list, every item of the list will
        be treated as a single folder. This can be used to search in multiple folders.

        An IOError is raised when the file has not been found.

        Example:
        When
            search("/home/bitbots/test/bla/", "res/anim/", "data.json")

        is called, the following paths will be searched:

            /home/bitbots/test/bla/res/anim/data.json
            /home/bitbots/test/res/anim/data.json
            /home/bitbots/res/anim/data.json
            /home/res/anim/data.json
            /res/anim/data.json

        At the first success, the path is returned.
        """
        if not isinstance(folders, list):
            folders = [folders]
        for name in folders:
            name = name + filename
            while True:
                # normpath is used to make this work in windows
                fname = normpath(join(path, name))
                if exists(fname):
                    return fname

                next_path = dirname(path)
                if next_path == path:
                    break

                path = next_path
        if not folders:
            return IOError("Resource '%s' not found. folders was empty, \
                only filename provided" % (filename))
        return IOError("Resource '%s' not found" % (str(folders) + filename))

    def find(self, name, filename=""):
        """
        :param name: Name of the file or folder to be searched
        :type name: String or List
        :param filename: Appended to name, default=""
        :type filename: String
        :raises: IOError
        :return: Absolute path to the file
        :returntype: String

        Searches the requested resource using :func:`search` with
        folders = name and filename = filename, and saves the result to
        reuse it the next time the same resource is requested.

        self.basepath will be used as search path.

        An IOError is raised when the file has not been found.
        """
        cache_name = str(name) + filename
        if cache_name not in self.cache:
            result = self.search(self.basepath, name, filename)
            self.cache[cache_name] = result

        else:
            result = self.cache[cache_name]

        if isinstance(result, Exception):
            raise result

        return result

    def generate_find(self, basepath):
        """ Return a find function that automatically adds a basepath to
        each name
        :param basepath: The path to add to each file
        :type basepath: String

        The returned function takes one argument which will be added to the
        basepath before calling the normal "find" method without optional
        arguments.
        """
        path = normpath(basepath)
        return lambda name: self.find(join(path, name))

    def find_animation(self, name):
        """
        Find an animation in <robot_name>_animations/animations/*. The filename
        should be given without ``.json``.
        path = find_animation('walkready')
        """
        return self.find(self.animpath, "%s.json" % name)

    def find_resource(self, name):
        """ Finds a resource relative to self.basepath """
        return self.find(name)

    def _get_animpath(self):
        """
        Get a list of folders in the animations/ folder.
        """
        anim_dir = self.find_resource("animations/")
        dirs = walk(anim_dir)
        anim_dirs = []
        for path in dirs:
            anim_dirs.append(path[0] + "/")
        return anim_dirs

    def find_all_animations(self, force_reload=False):
        """ Finds all animations in the animations-directory

        returns a list of all animation-paths in the system.
        """
        if not self.files or force_reload:
            def add_anim(arg, dirnames, fnames):
                """ Reiht die aufgefundenen Dateien auf,
                vorausgesetzt es sind .jsons :)
                """
                for f in fnames:
                    name, dot, extension = f.rpartition('.')
                    if extension == 'json':
                        self.files.append(os.path.join(dirnames, f))
                        self.names.append(name)
            path = find_resource('animations/')
            os.path.walk(path, add_anim, None)
        return self.files

    def find_all_animation_names(self, force_reload=False):
        """ Same as find_all_animations, but returns a sorted set of the animations
        for use in the record-ui
        """
        if not self.names or force_reload:
            self.find_all_animations(force_reload)
        return sorted(set(self.names))

    def is_animation_name(self, name, force_reload=False):
        """Check if a name belongs to a safed animation
        """
        if not self.names or force_reload:
            self.find_all_animations(force_reload=True)
        return name in self.names

_RM = None  # type: ResourceManager
# Shortcut to search for animations
def find_animation(*args, **kwargs):
    global _RM
    if not _RM:
        _RM = ResourceManager()
    return _RM.find_animation(*args, **kwargs)
