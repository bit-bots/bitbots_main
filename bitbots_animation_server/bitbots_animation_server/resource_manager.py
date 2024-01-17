"""
ResourceManager
^^^^^^^^^^^^^^^

The ResourceManager module provides functions to search and list animations for a given robot.
Thus, it is possible to find resources without knowing the current location in the file system.
"""

import os
from os.path import abspath, dirname, exists, join, normpath
from os import walk
from ament_index_python import get_package_share_directory


class ResourceManager:

    def __init__(self, robot_type: str):
        # Get the path to the animations folder
        self.basepath = abspath(os.path.join(
            get_package_share_directory(robot_type + "_animations"),
            "animations"))

        self.cache = {}
        self.files = []
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
        :rtype: String

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
        :rtype: String

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
            path = self.find_resource('animations/')
            for root, _, filenames in os.walk(path):
                for f in filenames:
                    name, dot, extension = f.rpartition('.')
                    if extension == 'json':
                        self.files.append(os.path.join(root, f))
                        self.names.append(name)
        return self.files

    def find_all_animations_by_name(self, force_reload=False):
        """Finds all animations in the animations directory.

        returns a dict from animation names to animation paths
        """
        if not self.files or force_reload:
            self.find_all_animations(force_reload)
        return dict(zip(self.names, self.files))
