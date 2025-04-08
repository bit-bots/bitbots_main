import os
from os import walk
from os.path import abspath, dirname, exists, join, normpath

from ament_index_python import get_package_share_directory


class ResourceManager:
    def __init__(self, robot_type: str):
        """
        The ResourceManager module provides functions to search and list animations for a given robot.
        Thus, it is possible to find resources without knowing the current location in the file system.

        :param robot_type: Suffixed with "_animations" to get the package name and thus the path to the animations
        """
        # Get the path to the animations folder
        self.basepath = abspath(os.path.join(get_package_share_directory(robot_type + "_animations"), "animations"))

        self.cache: dict[str, str] = {}
        self.files: list[str] = []
        self.names: list[str] = []  # Plain animation names, without filename-extension
        self.animpath = self._get_animpath()

    def search(self, path: str, folders: str | list[str], filename: str = "") -> str:
        """
        Searches in all folders in `path` recursively for the file specified in folders + filename.
        If folders is a list, every item of the list will be treated as a single folder.

        :param path: Path to search in
        :param folders: Folder or file to search for
        :param filename: Will be appended to each element in `folders` to complete it
        :raises IOError: When the file has not been found
        :return: Absolute path to the file

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
            raise OSError(f"Resource '{filename}' not found. folders was empty, only filename provided")
        raise OSError(f"Resource '{str(folders) + filename}' not found")

    def find(self, name: str | list[str], filename: str = "") -> str:
        """
        Searches the requested resource using `search` with folders = name and filename = filename,
        and saves the result to reuse it the next time the same resource is requested.

        :param name: Name of the file or folder to be searched
        :param filename: Appended to name, default=""
        :raises IOError: When the file has not been found
        :return: Absolute path to the file
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

    def find_animation(self, name: str) -> str:
        """
        Find an animation in <robot_name>_animations/animations/*. The filename
        should be given without ``.json``.
        path = find_animation('walkready')
        """
        return self.find(self.animpath, f"{name}.json")

    def find_resource(self, name: str) -> str:
        """Finds a resource relative to self.basepath"""
        return self.find(name)

    def _get_animpath(self) -> list[str]:
        """
        Get a list of folders in the animations/ folder.
        """
        anim_dir = self.find_resource("animations/")
        dirs = walk(anim_dir)
        anim_dirs = []
        for path in dirs:
            anim_dirs.append(path[0] + "/")
        return anim_dirs

    def find_all_animations(self, force_reload: bool = False) -> list[str]:
        """Finds all animations in the animations-directory

        returns a list of all animation-paths in the system.
        """
        if not self.files or force_reload:
            path = self.find_resource("animations/")
            for root, _, filenames in os.walk(path):
                for f in filenames:
                    name, dot, extension = f.rpartition(".")
                    if extension == "json":
                        self.files.append(os.path.join(root, f))
                        self.names.append(name)
        return self.files

    def find_all_animations_by_name(self, force_reload: bool = False) -> dict[str, str]:
        """Finds all animations in the animations directory.

        returns a dict from animation names to animation paths
        """
        if not self.files or force_reload:
            self.find_all_animations(force_reload)
        return dict(zip(self.names, self.files))
