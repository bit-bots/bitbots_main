#-*- coding:utf-8 -*-
"""
ResourceManager
^^^^^^^^^^^^^^^

Das Modul RecourceManager Stellt Funktionen zum suchen von Dateien im
Darwin Projekt bereit. Damit ist es möglich ohne kentniss des aktuellen
ortners wo man selbst ausgeführt wird Recourcen zu finden.

dieses Modul Stellt die Globalen Methoden :func:`find_recource`
, :func:`find_anim` und :func:`find` bereit welche eine globale Instanz
vom :class:`ResourceManager` benutzen. Das Sorgt dafür das eine einmal
gefundene Datei nicht wieder gesucht werden muss.

"""

#todo translate old german comments to english

import os.path
from os.path import abspath, dirname, exists, join, normpath
from os import walk
# get an instance of RosPack with the default search paths
import rosparam
import rospkg as rospkg

anim_package = rosparam.get_param("/robot_type_name").lower() + "_animations"
rospack = rospkg.RosPack()
path = rospack.get_path(anim_package)
BASEPATH = abspath(path + "/animations")


class ResourceManager(object):
    """ Simpler Resourcen Manager, der den absoluten Pfad für Resources
        in Abhängigkeit des Python-Paths findet, und dannach in einem
        Cache hält.
    """
    def __init__(self):
        self.cache = {}
        self.files = []  # Animations cached for find_all_animations
        self.names = []  # Plain animation names, without filename-extension
        self.animpath = self._get_animpath()

    def search(self, path, ort, datei=""):
        """
        :param path: Pfad in dem gesucht wird
        :type path: String
        :param ort: ordner und oder Dateinamen nach dem gesucht wird
        :type ort: String or List of Strings
        :param datei: wird an jedes element von ort angehängt um den ort
            zu verfolständigen, default =""
        :type datei: String
        :raises: IOError
        :return: Absoluter Pfad zur Datei
        :returntype: String

        Diese Methode durchsucht alle ordner von path abwärts nach / ob
        ort + datai existiert. Wenn ort eine liste ist wird jedes
        element von ort einzelnd als ort genommen, also alle einmal
        durchprobiert, dies lässt sich nutzen um in mehreren Ordnern zu
        suchen.

        Wenn die Datei in keinem der Pfade gefunden wird, wird ein
        IOError geschmissen.

        Ein kleines Beispiel: Wenn::
            search("/home/bitbots/test/bla/","res/anim/","data.json")

        aufgeruffen wird, so werden die folgenden Pfade getestet::

            /home/bitbots/test/bla/res/anim/data.json
            /home/bitbots/test/res/anim/data.json
            /home/bitbots/res/anim/data.json
            /home/res/anim/data.json
            /res/anim/data.json

        Beim ersten erfolg wird der gefundene Pfad zurückgegeben
        """
        if not isinstance(ort, list):
            ort = [ort]
        for name in ort:
            name = name + datei
            while True:
                # normpath sthet dort damit die dateipfade auch undert windows
                # Funktionieren (ersetzt im wesentlichen / durch \)
                fname = normpath(join(path, name))
                if exists(fname):
                    return fname

                next_path = dirname(path)
                if next_path == path:
                    break

                path = next_path
        if not ort:
            return IOError("Resource '%s' not found. Ort was empty, \
                only datei provided" % (datei))
        return IOError("Resource '%s' not found" % (str(ort) + datei))

    def find(self, name, datei=""):
        """
        :param name: Name der zu suchenden Datei oder Ordner
        :type name: String or List
        :param datei: Anhängsel an name, default = ""
        :type datei: String
        :raises: IOError
        :return: Absoluten Pfad zur Datei
        :returntype: String

        Sucht die Angeforderte Resource mithilfe von :func:`find` mit
        ort = name und datei=datei, speichert das ergebnis in einem
        cache so das bei erneuter anfrage, nicht erst gesucht werden
        muss.

        Als path wird der Pfad zu dieser Datei benutzt.

        Bei nicht finden der Datei wird ein IOError geworfen.
        """
        cache_name = str(name) + datei
        if cache_name not in self.cache:
            result = self.search(BASEPATH, name, datei)
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
        """ Findet eine Animation unter share/bitbots/animations/*.
            Der Dateiname in *name* ist ohne ``.json`` anzugeben.
            path = find_animation("walkready")
        """
        return self.find(self.animpath, "%s.json" % name)

    def find_resource(self, name):
        """ Findet eine Resource relativ zu BASEPATH """
        return self.find(name)

    def _get_animpath(self):
        """
        Sucht unterhalb von animations/ nach ordnern um alle ordner in einer
        Liste zurückzugeben.
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


# Globale Instanz des Resourcen-Managers
__RM = ResourceManager()

# Shortcuts zum Suchen von Animationen und Resourcen
find_animation = __RM.find_animation  # pylint: disable=C0103
find_resource = __RM.find_resource  # pylint: disable=C0103
find = __RM.find  # pylint: disable=C0103
generate_find = __RM.generate_find  # pylint: disable=C0103
find_all_animations = __RM.find_all_animations
is_animation_name = __RM.is_animation_name
find_all_animation_names = __RM.find_all_animation_names
