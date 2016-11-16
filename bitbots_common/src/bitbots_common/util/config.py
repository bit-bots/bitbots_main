#-*- coding:utf-8 -*-
"""
Config
^^^^^^

Dieses Modul läd die Configs
"""

import json
import os
from bitbots_common.util.resource_manager import find_resource
from bitbots_common.debug import Scope
import traceback
import yaml

import sys

if 'simplify-config' in sys.argv[0]:
    DO_NOT_LOAD_HOME_CONFIG = True
    print("I will not load Local-Config")
else:
    DO_NOT_LOAD_HOME_CONFIG = False


class HelperDict(dict):
    """
    Diese Klasse hält die Configwerte und verhält sich dabei
    weitestgehend wie ein dict.

    Beim versuch auf einen nicht vorhandenen Key zuzugreifen
    wird eine Warning und ein stacktrace ausgegeben, und 0 zurückgeben
    um den Programmablauf möglichst nicht zu stören
    """
    def __init__(self, *args, **kwargs):
        dict.__init__(self, *args, **kwargs)

        # Wir setzen einen Scope, wird meist ,it set_debug überschrieben
        self.debug = Scope("HelperDict")

    def set_debug(self, debug):
        """
        Setzt den debugscope

        Das ist dafür gedacht damit die Warnings möglichst aus dem scope der
        Config kommen. Wenn das nicht gemacht wird, kommen die ausgaben
        vom Scope "HelperDict"
        """
        self.debug = debug

    def __getitem__(self, key):
        try:
            return dict.__getitem__(self, key)
        except KeyError as e:
            self.debug.error(e, "Config key %s not found." % key)
            return 0


def yaml_include(loader, node):
    """Include another YAML file."""
    debug = Scope("Util.Config.yaml_loader")
    filename = loader.construct_scalar(node)
    filename = find_resource(filename + '.yaml')
    with open(filename) as include_fp:
        data = yaml.load(include_fp)
    debug.log("Include-Config '%s' geladen" % filename)
    return data

yaml.add_constructor('!include', yaml_include)

class Config(object):
    """
    Diese Klasse lädt die Config

    :param debug: der Debugscope
    :type debug: Scope
    :param config: name der Config die geladen werden
    :type config: String
    """
    def __init__(self, debug, config="config"):
        self.debug = debug.sub("Config")



    def recursive_update(self, old, new):
        """
        Funktion um ein dict recusiv zu updaten

        :param old: das zu updatene dict
        :type old: dict
        :param new: values aus diesem dict werden in old übernommen
        :type new: dict
        """
        old = old.copy()
        if new:
            if type(new) != dict:
                text = "Error in Config: expected dict, got %s in %s"
                text = text % (type(new), new)
                self.debug.warning(text)
                raise ValueError(text)

            for key in new:
                value = new[key]
                if key in old and type(old[key]) ==  dict and value is None:
                    self.debug.warning("Attemp to overwrite dict with None")
                if key in old and isinstance(value, dict):
                    # ist schon vorhanden und ist ein dict, dann updaten
                    old[key] = self.recursive_update(old[key], value)
                else:
                    # einfach übernehmen
                    old[key] = value

        return old

    def load_config_file(self, name):
        """
        lädt die Config mit dem namen name im recourcenpfad und im homedir
        die config aus dem homedir überschreibt dabei die values
        aus dem recource dir
        """
        default_name = find_resource(name + ".yaml")
        with open(default_name) as default_fp:
            config = yaml.load(default_fp)

        self.debug.log("Default-Config '%s' geladen" % default_name)

        try:
            if not DO_NOT_LOAD_HOME_CONFIG:
                home_name = os.path.expanduser("~/%s.yaml" % name)
                with open(home_name) as home_fp:
                    config = self.recursive_update(config,
                        yaml.load(home_fp))

                self.debug.log("Config '%s' mit spezifischen Werten geladen" %
                    home_name)
        except IOError:
            # keine config im homeordner, einfach ignorieren
            pass

        def log_config(conf, confdebug):
            """
            Send contents of the dict conf to the Scope confdebug
            """
            for (key, value) in conf.items():
                # the config might contain subsections (also dicts)
                if isinstance(value, dict):
                    log_config(value, confdebug.sub(key))
                else:
                    confdebug.log(key, repr(value))
        #send the loaded config to the debugger
        log_config(config, self.debug.sub(name))

        #conf = HelperDict(config)  # TODO: Das will wieder schöner gemacht werde
        # das ist ddraußen weil das dumpen damit probleme macht, und es
        # nicht so viel bringt
        #conf.set_debug(self.debug)
        #return conf
        return config

    def get_config(self, config="config"):
        """
        Gibt die Config aus der Datei `config` zurück

        :param config: Die zu ladene Config Datei
        :type config: String
        """
        try:
            self.config = self.load_config_file(config)
        except yaml.parser.ParserError as e:
            self.debug.error(e, "Parsing error while loading Config:", False)
            # Diesen Fehler nicht akrustisch ausgeben, da das massiv probleme
            # mit zücklischen abhängigkeiten gibt
            raise
        return self.config

CONFIG_OBJ = Config(Scope("Util"))
CONFIG = CONFIG_OBJ.get_config()


def get_config():
    """
    Methode zum einfachen zugriff auf die standardconfig
    """
    # Override for Player on Environment Variable for testing
    if "PLAYER" in os.environ:
        CONFIG["PLAYER"] = int(os.environ["PLAYER"])
    if "MITECOM" in os.environ:
        CONFIG["mitecom"]["port"] = int(os.environ["MITECOM"])

    return CONFIG

def get_special_config(config_file):
    """
    Holt die Config aus einer Speziellen ConfigDatei
    """
    return CONFIG_OBJ.get_config(config_file)
