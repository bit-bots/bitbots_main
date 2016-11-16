#!/usr/bin/env python
#-*-encoding:utf-8-*-
"""
Logikfunktionen zum Auffinden von Verbindungsstörungen in Motoren

.. moduleauthor:: Timon Giese<timon.giese@bit-bots.de>

History:
    2013-12-16: erstellt

"""
import yaml
from bitbots_common.util import find_resource, Joints
from bitbots_common.util.speaker import say
from bitbots_common.debug import Scope


class MotorConnectionAnalyser:
    """
    """
    def __init__(self):
        self.debug = Scope('MotorConnectionAnalyser')
        self.cables = {}
        self.load_connection_file()

    def load_connection_file(self):
        """ Lädt im idealfall die datei mit den Motorverbindungen
        """
        try:
            path = find_resource('cables.yaml')
            with open(path, 'r') as f:
                self.cables = yaml.load(f)
        except IOError:
            self.debug.warning('Konnte die Resource cables.yaml nicht finden, abbruch!')
        except yaml.YAMLError as e:
            self.debug.warning("Fehler in der cables.yaml: \n %s" % e)

    def find_connection_error(self, motors):
        """Sucht Verbindungsfehler
        anhand einer Liste nicht gefundener Motoren.

        :param motors: Liste nicht gefundener Motoren als int
        :return: Liste mit Motoren die wahrscheinlich Ausgangspunkt einer fehlerhaften verbindung sind
        """
        connected = self.cables.keys()
        connected = [motor for motor in connected if motor not in motors]

        errors = []
        try:
            for motor in motors:
                if self.cables[motor]['to'] in connected:
                    errors.append(motor)
        except KeyError as e:
            self.debug.warning("I don't know this 'motor' %s" % e)
            return 'fail'
        return errors

    def get_error_message(self, motors, config):
        """ Generiert einen Fehlerbericht
        in natürlicher Sprache als string, anhand einer Liste von nicht gefundenen Motoren
        durch verwendung von :py:func:`find_connection_error`
        """
        if not motors:
            return "I assume the error is in the caller of my motor-connection analysis, there was no faulty motor provided"
        if len(motors) == 1 :
            if motors[0] not in [20, 17, 18, 6, 5]:
                return ("Actually, I think this is not a connection error. I am not qualified for that analysis, but I think Motor %s might have the wrong ID" % motors[0])
        errors = self.find_connection_error(motors)
        if errors == 'fail':
            return "Error while analysing the motor-problem, check log-file!"

        l = len(errors)
        if l is 0:
            message = "I did not find any connection-error"
        elif l is 1:
            motor1 = errors[0]
            motor2 = self.cables[motor1]['to']
            #cable_id = self.cables[motor1]['name']

            message = "I assume the error is the cable between Motor %s and %s"
            message = message % (motor1, motor2)
            message += ("Which is the Cable to %s .\n" % Joints().get_motor_name(motor1))
        elif set(errors) == set([1, 2, 7, 8, 19]):
            message = "Sorry dude, it seems my cm-730 board is not working."
        else:
            message = "I assume the following %s connection errors:\n" % len(errors)
            for motor in errors:
                motor2 = self.cables[motor]['to']
                #cable_id = self.cables[motor]['name']
                message += ("Cable between Motor %s and %s . \n" % (motor, motor2))
                message += ("Which is the Cable to %s . \n" % Joints().get_motor_name(motor))

        return message

    def say_error_message(self, motors):
        """ Liest einen mit :py:func:`get_error_message` generierten String vor
        """
        message = self.get_error_message(motors)
        say(message)

    def get_motor_name(self, config, motor):
       for motor in config[joints]:
           if motor[id] == motor:
               return motor[name]
