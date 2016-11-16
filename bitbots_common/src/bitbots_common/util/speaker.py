# -*- coding:utf-8 -*-
"""
Speaker
^^^^^^^

Dieses Modul sorgt für die geordnete ausgabe von Texten. Es bemüht sich dabei
mehrfache, gleichzeitige ausgaben zu unterdrücken, dies geht momentan nur
inerhalb eines Prozesses. Dazu wird eine gemeinsame Instanz
von :py:class:`Speaker` vom :py:mod:`util` bereitgestellt es ist einfach
möglich so bitbots.util.say zu importieren um :func:`Speaker.say` an der
gemeinsamen Instanz aufzurufen.

"""

import subprocess
import random
import os
import socket
import time
#from bitbots.debug import Scope

from bitbots_common.util.config import get_config


def noop():
    """
    Noop tut nichts, für einfachere handhabung von lehren callbacks
    """
    pass


class Speaker(object):
    """
    Eine Klasse welche die ausgabe von Texten regelt, und sie in einem eigenen
    Thread handelt, und dabei Sequenziel abarbeitet.
    """

    def __init__(self):
        #self.debug = Scope("Util.Speaker")
        self.config = get_config()["speaker"]
        self.saylog = []
        self.__start_speaker()
        self.do_not_speck = os.getenv("FORCE_SPEAKER_TO_BE_QUIET")
        self.speaking = None

    def __reduce__(self):
        return (Speaker, (), None, None, None)

    # Funktion zum Sprechen von Wörtern oder Sätzen
    def say(self, text, blocking=False, callback=noop):
        """
        Ausgeben des Textest.

        :param text: Der zu Sprechende Text
        :type text: str
        :param blocking: Wenn True kehrt die Funktion erst nach abgeschlossener
            ausgabe zurück
        :type blocking: Boolean
        :param callback: Funktion welche nach ende der ausgabe aufgeruffen wird
        :type callback: Function

        .. hint:: Das callback wird aus einem eigenen Thread aufgeruffen, und
            sollte daher Threadsave sein

        Beim Call element wird als letzten element random.random()
        hinzugefügt, damit bei Blocking nachgesehen werden kann ob
        es noch in der que ist.
        """
        cal = (("espeak", text), callback, random.random())
        if self.do_not_speck is '1':
            return
        self._to_saylog(text, blocking)

    def speak_file(self, filename, blocking=False, callback=noop):
        """
        Ausgabe der Datei filename mittels espeak

        :see: :func:`say`
        """
        cal = (("espeak", "-m", "-f", filename), callback, random.random())
        self._to_saylog(cal, blocking)

    def to_speak(self):
        """
        Gibt die ANzahl der noch zu Sagenen Sätze zurück
        """
        return len(self.saylog)

    def _to_saylog(self, text, blocking):
        id = random.randint(0, 9999999)
        try:
            self.pipe.sendall(u"%d顶%s".encode('utf-8') % (id, text))
        except Exception as e:
            subprocess.call(["espeak", text])
            # call ist absichtlich blockierend da es sonst alles
            # aufeinmal erzählt, es ist nur ein fallback hier
            text2 = "An error Occured in Speaker! %s" % e
            print(text2)
            # print weil es hier kein debug geben kann (es würde ne rekursion
            # geben)
            subprocess.call(["espeak", text2])
            return
        if self.config['secondary-server']:
            try:
                self.pipe2.sendto(u"顶%s".encode('utf-8') % text), (
                    self.config['secondary-server'], self.config['server-port'])
            except:
                pass
        if blocking:
            now = time.time()
            while time.time() - now < 5:
                print("bla!!")
                try:
                    ret = self.pipe.recvfrom(10000)
                    print("bla2")
                    ret = ret[0].split(u"顶".encode('utf-8'))
                    for code in ret:
                        print("bla3")
                        if str(id) == code:
                            print("bla4")
                            return
                except socket.error:
                    # ignore because don't care
                    print("Blocking Say: Timeout2, don't care...")  # print für tmp debug
                    pass
            # Print weil es sonst im debug rekursion gibt
            print("Speaker blocking call timeout")

    def __start_speaker(self):
        """
        Startet den Speaker Thread
        """
        if self.config['test-server'] and not self.config['server-bin'] in os.popen("ps xa").read():
            subprocess.Popen([self.config['server-bin']])
            time.sleep(1)
        self.pipe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pipe2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pipe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.pipe.settimeout(1)
        self.pipe.connect((self.config["server-ip"], self.config['server-port']))
        # wir versuchen uns erstmal zu vorhandenen Instanzen zu verbinden


# globale speaker instanz
__speaker = Speaker()  # pylint: disable=C0103
say = __speaker.say  # pylint: disable=C0103
to_speak = __speaker.to_speak  # pylint: disable=C0103
