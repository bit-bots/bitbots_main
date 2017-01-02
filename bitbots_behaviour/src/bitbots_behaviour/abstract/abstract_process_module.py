# -*- coding:utf-8 -*-
"""
AbstractProcessModule
^^^^^^^^^^^^^^^^^^^^^
Modul um Verhaltensmodule bereitzustellen dere Hauptarbeit in einem
Subprozess läuft. Damit kann der Prozess auf einem anderen CPU laufen

:platform: Unix, (mit einschränkungen Windows)

.. moduleauthor:: Nils Rokita <0rokita@informatik.uni-hamburg.de>

"""

import multiprocessing
from bitbots.modules.abstract import AbstractModule
from bitbots.debug import Scope
import bitbots.debug
import traceback
import subprocess
import time
import bitbots.util.speaker
import bitbots.util


class AbstractProcessModule(AbstractModule):
    """
    Dieses Modul sorgt dafür das erbende Module in einem eigenen
    Prozess ausgeführt werden. Dies hat einige Vor-, und Nachteile.

    Damit das Modul trotzdem mit den aktuellen Daten aus dem data
    dictonarry versorgt werden kann, wird Interprozesscommunikation
    verwendet. Dabei ist immer daran zu denken das dies durchaus etwas
    Zeit braucht bis die aktuellen Werte da sind. Außerdem sollte man
    nicht zu viel Daten versenden. Das Modul achtet selbstständig darauf
    das zu einem Schlüssel nur Wertänderungen übertragen werden. Damit
    das Modul weiß welche Datenfelder zu übertragen sind müssen diese
    im Konstruktor dieser Klasse mit angegeben werden (beim Super-
    aufruf bei der vererbung).

    .. warning::
        Datenfelder welche nicht in den requires bez. provides stehen
        werden *niemals* übertragen. Beim versuch etwas zu providen was
        nicht vereinbart ist kommt es sogar zu einem Fehler

    .. hint::
        Es sollte vermieden werden die gleichen namen zu requiren und
        providen da durch die relativ langsame übertragung sonst mit
        hoher warschienlichkeit immer der alte und neue Wert abwechselnd
        ankommen, und unendlich lange im kreis gesendet werden

    Die Daten werden wenn das Modul dran ist im :func:`update`
    ausgelesen und dann bei wertänderungen an den Subprosess gesendet.
    An gleicher stelle werden auch die vom Subprozess empfangenen Daten
    in das data dictonary geschrieben.

    .. warning::
        Bei einigen in C geschriebenen Klassen funktioniert das
        Senden nicht immer, diese können nicht als require oder provide
        genutz werden. (Objekte müssen Pickabel sein) Bei Fragen: Nils
        fragen.

    Im Subprocess können die daten mit :func:`get` ausgelesen und mit
    :func:`set` gesetzt werden.

    .. warning::
        Die Funktionen :func:`get` und :func:`set` nur im subprozess
        benutzen (alles was in :func:`run` ist oder  davon aufgeruffen
        wird) sonst kommt es
        zu einer Datenkorruption. Siehe: :func:`set_init`

    Die Methode :func:`run` wird als eingener Prozess gestartet, sollte
    also überschrieben werden wenn man ein Modul von diesem erben lässt.
    """

    def __init__(self, requires=(), provides=()):
        """
        Initialisiert die internen Datenstrukturen

        :param requires: eine Liste der Datenfelder welche dieses Modul
            benötigt
        :type requires: list of str
        :param provides: eine Liste der Datenfelder welche dieses Modul
            breitstellt
        :type provides: list of str
        """
        self.__data_server = {}
        self.__data_client = {}
        self.__requires = requires
        self.__provides = provides
        self.__con, self.__client_con = multiprocessing.Pipe()
        self.__internal = {}
        self.__started = False
        self.__debug = Scope("Multiprocessing.%s" %
                             self.__class__.__name__)

    def run(self):
        """ Dies ist die Methode welche in einem eigenen Process
        ausgeführt wird
        """
        raise NotImplementedError

    def set(self, name, value):
        """
        Setzt einen Wert im data dict

        :param name: Der name des Datenfeldes
        :param value: der zu übertragen value zu dem Namen. Achtung:
            dieser Wert muss mit pickle zu packen sein. (ist fast alles).

        Es wird darauf geachtet nur Wert änderungen zu Senden, um das
        datenvolum gering zu halten

        .. warning::
            Diese Funktion darf *nur* innerhalb des Subprozesses benutzt
            werden, sonst kommt es in der interprozesskommunikation
            zu Problemen (korrupte Daten im schlimmsten fall, mindestens
            aber kommen die werte nicht da an wo man dachte)

            Siehe :func:`set_init`

        .. warning::
            Namen die nicht im provide stehen werfen einen KeyError
        """
        # minimierung des Datenaufkommens, wenn der wert schon so ist,
        # muss er nicht neu gesendet werden
        if self.__data_client[name] != value:
            self.__data_client[name] = value
            self.__client_con.send((name, value))

    def get(self, name, default=None):
        """
        Hohlt einen Wert vom data dict

        :param name: Der name des Datenfeldes
        :param default: Der wert der zurückgegeben wird wenn noch kein
            Wert für den Namen übertragen wurde.
        :return: den Wert der unter name liegt, oder default

        default kommt normalerweise in drei Fällen zum einsatz: Wenn
        der wert für den namen durch die langsame
        Interprozesskommunikation am anfang noch nicht übertragen wurde
        oder wenn der name nicht in den requires war also niemals
        übertragen wird, oder der Wert vom sendenen Modul einfach noch
        nicht gesetzt wurde.

        Default ist standartmäßig None

        .. warning::
            Diese Funktion darf *nur* innerhalb des Subprozesses benutzt
            werden, sonst kommt es in der interprozesskommunikation
            zu Problemen (korrupte Daten im schlimmsten fall, mindestens
            aber kommen die werte nicht da an wo man dachte)

            Siehe :func:`set_init`
        """
        self.__client_con_reader()
        if name in self.__data_client:
            return self.__data_client[name]
        return default

    def set_init(self, name, value, data=None):
        """
        Setzt einen Wert im data dict.

        Diese Methode ist dafür da um Werte vor dem Starten vom
        Subprozess zu setzen. Die Werte stehen damit sofort sowohl im
        globalen data dict als auch im Subprozess (über:func:`get`) zur
        verfügung, es muss nicht die wartezeit der Kommunikation in kauf
        genommen werden.

        .. warning::
            Nur in :func:`__init__` benutzen. Alles andere würde die
            Daten nicht da abliefern wo sie hingehören

        Wenn ein data dict angegeben wir, dann wird der Wert auch sofort in
        das dict geschrieben

        :param name: Name des Keys
        :type name: String
        :param value: Der Wert
        :param data: Das globale Datadictonary (aus start(), optional)
        :type data: dict
        """
        self.__data_server[name] = value
        self.__data_client[name] = value
        if data:
            data[name] = value

    def __start(self, debugname):
        """
        Interner Starthelfer für den Subprozess
        """
        # Reinitialisieren diverser Problematischer Bereiche
        # da sie sonst im Subprozess broken sind
        reload(bitbots.util)
        reload(bitbots.debug)
        reload(bitbots.debug.debug)
        reload(bitbots.debug)
        self.__debug = Scope("Multiprocessing.%s" %
                             self.__class__.__name__)
        self.__debug("Subprocess gestartet")
        self.debug = Scope(debugname)
        self.__client_con_reader()
        # initialisieren aller felder in __data_client die es noch nicht
        # gibt welche aber provides werden, damit beim senden schon
        # alle namen vorhanden sind, und das nicht immer getesten werden
        # muss
        for name in self.__provides:
            if name not in self.__data_client:
                self.__data_client[name] = None
                self.__client_con.send((name, None))
        while True:
            try:
                self.run()
            except Exception as e:
                if isinstance(e, KeyboardInterrupt):
                    # Keyboard interrupt wollen wir nicht fangen!
                    raise
                try:
                    self.__debug.error(e, "Subprozess has terminated, restart. ")
                    time.sleep(1)
                except Exception as e:
                    print(e)
                    # TODO: vernünftige fehlerbehandlung!!!

    def __client_con_reader(self):
        """
        Ließt alle daten die in der Clientseitigen Pipe warten
        und pflegt diese in das interne data dict ein, damit sie mit
        :func:`get` geholt werden können. Diese Funktion muss nicht
        explizit aufgeruffen werden, das geht automatisch
        """
        try:
            while self.__client_con.poll():
                data = self.__client_con.recv()
                # print "Client: Recived: ",data
                self.__data_client[data[0]] = data[1]
        except EOFError:
            self.__debug.warning("Clientseitige Pipe ist korrupt")
            raise SystemExit()

    def __server_con_reader(self):
        """
        Ließt alle Daten die in der Serverseitigen Pipe warten
        und pflegt diese in das interne data dict ein, damit sie im
        :func:`update` zurückgeschrieben werden können. Diese Funktion
        muss nicht explizit aufgerufen werden, :func:`update` macht das
        """
        try:
            while self.__con.poll():
                data = self.__con.recv()
                # skyprint "Server: Recived: ",data
                self.__data_server[data[0]] = data[1]
        except EOFError:
            self.__debug.warning("Serverseitige Pipe ist korrupt")
            raise SystemExit()

    def __server_con_sender(self):
        """
        Sendet alle geänderten Daten an den Subprozess
        """
        for key in self.__internal:
            # zur vermeidung unnotig gesendeter daten
            if key not in self.__data_server or \
                            self.__internal[key] != self.__data_server[key]:
                self.__con.send((key, self.__internal[key]))
                self.__data_server[key] = self.__internal[key]

    def update(self, data):
        """
        Update der normalen Updatekette der Modul-Architektur.

        Hier werden die Daten für den Subprozess eingesamelt und das
        versenden gestartet, ebenso werden die empfangenen Daten
        zurück ins data dict geschrieben
        """
        self.__server_con_reader()

        for name in self.__requires:
            if name in data:
                self.__internal[name] = data[name]

        self.__server_con_sender()

        if not self.__started:
            self.__debug("Starte Subprocess")
            process = multiprocessing.Process(target=self.__start,
                                              name=self.__class__.__name__, args=(self.debug.get_name(),))
            process.deamon = True
            process.start()
            self.__started = True

        for name in self.__provides:
            data[name] = self.__data_server.get(name, None)
