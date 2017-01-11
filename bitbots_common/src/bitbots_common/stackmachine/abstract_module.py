"""
AbstractModule
^^^^^^^^^^^^^^

Dieses Modul ist die Basis aller VerhaltensModule

.. autoclass:: AbstractModule
"""


class AbstractModule:
    _event_framework = None

    def start(self, data):
        """
        Die Methode Start wird bei allen Modulen vor dem begin der
        normalen Update schleife ausgeführt. Dabei wird schon auf die
        reihnfolge die sich aus den abhängigkeiten ergeben geachtet.

        Sinn dieser Methode ist vor allem felder im data-dictonary
        zu initialisieren.

        .. warning ::
            Es stehen hier noch nicht alle felder in data zur
            verfügung, nur einige Module initiallisieren hier (z.B.
            ´ConfigModule´)
        """
        pass

    def update(self, data):
        """
        Diese Methode wird regelmäßig aufgerufeen. Hier soll die
        hauparbeit des Modules stadtfinden
        """
        pass

    def post(self, data):
        """
        wird nach dem durchlauf von update aufgeruffen
        """
        pass

    def pre(self, data):
        """
        wird vor dem update durchlauf aufgeruffen
        """
        pass

    def internal_init(self, debug, event_framework):
        """
        Initiallisiert bestimmte Variablen der Module, z.B. fürs
        debugging.

        Diese Funktion sollte nicht überschrieben werden, sie wird
        vom :mod:`framework` benutzt.

        Damit stehen in jedem Modul innerhalb von  :func:`start`
        , :func:`pre`, :func:`update` und :func:`post` folgende
        Variablen zur Verfügung:
        """
        self._event_framework = event_framework

    def register_to_event(self, event, function):
        """
        Registriert sich beim Framework für ein Event.

        :param event: das Event auf das reagiert werden soll
        :param function: Die Funktion ie im Falle eines Events
            aufgeruffen wird.
        """
        self._event_framework.register_to_event(event, function)

    def send_event(self, event, data=None):
        """
        Sendet das Event event ans Framework
        """
        self._event_framework.send_event(event, data)

    def __repr__(self):
        """
        Wir kürzen die Repräsentation ab, ist so kürzer, und sagt
        trotzdem noch genug
        """
        return "<Module: " + self.__class__.__module__.split('.')[-2] \
               + "." + self.__class__.__name__ + ">"

    def __str__(self):
        return self.__repr__()
