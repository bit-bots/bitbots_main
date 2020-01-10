Zeitsynchronisation
===================
Alle über ROS miteinander interagierenden Systeme müssen mit einer gemeinsamen Zeit arbeiten, da sonst Messages nicht
richtig übertragen werden. Auserdem benutzen andere Systeme Timestamps zum Beispiel Transforms.


Nuc und Jetson
--------------
Weil die `Jetson` keine Hardware-Clock besitzt, verliert sie die aktuelle Zeit.
Um dies zu beheben läuft auf dem `Nuc` ein `chrony` als Zeitserver. Die Jetson holt sich von dort
ebenfalls mit `chrony` immer eine synchronisierte Zeit.

Nuc-Konfiguration
~~~~~~~~~~~~~~~~~
Die Konfigurationsdatei ist ``/etc/chrony/chrony.conf``

:code:`local stratum 10` lässt Chrony als NTP-Server mit einem Stratum von 10 (ganz weit weg von Realzeit und nicht zuverlässig) laufen

:code:`allow 192.168.17.0/24` erlaubt Zugriff auf den Server vom gesamten internen Roboternetz (nuc, odroid, jetson)

Jetson-Konfiguration
~~~~~~~~~~~~~~~~~~~~
:code:`server nuc iburst` stellt den lokalen Nuc als Server ein (`iburst` erhöht Zuverlässigkeit beim Start)

:code:`makestep x y` erlaubt hartes Setzen der Zeit (ohne  slew), wenn die Differenz > `x` und nur während der ersten `y` Anfragen

Auserdem wird der `chrony` Service auf der Jetson mit Delay gestart, weil die Synchronisierung sonst nicht richtig funktioniert.
Mit :code:`systemctl edit chrony.service` kann ein Teil der Service-Definition überschrieben werden. Bei uns::

    [Service]
    ExecStartPre=/bin/sleep 5


Zwischen verschiedenen Robotern
-------------------------------
Es gibt zwar einen `peer`-Modus jedoch verwenden wir auf allen Robotern eine Client-Server Verbindung, da der Peer-Modus
nicht empfohlen wird.

:code:`keyfile /etc/chrony/chrony.keys` definiert die Keyfile, in der ID-Key Paare definiert werden

:code:`allow w.x.y.z` erlaubt Zugriff des "Peers" auf diesen Server

:code:`server w.x.y.z key 10` setzt den "Peer" auch als eigenen Server. Zusätzlich werden Pakete über den in der Keyfile
definierten Key 10 authentifizert. Der Server muss den gleichen Key mit der ID 10 haben.
