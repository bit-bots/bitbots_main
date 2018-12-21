===================
How to Doku
===================

Unter `doku.bit-bots.de`_ wird eine automatisch generierte Doku zur Verfügung gestellt.

Automatisch generiert bedeutet, dass catkin Pakete eigenständig erkannt und `.rst` Dokumente aus dem Source-Code
generiert werden.
Leider können momentan nur `python2` catkin Pakete automatisch dokumentiert werden lassen und außerdem muss die
Dokumentation vom Paket-Maintainer erst aktiviert werden.

Dokumentation für catkin Paket aktivieren
=========================================
In der `package.xml` muss ein `rosdoc` Tag exportiert werden:

::

    <!-- The export tag contains other, unspecified, tags -->
    <export>
        <!-- Other tools can request additional information be placed here -->
        <rosdoc config="rosdoc.yaml"/>
    </export>


Manuell Dokumente erstellen
====================================
Zusätzlich zur Dokumentation des Source-Codes können eigene Dokumente erstellt werden.

:Format:
    Restructured Text (`.rst` Dateien)
:Ort:
    `bitbots_meta/doc/manual/...`
:Verlinkung:
    In einem beliebigen anderem bereits verlinkten Dokument über
    ::

        .. toctree::

            <relative-path-to-rst-file-without-file-ending>


.. _doku.bit-bots.de: http://doku.bit-bots.de/


Doku lokal bauen
================
Stell sicher, dass du alle Abhängigkeiten installiert hast indem du bitbots_meta/scripts/install_dependencies.sh ausführst.
Dann einfach im bitbots_meta repository folgenden Befehl ausführen:

.. code:: bash

    make doc

Die Doku wird gebaut und kann dann folgendermaßen aufgerufen werden

.. code:: bash

    firefox doc/html/index.html
