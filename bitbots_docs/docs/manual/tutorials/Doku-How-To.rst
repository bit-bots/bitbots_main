===========
How to Doku
===========

Unter `doku.bit-bots.de <http://doku.bit-bots.de>`_ wird eine automatisch aktualisierte Doku zur Verfügung gestellt.

Automatisch aktualisiert bedeutet, dass der Stand der Dokumentation im Paket `bitbots_misc/bitbots_docs
<https://github.com/bit-bots/bitbots_misc>`_ automatisch von unserer `Jenkins CI <http://ci.bit-bots.de>`_
gebaut und deployt wird.

Dokumentation für catkin Paket
==============================

Es ist auserdem möglich paketspezifische Dokumentation zu erzeugen. Diese integriert sich über das
catkin Paket `bitbots_docs` in den catkin workflow.

Dokumentation einrichten
------------------------

Um die Dokumentation eines Bit-Bots Pakets zu aktivieren sind folgende Änderungen erforderlich:

#) ``package.xml``:
    Abhängigkeit auf ``bitbots_docs`` hinzufügen. Semantisch sollte ``<doc_depend>`` ausreichen
    aber, catkin braucht trotzdem das normale ``<depend>``.

    .. code-block:: xml

        <package>
            <depend>bitbots_docs</depend>
        </package>

#) ``CMakeLists.txt``:
    Paket verfügbar machen:

    .. code-block:: cmake

        find_package(catkin COMPONENT bitbots_docs)

    Doku bauen lassen:

    .. code-block:: cmake

        enable_bitbots_docs()

#) ``.gitignore``:
    Diese Anpassungen sind nicht zwingend jedoch empfohlen, um das Git Repo nicht mit überflüssigen
    Dateien vollzumüllen:

    .. code-block:: text

        # auto-generated documentation
        **/docs/_build
        **/docs/_out
        **/docs/conf.py

Anschliesend wurde ein neuer Unterordner `docs` im Paket erstellt. Dieser soll von nun an die
Dokumentation beherbergen.

Orderstruktur
-------------

Fertige Dokumentation kann unter ``docs/_out/index.html`` lokal abgerufen werden.

.. todo:: Genauer beschreiben. Vor allem, welche Dateien wie wiederhergestellt werden

Doku selbster bauen
-------------------

catkin build bitbots_docs --no-deps --make-args Documentation