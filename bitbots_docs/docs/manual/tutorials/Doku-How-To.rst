===========
How to Doku
===========

Unter `doku.bit-bots.de <http://doku.bit-bots.de>`_ wird eine automatisch aktualisierte Doku zur Verfügung gestellt.

Automatisch aktualisiert bedeutet, dass der Stand der Dokumentation im Paket `bitbots_misc/bitbots_docs
<https://github.com/bit-bots/bitbots_misc>`_ automatisch von unserer `Jenkins CI <http://ci.bit-bots.de>`_
gebaut und deployt wird.

Installation von System Dependencies
====================================

Die Doku wird von Sphinx mit den Erweiterungen `breathe` und `exhale` gebaut.
Diese müssen entsprechend in den richtigen Versionen installiert werden.

.. code-block:: bash

        sudo apt remove python-sphinx
        sudo apt install python3-sphinx python3-sphinx-rtd_theme python3-breathe
        pip3 install -u exhale


Existierende Doku bauen
=======================

Diese allgemeine Dokumentation (die mit Tutorials inklusive dieser Seite) kann mit dem Command
``catkin build bitbots_docs --no-deps --make-args Documentation`` gebaut werden.

Für ein anderes Paket, bei dem die Doku wie oben beschrieben aktiviert wurde, kann sie mit
``catkin build <package> --no-deps --make-args Documentation`` gebaut werden.


Doku für ein Paket schreiben
============================

In jedem Paket, für das Dokumentation aktiviert ist gibt es den Ordner ``docs``.
In diesem ist sowohl die Konfiguration des Dokusystems (``docs/conf.py``) als auch
das Root-Dokument der Doku (``docs/index.rst``) zu finden.

Wichtig ist hier nur die ``index.rst``.
Von dieser Datei aus werden alle anderen Dokumente eingebunden.
Wie dies passiert ist frei wählbar wobei wir selbst geschriebene Dokumente in dem Ordner ``docs/manual`` legen.


Doku für ein Paket aktivieren
=============================

Um Dokumentation für ein Paket wie oben beschrieben, schreiben zu können, muss das bauen der Doku für das
entsprechende Paket aktiviert sein.

Dadurch wird auch automatisch der Ordner ``docs/`` angelegt und eine initiale ``docs/conf.py`` und ``docs/index.rst``
Datei angelegt.

#) ``package.xml``:
    Die ``package.xml`` beschreibt das Paket.
    Wir fügen ihr nun eine Abhängigkeit auf das Paket `bitbtos_docs` hinzu.

    .. code-block:: xml

        <package>
            <depend>bitbots_docs</depend>
        </package>

#) ``CMakeLists.txt``:
    In der ``CMakeLists.txt`` wird definiert, wie ein Paket gebaut wird.

    Zuerst nutzen wir die Abhängigkeit auf `bitbots_docs`, um zusätzliche CMake Kommandos verfügbar zu machen:

    .. code-block:: cmake

        find_package(catkin COMPONENT bitbots_docs)

    Dadurch wurde das Kommando `enable_bitbots_docs()` geladen, welches Dokumentation für dieses Paket aktiviert.

    .. code-block:: cmake

        enable_bitbots_docs()

#) ``.gitignore``:
    Diese Anpassungen sind nicht zwingend jedoch empfohlen, um das Git Repo nicht mit überflüssigen
    Dateien vollzumüllen:

    .. code-block:: text

        # auto-generated documentation
        **/docs/_build
        **/docs/_out
        **/docs/cppapi
        **/docs/pyapi


CI für ein Repository aktivieren
================================

Damit die Dokumentation von Paketen auch automatisch gebaut wird, muss unserem CI-System (Jenkins) auch beschrieben
werden, wie dies geht.

.. todo:: Jenkins Dokument referenzieren

Dafür muss die Datei ``Jenkinsfile`` im Hauptordner des Git Repositories angelegt werden.
Die Jenkinsfile ist in groovy zu schreiben, was ähnlich wie Java ist.

.. note:: Nur `<package-name>` muss geändert werden:

.. code-block:: groovy

    @Library('bitbots_jenkins_library') import de.bitbots.jenkins.PackageDefinition

    bitbotsPipeline([
        new PackageDefinition("<package-name>", true)
    ] as PackageDefinition[])


Damit wird unsere Jenkins Bibliothek eingebunden und danach die ``bitbotsPipeline`` gestartet.
Diese braucht ein Array an Paketdefinitionen.
Der Konstruktor akzeptiert den Paketnamen und dann, ob Doku gebaut werden soll.
Da wir Doku bauen wollen, muss das 2. Argument auf true gesetzt werden.
