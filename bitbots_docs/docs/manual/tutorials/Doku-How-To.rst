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
        **/docs/cppapi
        **/docs/pyapi

#) ``Jenkinsfile``:
    Die Jenkinsfile ist nicht für die Doku an sich notwendig jedoch steuert sie unsere CI und damit das automatische Bauen der Doku.
    Die Jenkinsfile ist in groovy zu schreiben, was ähnlich wie Java ist.
    
    .. todo:: Jenkins Dokument referenzieren

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


Ordnerstruktur
--------------

Fertige Dokumentation kann unter ``docs/_out/index.html`` lokal abgerufen werden.

.. todo:: Genauer beschreiben. Vor allem, welche Dateien wie wiederhergestellt werden

Doku selber bauen
-------------------

Sobald die Doku das erste Mal gebaut wird, wird automatisch ein neuer Unterordner `docs` im Paket erstellt. 
Dieser soll von nun an die Dokumentation beherbergen.


Diese allgemeine Dokumentation (die mit Tutorials inklusive dieser Seite) kann mit dem Command ``catkin build bitbots_docs
--no-deps --make-args Documentation`` gebaut werden.

Für ein anderes Paket, bei dem die Doku wie oben beschrieben aktiviert wurde, kann sie mit
``catkin build <package> --no-deps --make-args Documentation`` gebaut werden.
