===============
How to document
===============

Our documentation is published under  `doku.bit-bots.de <http://doku.bit-bots.de>`_ and will automatically be regenerated from the package `bitbots_meta/bitbots_tools/bitbots_docs <https://github.com/bit-bots/bitbots_tools/tree/master/bitbots_docs>`_ and deployed by our `Jenkins CI <http://ci.bit-bots.de>`_.

Installation of system dependencies
===================================

We are using `Sphinx <https://www.sphinx-doc.org/>`_ and the following extentions to generate our documentation: `breathe`, `exhale`.
The correct version of the extentions must be installed.

.. code-block:: bash

        sudo apt remove python-sphinx
        sudo apt install python3-sphinx python3-sphinx-rtd-theme python3-breathe
        pip3 install exhale --user


How to build the documentation
==============================

* To build the general documentation (tutorials etc. including this one), run the following command:

  .. code-block:: bash

          catkin build bitbots_docs --no-deps --make-args Documentation

* To build documentation for another package (with :ref:`activated<activate_docs_for_package>` documentation), run the following command by replacing ``<package>`` by your package name:

  .. code-block:: bash

          catkin build <package> --no-deps --make-args Documentation


How to write documentation for a package
========================================

In every package (with :ref:`activated<activate_docs_for_package>` documentation) you can find a directory, called ``docs/`` including the configuration file (``docs/conf.py``) and the root-document (``docs/index.rst``).

Preferably create your ``.rst`` documents in the directory ``docs/manual``, then reference them in the ``docs/index.rst`` as follows::

    .. toctree::
        :maxdepth: 1
        :glob:
        :caption: Manuals:

        manual/*

.. _activate_docs_for_package:

Activate documentation for a package
====================================




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

#) ``Jenkinsfile``:
    Die Jenkinsfile ist nicht für die Doku an sich notwendig jedoch steuert sie unsere CI und damit das automatische Bauen der Doku.
    Die Jenkinsfile ist in groovy zu schreiben, was ähnlich wie Java ist.

    .. seealso:: :doc:`../software/ci` for a more detailed description of how our CI works.

    .. note:: Nur `<package-name>` muss geändert werden:

    .. code-block:: groovy

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
