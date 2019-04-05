=====================
Software Overview
=====================

Dieses Dokument soll übersichtlich und zusammengefasst erklären, wie man ROS einrichtet und BitBots Software zum laufen
bringt.

Fühlt euch frei, Informationen zu ergänzen, löschen oder ändern!

Einen etwas umfangreicheren Überblick bietet Martins :doc:`software-schnelleinstieg`. Wer sich tiefer mit dem Architektur-Thema beschäftigen möchte kann auch `Marcs Masterarbeit`_ lesen.


Installation
==============
Wir benutzen `ROS Melodic`_. Diese ROS-Version läuft unter Ubuntu 18.04 (LTS) und Debian, kann in anderen Betriebssystemen aber aus den `Quellen kompiliert`_ oder über Docker genutzt werden (rosdocked_).
Alternativ kann auch eine VM oder ein Dualboot-Setup genutzt werden. Hier wird die Installation in einem Ubuntu-System beschrieben.

1. Linux (`Ubuntu 18.04`_) installieren (notfalls VM)
2. einen SSH-Key erstellen (falls nötig) und bei Gogs_ und Github_ eintragen
3. `bitbots_meta`_ klonen
4. `make install` in bitbots_meta ausführen
    - dadurch wird das repository und Submodules aktualisiert
    - ROS und dependencies installiert
    - Ein Catkin Workspace unter `~/catkin_ws` eingerichtet
5. `make build` in bitbots_meta ausführen
    - Dadurch werden alle Pakete gebaut


Bauen
=========
- Mit make (empfohlen):
    1. `make build` in bitbots_meta ausführen

- Manuell:
    1. in catkin Workspace navigieren (Standard = `~/catkin_ws`)
    2. `source ./devel/setup.<insert your shell here>`
    3. `catkin build`


Code schreiben
===================
Am bestern vergewissern, dass ihr auf den Richtigen Branches seid.
Das sind im zwifel die, die im bitbots_meta für die submodules angegebn werden.

:Aktueller stand: alle Module stehen auf master

ROS ganz grob erklärt
------------------------
ROS besteht im Prinzip aus verschiedenen *Nodes*, welche über *Messages* miteinander interagieren können.
Diese Nodes haben per design keine statischen Abhängigkeiten aufeinander sondern subscriben nur auf vorher definierte
Message-Typen und reagieren, wenn so eine Message eintrifft. Andersherum kann eine Node auch Messages mit vorher
definiertem Typ publishen.

Ein seperater *Roscore* ist dafür verantwortlich die verschiedenen Publisher und Subscriber "miteinander bekannt zu
machen". Jede Node muss deswegen den Roscore irgendwie erreichen (Netzwerk oder lokal).

Durch dieses Design ist es relativ einfach möglich statische Abhängigkeiten zwischen verschiedenen Nodes aufzuheben
und den Code so modularer zu gestalten.

Zusätzlich dazu können Nodes in Paketen gebündelt werden, welche von `catkin` einzeln gebaut werden können.

Bitbots Conventions
----------------------
Das Repository `bitbots_meta` ist in verschiedene *Git-Submodules* unterteilt, welche jeweils einen abstrakten Teil der
Aufgaben im Roboter abdecken (z.B. `bitbots_vision` für alles, was mit Bilderkennung zu tun hat).
Diese *Submodules* sind praktisch nur weitere Unterordner in `bitbots_meta`, welche von einem eigenen *Git* verwaltet
werden.

In jedem *Submodule* kann es ein oder mehrere *catkin Pakete* geben, welche einen konkreten Aufgabenbereich abdecken.
(z.B. `bitbots_head_behaviour` für die Kopfbewegung / Ballsuchverhalten)

**Sei vorsichtig, Submodules und Pakete nicht zu vertauschen**

Launchdateien (s. u.) gibt es häufig in doppelter Ausführung, wobei eine das Suffix `_standalone` trägt. Das 
bedeutet, dass zusätzlich zum eigentlichen Node, der gestartet wird, auch der *Robot State Publisher* gestartet
und das *URDF* geladen wird.


ROS und Nodes starten
=======================
Dank der modularen Architektur von ROS müssen nicht immer alle Nodes gestartet werden.
Stattdessen können *launch files* erstellt werden, welche beschreiben, welche Nodes für einen bestimmten
Kontext gebraucht werden.

Auserdem müssen nicht alle launch files auf dem gleichen Gerät gestartet werden solange der Roscore erreichbar ist.

Die Syntax zum starten von launch files lautet:

:code:`roslaunch <paket> <launch file>`

Alternativ kann auch eine einzelne Node gestartet werden

:code:`rosrun <paket> <ausführbare Datei>`

**Beide Befehle unterstützen Tab-Vervollständigung**

Beispiel
----------
- Einzelne launch file starten:
    Es empfiehlt sich einen *roscore* seperat zu starten:

    :code:`roscore`

    Nun kann die gewünschte launch file einfach gestartet werden:

    :code:`roslaunch bitbots_head_behaviour head_behaviour_standalone.launch`

- Verschiedene Nodes auf verschiedenen Geräten:
    Es empfiehlt sich einen *roscore* seperat zu starten. Das Gerät mit dem roscore wird weiterhin als *Host*
    bezeichnet.

    :code:`roscore`

    Auf dem gleichen Gerät können Nodes einfach gestartet werden

    :code:`roslaunch <...>`

    Auf anderen Geräten muss vor dem starten die Adresse des Roscore eingestellt werden. Dies geschieht durch die
    Environment Variables `ROS_MASTER_URI`

    :code:`export ROS_MASTER_URI=http://<Ip-Adresse>:11311`

    :code:`roslaunch <...>`


Simulator
===========
Wir benutzen die Simulator-Software Gazebo. Für den Simulator gibt es eigene Launchfiles, die gestartet sein müssen
aber ansonsten sollten alle Nodes normal funktionieren.

Für den Simulator selbst

:code:`roslaunch bitbots_bringup simulator.launch`

Für den Roboter darin

:code:`roslaunch bitbots_bringup teamplayer_simulated.launch`



.. _ROS Melodic: https://wiki.ros.org/melodic
.. _Ubuntu 18.04: http://releases.ubuntu.com/18.04/
.. _Github: https://github.com/
.. _Gogs: https://gogs.mafiasi.de
.. _bitbots_meta: https://github.com/Bit-Bots/bitbots_meta
.. _Marcs Masterarbeit: https://tams.informatik.uni-hamburg.de/publications/2017/MSc_Marc_Bestmann.pdf
.. _rosdocked: https://github.com/timonegk/rosdocked
.. _Quellen kompiliert: https://wiki.ros.org/melodic/Installation/Source
