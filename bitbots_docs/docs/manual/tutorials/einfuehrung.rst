.. _Einfuehrung:

===========================
Einführung
===========================

Wenn du diesen Text liest, hast du dich entschieden der wohl besten AG des Fachbereichs
beizutreten - den Bit-Bots. Vielleicht hast du schon mit einigen AG-Mitgliedern gesprochen
und hast jetzt eine ungefähre Vorstellung, was es in der AG alles so zu tun gibt.
Vielleicht hast du auch schon mal mit ROS gearbeitet und die Strukturen sind dir alle
bereits bekannt. Falls jedoch nicht, dann hilft dir diese Einführung hoffentlich weiter.

Am allerwichtigsten ist es Fragen zu stellen, wenn du etwas nicht verstehst. Jedes AG-Mitglied
hilft dir gerne weiter und erklärt dir, was er oder sie gerade macht und was es in dem Bereich
zu tun gibt. Vor allem am Anfang ohne jegliche Vorerfahrung kann alles schnell sehr viel werden
und man ist leicht überfordert. Lass dich davon nicht abschrecken! Bleib am Ball und vielleicht
schaffst du es dann sogar eines Tages ihn kicken zu lassen :).

Für absolute Newbies gibt es am Anfang einen kurzen Überblick über git und die wichtigsten
shell commands.

Begriffe, die dir im Laufe dieser Einführung über den Weg laufen:
Python, ROS, Git, Shell/Terminal, Ubuntu, C++, ROS-Nodes, ROS-Messages, Git-Submodules
(Wenn dir das alles schon was sagt, reicht es vermutlich dieses Dokument einfach zu überfliegen.
Wenn dir nichts davon was sagt: dann wirst du hoffentlich viel lernen beim Lesen.)

Falls doch einmal ein Begriff nicht erklärt wird, findest du ihn vermutlich im `Glossar`_.

Git
===
Je nachdem wie vertraut du mit Softwareentwicklung bist, kennst du dich auch mit verschiedenen
Möglichkeiten aus, diese Software zu verwalten. Eine sehr beliebte Möglichkeit ist git.
Git ist ein Versionsverwaltungstool, das es dir erlaubt Änderungen rückgängig zu machen und
neue Features erst mal getrennt von der funktionierenden Software zu entwickeln. Das ist nicht
nur sehr praktisch, sondern eignet sich auch besonders gut, wenn man zu mehrt an einem Projekt
arbeitet.

Es gibt einige graphische Oberflächen für git allerdings benutzen die meisten die Kommandozeile.
Viele der Befehle sind sehr einfach und lassen sich leicht merken und sobald du komplexeren
Situationen oder Problemen gegenüberstehst, ist es einfacher diese in der Kommandozeile zu lösen.
Vorteile der grafischen Schnittstellen sind allerdings, dass man die Struktur besser sieht.

Wenn du noch nicht mit Git gearbeitet hast, werden dir hier ein paar Konzepte und ihre jeweilige
Benutzung erklärt.

Repository
----------
Ein Repository ist die größte Einheit an Software in der "git-Welt". Ein git-repository kannst
du lokal auf deinem Rechner oder auf einer der Hostingseiten wie GitHub oder dem fachschaftseigenen
Gogs anlegen. Wenn bereits ein Repository besteht, dann kannst du dieses einfach herunterladen. In
der git-Welt nennt man das clonen.

.. code-block:: bash

    $ git clone <repository> [directory]

.. ssh erklären ?

Den Befehl führst du in dem Ordner aus, in dem du das Repository haben willst. Oder du gibst
das Verzeichnis hinter dem Repository an. Was hier einfach als <repository> bezeichnet wird ist
nicht nur der Name, sondern eine URL, also die Adresse des Repositories im Internet. Diese bekommst
du sowohl bei GitHub als auch bei Gogs auf der Übersichtsseite des Repositories.

Commits
-------

Wenn du jetzt Dateien in dem repository änderst, hinzufügst oder löschst, dann müsst du
diese Änderungen dem git hinzufügen. Dies machst du mit git add. Was ich persönlich als sehr
hilfreich empfinde, ist es, wenn die konkreten Änderungen nochmal aufgeteilt und angezeigt werden,
bevor du sie hinzufügst. Dadurch bekommst du einen besseren Überblick. Das machst du mit

.. code-block:: bash

    $ git add -p

Wenn du die Änderungen hinzugefügt hast, dann kannst bzw. musst du sie committen. Müssen deshalb,
weil wenn du sie nicht committest werden sie überschrieben, wenn du das nächste Mal pullst. Darauf
weist dich git aber vorher hin. Mit git status kannst du auch sehen, welche Änderungen du bereits
hinzugefügt hast (grün) und welche noch nicht (rot).

.. code-block:: bash

    $ git status

Wenn du die Änderungen dann committest musst du eine Nachricht angeben, in der du die gemachten
Änderungen beschreibst. Wenn du einfach nur ``git commit`` als Kommando nutzt, dann gelangst du in ein
editor, den wenige wirklich zu bedienen wissen. Daher mein Tipp:

.. code-block:: bash

    $ git commit -m 'Gib in Anfuehrungszeichen die Nachricht an'

Bitte beachte für die Commit Message Commit-Richtlinien (:doc:`Coding Style<../software/coding_style>`, Abschnitt Git).

Wenn man aufhört an dem git zu arbeiten oder einfach mal so "speichern" will, was man getan hat,
dann pusht man seine committs. Dies führt dazu, dass sie nicht mehr nur lokal verfügbar sind,
sondern auch für alle anderen sichtbar werden.

Workflow
--------

Der normale Workflow mit git ist so, dass man zuerst die neueste Version herunterlädt mit
``git pull``. Dann arbeitet man an dem Projekt und erstellt am Besten nach jeder Änderung oder Bearbeitung
einen Commit (davor adden). Wenn man fertig ist oder die Änderungen für die anderen verfügbar machen
will, dann pusht man.

Branches
--------

Als kleines Extra: Man kann sich die Struktur als Baum vorstellen. Der Baum hat mehrere Äste, in der
git-Welt branch. Es gibt per default immer einen Hauptast, den ``master``. Wenn man jetzt neue features
hinzufügen will oder eine alternative Version gestalten will, dann erstellt man einen neuen branch.

.. code-block:: bash

    $ git branch <name-des-neuen-branches>

Auch für das Benennen von Branches haben wir :doc:`Richtlinien <../software/coding_style>` (Abschnitt Git).

Wenn du den Branch wechseln willst, dann kannst du das machen. Achtung: da man nicht sofort sieht, auf
welchem Branch man sich grad befindet, kann es da schnell zu Fehlern führen. Da ist eine GUI z.B.
hilfreich. Mit einfach nur ``git branch`` wird dir angezeigt, auf welchem Branch du dich gerade
befindest.

.. code-block:: bash

    $ git checkout branch-auf-den-du-wechseln-willst


Shell
=========
Die Shell, eigentlich Unix-Shell genannt ist die Schnittstelle zu unixoiden Systemen wie GNU/Linux oder MacOs. Windows hat zwar auch eine Kommandozeile, die ist aber nicht wirklich vergleichbar.
Da du hier deine Befehle eintippst, die dann gleich ausgeführt werden, heißt sie auch Kommandozeileninterpreter.
Es gibt unterschiedliche Arten von Shells. Wenn du dich noch nie damit auseinander gesetzt hast, dann benutzt du
vermutlich die Bourne-Again-Shell, kurz bash.
Wenn du bereits Ubuntu installiert hast, dann kannst du über Strg + Alt + T eine Shell öffnen. Ansonsten kannst
du aber auch in deinen Programmen danach suchen, hierbei können allerdings verschiedene Namen vorkommen
wie z.B. Terminal, Command Prompt, ...

Überblick
---------
Kommandozeilen sind am Anfang schwer zu lernen, aber so wie meistens in der Informatik, lohnt es sich den
anfänglichen großen Aufwand auf sich zu nehmen, da du, wenn du die wichtigsten Kommandos beherrscht, sehr viel
schneller und effektiver voran kommst als ohne dieses Wissen.
Du kennst das vielleicht schon von LaTeX, Photoshop oder anderen Programmen.
Wenn du die bash öffnest, dann siehst du vermutlich nicht viel mehr als den Prompt. Der Prompt
heißt auch Eingabeauforderung und sieht in etwa so aus:

.. code-block:: bash

    benutzername@geraetename: ~/verzeichnis$ echo "hier könnte dein kommando stehen"

Du kannst also bereits auslesen, als welcher Benutzer du angemeldet bist, auf welchem Gerät du die Befehle
ausführen wirst und in welchem Ordnerverzeichnis du dich befindest. Die Tilde (~) verweist dabei auf das Home-
Verzeichnis deines Benutzers.

Kommandos
---------
Die wichtigsten Kommandos, die du immer wieder brauchen wirst, werden dir hier kurz erklärt.

:man: steht für "manual" und gibt Informationen zur Benutzung eines Kommandos oder einer Anwendung. Mit ``man man`` wird zum Beispiel das Manual für den Command ``man`` geöffnet.
:cd: steht für "change directory" und sorgt dafür, dass du in einen anderen Ordner wechseln kannst
:mkdir: erstellt ein neues Verzeichnis, also einen neuen Ordner
:touch: erstellt eine neue, leere Datei, falls sie noch nicht existiert
:cp: kopiert Dateien und mit -r (für rekursiv) auch Verzeichnisse
:rm: löscht Dateien und mit -r auch Verzeichnisse (ACHTUNG: die gelöschten Objekte werden von der Festplatte (meist unwiderruflich) gelöscht und landen NICHT im Papierkorb
:echo: zeigt einen Text an
:exit: beendet die Sitzung und schließt das Interface
:ls: listet alle Dateien und Verzeichnisse auf
:sudo: gibt dir Root-Rechte
:ln: erstellt einen Link zu einer Datei oder einem Verzeichnis

ROS (Theorie)
=============
Das Robot Operating System (ROS) ist eher weniger ein Betriebssytem und mehr ein Framework, das ermöglicht
austauschbare Software für Roboter zu schreiben. Erreicht wird dies durch eine Abstrahierung der Hardware
und der Bereitstellung vieler nützlicher Tools, Bibliotheken und Nutzung von Konventionen.

Node
----
Eine Node ist ein Programm, das eine bestimmte Aufgabe erfüllt. Sie kann Informationen von anderen Nodes über
Messages erhalten, in dem sie sich bei diesen als Subscriber meldet und sie kann auch selbst Nachrichten
veröffentlichen (publishen), die von anderen Nodes abonniert werden können. Eine Node in der Vision kann zum
Beispiel für die Ballerkennung verantwortlich sein und auf dem Topic **/ball_candidates** publishen.

Master
------
Damit die einzelnen Nodes miteinander kommunizieren können, stellt der master eine Verbindung zwischen den Subscribern und Publishern her. Dafür muss jede Node mit dem Master kommunizieren können und daher irgendeine Art von Verbindung zu ihm aufbauen können. 

Message & Topics
-----------------
Die Kommunikation bei ROS läuft über Messages asynchron, das heißt, dass die Nodes, die Nachrichten senden, nicht wissen,
wer diese empfängt, sondern einfach ihre Daten veröffentlichen. Nachrichten sind dabei recht einfach gehaltene
Datenstrukturen, die aus (primitiven) Datentypen und Arrays bestehen können oder aus geschachtelten Strukturen.

Topics dienen einem unidirektionalen Streaming und stellen die Busse dar, über die die Kommunikation mit den Messages läuft.
Ein Topic hat immer einen Namen und auf diesem Topic können Nachrichten gepublisht werden und/oder subscribed. Die
Kommunikation läuft dabei anonym ab, d.h. wie grade schon erwähnt, wissen die einzelnen Nodes nicht, wer die Nachrichten empfängt
oder woher sie kommen.

Service
-------
Services bieten die Möglichkeit Anfrage-Antwort-Kommunikation durchzuführen. Sie verwenden dazu spezielle request und reply
Messages. Eine ROS Node stellt dann einen Service unter einem Namen zur Verfügung und wenn jemand diesen Service nutzen möchte,
dann sendet er eine request Message und wartet auf das reply.

Bags
----
Wenn man eine Node geschrieben hat, dann sollte man diese auch testen. Dazu braucht man Daten und diese Daten bekommt nur über die
Hardware. Weil man aber nicht immer alles sofort auf dem Roboter testen möchte und sollte, gibt es Bags. ROS Bags sind bestimmte
Formate, um ROS Messages zu speichern. Sie lassen sich sehr einfach aufzeichnen und auch abspielen.


Was du tun musst, um anzufangen 
================================
Hier folgt jetzt der eher praktische Teil und der wohl wichtigste Teil des gesamten Einstiegs.

Installation
------------
Um ROS benutzten zu können, muss es installiert werden. Es gibt mehrere Versionen von ROS, wir arbeiten mit `ROS Melodic`_ , im Folgenden
oft als ROS M bezeichnet.
ROS läuft leider nicht unter jedem Betriebssytem ohne Probleme, daher ist auf den Laborrechnern `Ubuntu 18.04`_ installiert. Wenn du es also
unter Ubuntu installierst, wählst du den einfachsten Weg, allerdings kannst du es auch in anderen Linuxdistributionen (und vielleicht sogar
unter MacOS) über `rosdocked`_ laufen lassen oder aus den `Quellen kompilieren`_. Finn und Timon
haben das schon gemacht und können dir in dem Fall bestimmt helfen.

Wenn du nicht gleich das Betriebssystem wechseln willst, kannst du auch eine VM oder über ein Dualboot-Setup Ubuntu installieren ohne auf dein altes Betriebssystem verzichten zu müssen. 

Um Ubuntu zu installieren, flasht du am besten einen USB-Stick mit dem Installations-Image und wenn du dann deinen Rechner neustartest, kannst du ihn von dem Stick aus booten und dann installieren. Wenn du das jetzt nicht verstehst, macht das nichts. Es gibt sehr gute Schritt-für-Schritt-Anleitungen, die sehr ausführlich beschreiben und zeigen, was du machen musst. Und falls du dann noch fragen hast, kannst du jederzeit Menschen im Labor fragen.

Wenn du nicht weißt, wie man die Kommandozeile benutzt, dann geh nochmal nach oben und lies dir den Abschnitt dazu durch.

Wenn du jetzt Ubuntu installiert hast, kannst und musst du ROS installieren. Dies kannst du zum einen "händisch" selber machen, dafür findest du eine gute Anleitung im ROS-Wiki, die dir sagt, welche Kommandos du ausführen musst. 
Viel praktischer ist es allerdings über ein Skript, das in `bitbots_meta`_ liegt. Um dieses nutzen zu können musst du allerdings das repository clonen. Falls du nicht weißt, wie das geht, dann kannst du es oben nachlesen.

Um alles notwendige zu installieren wechselst du ins Verzeichnis bitbots_meta und führst dort ``make install`` aus:

.. code-block:: bash

	nutzer@geraet:~/bitbots_meta$ make install

Jetzt hast du nicht nur ROS mit allen Dependencies installiert, sondern auch das repository mit allen Submodules aktualisiert. Außerdem wurde dir ein catkin Workspace unter ``~/catkin_ws`` eingerichtet.

Bauen
-----

Bevor du die Software ausführen kannst, musst du sie bauen. Dazu gibt es den Befehl **catkin build**. 
`Catkin`_ ist von ROS bereit gestellt und hilft beim bauen von C++ (und anderem) über die CMake-Files. Um Bauen zu können, musst du in dein Workspace wechseln.
Wenn du make install ausgeführt hast, dann wurde dir bereits ein Workspace nach dem Standard (~/catkin_ws) erstellt. In diesen musst du wechseln, um den Befehl ausführen zu können.
Bevor du catkin build auführen kannst, musst du zu erst nochmal deine shell sourcen. Das machst du mit `source ./devel/setup.<insert your shell here>` (z.B. bash).
Dies ist der manuelle Weg um zu bauen.
Alternativ kannst du auch **make build** in bitbots_meta ausführen.
Dabei werden aber immer alle Packages gebaut, das kann teilweise sehr viel Zeit in Anspruch nehmen. Wenn du nur einzelne Pakete bauen willst, dann musst du mit ``catkin build package_name`` verwenden. 



Der Aufbau unserer Software
============================

Die wichtigsten ros-Kommandos vorab
-----------------------------------
Allgemein gilt: 
- es können mehrere Parameter übergeben werden
- hier werden die Paketnamen gebraucht, **nicht** die Submodules (das kann vor allem am Anfang verwirrend sein, da sich aber eigentlich alles mit *tab* automatisch vervollständigen lässt, 
solltest du recht schnell merken, ob du den richtigen Namen verwendest)

Um einzelne Nodes zu starten nutzt man rosrun (alles in Großbuchstaben sind Platzhalter).

.. code-block:: bash

	$ rosrun PAKETNAME NODE(.py)

Um ein Launchfile zu starten nutzt man roslaunch

.. code-block:: bash

	$ roslaunch PAKETNAME LAUNCHFILE.launch PARAMETER:="VALUE"

Um Informationen zu einem Topic zu bekommen, nutzt man rostopic; meistens echo (um die Inhalte der Messages auf diesem Topic sehen zu können), hz (um zu sehen in welcher Rate die msgs gesendet werden), pub (um selber eigene Messages zu publishen) und list (um zu sehen auf welchen topics was gepublisht wird).
Dieses Kommando (rostopic) ist vor allem zum Debuggen sehr praktisch und kann mehr Einblicke in das Geschehen liefern. 

.. code-block:: bash

	$ rostopic echo/list/pub/hz TOPICNAME ...


Unsere Verwendung von Git
-------------------------

Wir verwenden zum einen den Fachschaftseigenen Dienst `Gogs`_, den man über mafiasi erreicht und zum anderen `Github`_. Tatsächlich nutzen wir gerade eigentlich hauptsächlich Github, allerdings liegen im Gogs noch ein paar nicht-öffentliche Gits.

Die Software ist so aufgebaut, dass das Repository `bitbots_meta`_ nochmals in einzelne *Git-Submodules* unterteilt, welche jeweils eine übergeordnete Aufgabe abdecken. Die Vision oder das Behaviour sind zum Beispiel eigene Submodules. Diese Submodules sind im Prinzip einfach weitere Unterordner, die jeweils von einem eigenem Git verwaltet werden.
In jedem Submodule gibt es nochmals (Catkin-)Pakete für die einzelnen konkreten Aufgaben innerhalb des großen Aufgabenbereichs.

**ACHTUNG: Submodules und Pakete sollten nicht verwechselt werden!** Was ein Paket und was ein Submodule ist, erkennt man recht schnell, wenn man tab completion benutzt. Denn die git-Befehle funktionieren nur mit den Submodules und die ros-Befehle nur mit den Paketen (s.o.).

In den einzelnen Paketen
------------------------
In den einzelnen Paketen gibt es mehrere verschiedene Unterordner. Die meisten Pakete haben diese Ordnerstruktur:

- config
- docs
- launch
- src

und dann gibt es noch in dem übergeordneten Packageordner die CMakeLists.txt, package.xml, rosdoc.yaml und ein setup.py. Diese Dateien lassen wir erstmal außen vor. Die anderen Ordner schauen wir uns kurz näher an.

*config*
Im config Ordner liegen YAML-Dateien. In diesen werden bestimmte Werte/Parameter spezifiziert. Diese Parameter dienen der Konfiguration (daher der Name config). Da alle zu setztenden Parameter dort gemeinsam an einem Ort liegen, findet man schnell was man sucht und muss sich nicht in den Tiefen der Ordnerstrukturen verlieren. Diese Konfigdateien können innerhalb des Codes geladen werden und werden so verfügbar.
In den Launchfiles kann auch spezifiziert werden, welche config-Dateien wann geladen werden sollen. Zum Beispiel kann man, wenn man ein Spiel vor sich hat, die game_settings.yaml laden. Standardmäßig wird dies nicht getan.

*docs*
In diesem Ordner befindet sich die Dokumentation für das Paket. Diese wird auch automatisch über
Catkin gebaut. Meistens musst du dich mit dem Bauen der Dokumentation aber nicht auseinandersetzen,
sondern kannst sie online lesen. Wie man Dokumentation schreibt, ist unter :doc:`Hot to Doku
<Doku-How-To>` dokumentiert.

*launch*
Launch-Dateien starten eine oder mehrere Nodes. Das ist sehr praktisch, denn ansonsten müsste man jede einzelne Node mit ihren spezifischen Konfigurationen einzeln über das Terminal mit rosrun starten. 
Launch-Files sind im XML-Style gehalten und sehen meist recht ähnlich aus, sie haben die Dateiendung '.launch'. Normalerweise "deklariert" man am Anfang der Datei ein paar Argumente
und gibt ihnen einen Defaultwert. Diese Parameter können beim Aufrufen der Datei mit roslaunch gesetzt werden. 

In den <group>-Klammern kann man Fallunterscheidungen einbinden und über das $-Zeichen gibt man an, dass hier der Name des arg nacher durch den tatsächlichen Wert des Parameters ersetzt wird.
Über Include kann man andere Launchdateien einbinden. So entsteht bei uns zum Beispiel eine genestete Struktur, in der ein Launchfile ein anderes aufruft, welches ein anderes aufruft, welches en anderes aufruft...
Mit rosparam kann man die Parameter aus den einzelnen Konfigurationsfiles einbinden (s. config).
Node startet einfach die angegebene Node, in dem spezifizierten Paket.

Launchdateien (s. u.) gibt es häufig in doppelter Ausführung, wobei eine das Suffix `_standalone` trägt. Das 
bedeutet, dass zusätzlich zum eigentlichen Node, der gestartet wird, auch der *Robot State Publisher* gestartet
und das *URDF* geladen wird.

*src*
In diesem Ordner liegen alle wichtigen Programme, die sogenannten Nodes. Die meisten Nodes sind in Python, also mit '.py' geschrieben. Der Effizienz wegen, wirst du aber auch ein paar mit der Endung '.cpp' finden. Was die einzelnen Programme tun, kann sehr unterschiedlich sein. Manche sind sehr mächtig und erfüllen eher übergeordnete Funktionen, andere sind sehr speziell und erfüllen genau eine Funktionalität.
Wenn du mehr zu den einzelnen Programmen wissen willst, dann frag am besten den Maintainer des Paketes. Diesen findest du in der 'package.xml' - eines der Dokumente, die in jedem Paket vorhanden sind. Der oder die Maintainer ist dein Ansprechpartner für dieses Paket und hilft gerne weiter, wenn du Fragen hast.

Falls du diese Informationen einmal brauchen wirst, ist hier auch die Dokumentation zu
`CMakeLists`_ und `package.xml`_ verlinkt.

Allgemeines zu den Bit-Bots
---------------------------
Wir treffen uns einmal die Woche und besprechen wer was gemacht hat, welche Termine anstehen, wichtige Deadlines und anderes wichtiges Zeugs. Man kann sehr viel in den Weeklys lernen, komm also vorbei wenn du Zeit hast. Momentan finden die Weeklies immer *Mittwoch, 18 Uhr* statt.
Wann immer du etwas für die Bit-Bots tust, trägst du dir (wie auf Arbeit) die Zeit ein. Das hört sich erstmal komisch an, ist aber wichtig, wenn du mit auf die Wettbewerbe fahren willst.
Dafür brauchst du einen Account auf der `Bit-Bots-Karma`_ Website. Den kannst du dir leicht selber erstellen. Jede Minute ist dabei ein Karma-Punkt.
Die Regeln für Wettbewerbe und Karma findest du im Detail im Mitgliedsvertrag.

Wenn du nach einiger Zeit in der AG entschieden hast ein Mitglied zu werden, musst du diesen Vertrag unterschreiben. Er gibt dir die Rechte abzustimmen und vieles anderes. Lies ihn dir sorgsam durch 
und bei Fragen gilt wie immer, einfach stellen.

Programmiererfahrung
====================

In dieser AG ist jeder willkommen. Es gibt sehr viele Aufgaben, die erledigt werden wollen und in diesem Team findet sich für jeden einen Platz.
Wenn du schon einiges an Vorerfahrung mitbringst und vielleicht sogar schon mal mit Robotern gearbeitet hast, dann wird dir einiges hier leichter fallen. Das Wichtigste ist aber nicht, 
was du schon weißt, sondern dass du interessiert daran bist, neues zu lernen. 

Es kann am Anfang alles sehr viel sein. Die Software, die wir schreiben, verbessern und neu entwickeln, setzt sich aus vielen kleinen Einzelteilen zusammen und oft reicht es nur die grobe Funktionsweise und die genaue Schnittstelle zu kennen, um sie benutzen zu können. Manche AG-Mitglieder haben sich innerhalb von ein paar Monaten eingearbeitet (und sehr, sehr viel Zeit investiert), andere haben dafür ein ganzes Jahr gebraucht. Lass dich auf jeden Fall nicht abschrecken, denk daran dass jeder mal klein angefangen hat.

Python (und C++)
----------------

Wir verwenden hauptsächlich Python als Sprache. Wenn du nur SE1 (und eventuell SE2) als Vorkenntnis hast, dann ist das erst mal eine ungewöhnte Umstellung, aber du wirst schnell merken, 
dass Python sehr anfängerfreundlich ist.
Am besten du machst dich zuerst mit der Syntax vertraut. Dazu kannst du online ein paar Tutorials (zum Beispiel das auf `codecademy`_) oder die offizielle `Python Doku`_ lesen.
Python ist eine recht Einsteiger-freundliche Sprache, das einzige, das manchmal zu Fehlern führen kann, sind die Einrückungen. Denn anders als in Java werden in Python keine geschweiften Klammern zur Strukturierung genutzt. Aber auch daran gewöhnt man sich schnell.

Die meisten Mitlgieder benutzen als Entwicklungsgebung `PyCharm`_ (Professional Edition), eine IDE, die für dich als Student kostenlos zur Verfügung steht und viele Vorteile bietet, die weit über Autocompletion hinaus geht. Allerdings kannst du auch jeden anderen Editor deiner Wahl benutzen. Ob Vim, Sublime, Atom oder ein beliebig anderer ist letztendlich egal.


Getting started
===============
Am besten liest zu zusätzlich zu diesem Dokument noch die `Neulingsdoku`_. Dort werden kurz alle groben Themen beschrieben. Wenn dich eines (oder mehrere) davon interessieren, dann wende dich an den Zuständigen und mach dein Interesse kund. Dir wird dann eine kleine Einführung gegeben und du kannst mal jemandem über die Schulter schauen und ein bisschen mehr darüber lernen.

Falls du auf Begriffe stößt, die du noch nicht kennst, dann kannst du diese im `Glossar`_ nachlesen. Wir bemühen uns dieses aktuell zu halten und alle wichtigen Begriffe dort zu erklären.

Am besten du suchst dir am Anfang eine (oder mehrere - wir sind alle sehr nett ;) ) Person deines Vertrauens und stellst deine Fragen sobald sie aufkommen. Jeder fängt mal klein an, also nur Mut und ran an den Code!


Vielen Dank fürs Lesen und viel Spaß bei den Bit-Bots!


.. _ROS Melodic: https://wiki.ros.org/melodic
.. _Ubuntu 18.04: http://releases.ubuntu.com/18.04/
.. _Github: https://github.com/
.. _Gogs: https://gogs.mafiasi.de
.. _bitbots_meta: https://github.com/Bit-Bots/bitbots_meta
.. _Marcs Masterarbeit: https://tams.informatik.uni-hamburg.de/publications/2017/MSc_Marc_Bestmann.pdf
.. _rosdocked: https://github.com/timonegk/rosdocked
.. _Quellen kompilieren: https://wiki.ros.org/melodic/Installation/Source
.. _Catkin: http://docs.ros.org/api/catkin/html/
.. _Python Doku: https://docs.python.org/3/tutorial/index.html
.. _codecademy: https://www.codecademy.com/
.. _PyCharm: https://www.jetbrains.com/pycharm/
.. _Bit-Bots-Karma: https://karma.bit-bots.de/
.. _Coding Style: <../software/coding_style>
.. _Glossar: http://doku.bit-bots.de/private/manual/glossar.html
.. _CMakeLists: https://wiki.ros.org/catkin/CMakeLists.txt
.. _package.xml: https://wiki.ros.org/catkin/package.xml
.. _Neulingsdoku: http://doku.bit-bots.de/private/manual/neulingsdoku.html
