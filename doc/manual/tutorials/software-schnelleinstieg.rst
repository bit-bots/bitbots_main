.. _Software-Schnelleinstieg:

==========
Software Schnelleinstieg
==========

Die original LaTex Version gibt es auf Overleaf_.

.. _Overleaf: https://www.overleaf.com/read/ftfndsgzvgdb#/26193133/


Willkommen
==========

Vielen Dank, dass du dich für die Mitgliedschaft in dieser Qualitäts-AG
aus dem Hause Mafiasi entschieden hast!

Der Hamburg Bit-Bots Schnelleinstieg, ein Leitfaden für alle, die neu in
der AG sind oder sich manchmal so fühlen. Ein Einstieg zum Mitlachen,
Mitfiebern und Wohlfühlen. Für gemütliche Stunden im RoboCup-Labor oder
als Gute-Nacht-Lektüre nach einem stressigen Tag voller
Mathevorlesungen. Zum Nachschlagen gibt es am Ende noch eine
Befehlsübersicht und Checklisten.

Das folgende Dokument soll einen kleinen Einstieg (Überfliegen 20 Min,
Mitarbeiten 1-2h) in die AG geben, oder als schnelle Referenz dienen.
Der Fokus liegt hierbei auf dem Bereich der Softwareentwicklung, aber
auch für Interessierte der Bereiche Hardware und Orga ist die Lektüre zu
empfehlen, so ist es doch essentieller Bestandteil der AG, Software auf
dem Roboter zum Laufen zu bekommen.

Zum Jahreswechsel 2016/17 haben wir angefangen von unserem alten,
eigenen Framework auf ROS umzusteigen. Entsprechend soll der Leitfaden
hier helfen, sich in dem Bereich zurechtzufinden.

Du solltest bereits Teil der RoboCup-Mafiasigruppe sein und auch im Gogs
(Quellcodeverwaltung) zur Bit-Botsgruppe hinzugefügt worden sein. Falls
nicht, sprich einen der Alt-AGler an.

ROS
===

ROS ist das meist verbreitete Framework für Robotiksysteme. Vor einigen
Jahren noch als zu sperrig verschrien, hat sich viel getan und ROS wird
in vielen Bereichen eingesetzt. Die Entwicklung in ROS erfolgt in C++
oder Python. Während Python einfacher in der Bedienung und schneller zu
lernen ist, hat C++ einen klaren Geschwindigkeitsvorteil. Für den
Einstieg wählen wir aber Python, die Syntax ist klar und strukturiert,
die Lernkurve sehr vorteilhaft.

Wir arbeiten auf ROS Kinetic, der 2016/17 aktuellsten LTS ROS-Version.
Die Installation und Einrichtung ist unter Ubuntu vergleichsweise
einfach (http://wiki.ros.org/ROS/Installation/). Wird eine andere
Linuxdistribution (Fedora, Arch,..) verwendet, können (vermutlich)
keinen Packages aus den Repositories verwendet werden, sondern der
Quellcode muss selbst kompiliert werden, das wird auch im Roswiki unter
*Sourceinstall* erklärt. Bei Problemen wende dich einfach persönlich an
einen der alt-AGler. Die Installation auf Ubuntu macht einem das Leben
aber deutlich einfacher. Folge hier am besten der Anleitung, zu
empfehlen sind die Varianten desktop oder desktop-full (mit Simulator
etc). Je nachdem was du machen willst, können noch weitere Pakete
notwendig sein. Der Sourceinstall braucht eine gewisse Zeit zum
Kompilieren. Im besten Fall erstellst du dir in diesem Fall einen
zweiten Workspace für den eigenen Code. Wenn die ROS-Befehle nicht
funktionieren, überprüft, ob ihr die Umgebungsdatei gesourced habt
(source **./devel/setup.bash** bzw. ./installed_isolated/setup.bash) Ihr
könnt dies auch unter **~/.bashrc eintragen**, das erspart das
regelmäßige Neu-Eintippen).

Um vollständig mit Python3 arbeiten zu können (speziell Simulator) haben
wir ein paar wenige Pakete selber gebaut, die im Anschluss noch
ausgetauscht werden können. Bei einer Sourceinstallation entfällt dies,
da entsprechendes upstream schon behoben ist.

Alternativ kannst du die Rechner in unserem Labor benutzten, auf denen
dies schon eingerichtet ist.

| Ihr müsst jetzt noch einen Workspace anlegen, in dem ihr den Code
  baut. **mkdir -p ~/ws/src** Der Ordner ,,ws” wäre in dem Fall euer
  Workspace. Die meiste Arbeit erfolgt wie unter Linux oft üblich in der
  Konsole. ROS bietet die Bindings für bash und zsh. (Bash ist meist der
  Standard. Wenn du das alles nicht kennst, benutzt du bash). Siehe dazu
  im Zweifel hier:
  http://wiki.ros.org/catkin/Tutorials/create_a_workspace.
| Ein paar generelle Worte zu dem Aufbau von ROS, bevor wir zu einem
  kleinen Beispiel kommen: Jede Aufgabe wird von einem Programm
  erledigt, einem sogenannten Node. Hierbei handelt es sich um einen
  eigenständigen Prozess. Dieser wird einmal gestartet und soll solange
  laufen bleiben, wie er benötigt wird, also meist dauerhaft. Daten
  werden mit Messages zwischen den einzelnen Nodes verteilt. Dazu
  ,,published” ein Node eine ,,message” zu einem ,,topic”, einem
  Identifier, was das für eine Nachricht ist, und ein (anderer) Node
  ,,subscribed” die Nachricht. Wird nun eine Nachricht losgeschickt,
  wird der subscribende Prozess eingefroren und die definierte
  Callback-Methode aufgerufen und zum Beispiel in der Klassenvariablen
  geändert.

Entweder wird gleich hier nun die Aufgabe gemacht und eine neue Message
herausgegebenen, oder nur etwas gespeichert und zum Beispiel erst beim
nächsten Durchlauf einer Schleife wird mit den neuen Daten gearbeitet.

Das Prinzip ist recht einfach und da die Messages klar definiert sind
lassen sich schnell Nodes austauschen und so verschiedene
Implementationen evaluieren.

Tools
=====

Für die Entwicklung mit Python empfehlen wir PyCharm, eine sehr gute
IDE. Kostenlos in der Community Edition und nach Anmeldung ist für
Studenten auch die Professional Version kostenlos. Einfach downloaden,
entpacken und ausführen (einmalig aus der Konsole im ./bin Ordner). Wenn
die Code Autovervollständigung richtig funktionieren soll, müssen ein
paar Kleinigkeiten eingestellt werden:
http://wiki.ros.org/IDEs#PyCharm_.28community_edition.29 oder die Ordner
*devel/lib/python2.7/{dev,side}-packages/* als source root setzen.

Die Codeverwaltung machen wir mit Git, einem praktischem Werkzeug,
welches es ermöglicht mit vielen Leuten zusammen an einer Codebase zu
arbeiten. Um den Code auszuchecken, geht auf
https://gogs.mafiasi.de/Bit-Bots und holt euch (am besten mit **git
clone** über ssh) entweder alle benötigten Subrepositories, oder
**bitbots_meta**, das Tools sowie alle anderen Repositories enthält.
Dort gibt es auch das Skript **./pull_all.sh** welches rekursiv alle
Repositories updatet, dies solltet ihr initial ausführen. Jedes Modul
kann aber auch einzeln geupdatet werden, was auch bei möglichen
Mergekonflikten zu empfehlen ist, zum Beispiel über PyCharm, die Konsole
einzelnen oder mit **git submodule foreach git pull**. Wenn ihr auf dem
Master Änderungen pusht (hochladet) dann solltet ihr daran denken, dass
wir unseren Code auch nach GitHub weiterleiten.

Wir benutzten zum Programmieren Python in der Version 3.5, einzelene
Module wie "cv2" können Probleme machen, hier gibt es aber im Zweifel
Workarounds. Um alle benötigten Module zu installieren, gibt in der
Pythonwelt verschiedenen Möglichkeiten. Wenn nicht alle Abhängigkeiten
global installiert werden sollen, kann mit **./create_venv.bash** das
Virtualenv erzeugt werden, das sind getrennte Python-,,Container” in
denen die Abhängigkeiten lokal installiert werden
(http://docs.python-guide.org/en/latest/dev/virtualenvs/#lower-level-virtualenv).
Dies dauert ggf. ein paar Augenblicke. Ihr könnt in der Konsole ein
Virtualenv aktivieren mit **source path/to/git/env3/bin/activate**. In
viele Fällen spricht aber nichts dagegen die Abhängigkeiten systemweit
ohne Virtualenv zu installieren. Es kann bei der Benutzung der
Systempakete Problemen vorbeugen, wenn alle Pythonpakete für je *Python2
und Python3* systemweit installiert wird. Das erspart auch das
Aktivieren der Umgebung in jeder Konsole.

Das Bauen der Software
======================

Kompiliert (von C++, bei Python werden nur die Abhängigkeiten (nicht die
Syntax) geprüft) wird mit *catkin*. Zum Bauen führt man in dem **catkin
workspace**, den ihr bei der ROS-Einrichtung erstellt habt,
:math:`catkin\_make` aus (wenn das nicht geht, habt ihr vermutlich nicht
eure ROS-Installation gesourced). Nun werden alle Ordner und Subordner
unter ,,src” gebaut. Damit es auch was Sinnvolles tut, solltet ihr alle
Ordner, die ihr bauen wollt, z.B bitbots_vision, aus dem von euch
geclonten Git mit *ln -s ECHTE_DATEI VERLINKUNG* verlinken, oder gleich
das ganze bitbots_meta Repository. Vor dem Ausführen der Software und
nach dem Bauen lohnt es sich das **./repair.sh** Skript auszuführen, um
alle Abhängigkeiten zu reparieren. Hat das funktioniert, können wir uns
schon daran machen unseren ersten Node zu schreiben. Wir schreiben uns
eine Vision, also den Node, der Ballkandidaten aus einem Bild
heraussucht. Wir nehmen einen simplen Ansatz, der ausreicht als
Beispiel, aber leider nicht ausreichend für ein echtes Spiel ist.

| Ein Node liegt in einem Ordner mit der folgenden Struktur:
| /packagename/
| /packagename/src/
| /packagename/src/packagename/
| /packagename/src/packagename/file.py
| /packagename/CMakeLists.txt
| /packagename/package.xml
| /packagename/setup.py

Die Dateien CMakeLists.txt, package.xml und setup.py werden von ROS
benötigt und geben unter anderem die Abhängigkeiten an. Unter *src*
liegt das eigentliche Programm. Desweiteren kann es noch mehr Ordner,
zum Beispiel für Launchfiles oder Konfigurationen geben.

Schreiben eines Nodes
=====================

Laden wir uns erst einmal ein leeres Package runter:
http://data.bit-bots.de/bitbots_beispielpackage.zip Dort müssen zunächst
CMakeLists.txt, package.xml, setup.py sowie die Pfade angepasst werden.
Für unser Beispiel reicht es die Paketnahmen von "bitbots_beispiel" zu
ändern, zum Beispiel in "bitbots_ballerkennung". Leere Pakete können
alternativ auch mit *catkin_create_pkg PAKETNAME ABHÄNGIGKEIT1 AH2 AH3*
erzeugt werden.

Das eigentliche Programm schreiben wir in einer neue Pythondatei
*ballerkennung.py*, die wir im Folgenden Schritt für Schritt durchgehen
werden. Ich ermuntere da mal jeden auch gerne experimentierfreudig
abzuweichen von dem Tutorial und zu gucken was passiert, mehr als ein
nicht laufendes Programm kann ja nicht schief gehen. Der Code sollte
auch für Pythonneulinge verständlich sein.

**Der Zahn der Zeit hat leider etwas an dem Beispiel genagt, oder, um es
positiv auszudrücken, wenn man es zum Laufen bekommt, hat man besonders
viel gelernt.->TODO überarbeiten**

::

   #!/usr/bin/env python2.7

Die erste Zeile ist die klassische shebang Zeile, welche in
Linuxsystemen angibt mit welchem Programm das Skript ausgeführt wird.

::

   import cv2
   import numpy as np
   import rospy
   from humanoid_league_msgs.msg import BallInImage, BallsInImage
   from sensor_msgs.msg import Image
   from cv_bridge import CvBridge, CvBridgeError

Der Block macht die nötigem imports bzw. includes von libraries.
Üblicherweise findet PyCharm auch automatisch die richtigen Pakete, wenn
man eine undefinierte Klasse oder Methode benutzt.

Python arbeitet nicht mit Klammerung sondern mit Einrückungstiefen für
die Verschachtelung. Klassischerweise sind 4 Leerzeichen ein Tab und
eine Tiefe.

::

   class DummyVision:
       def __init__(self):

*class* definiert unsere Klasse, *\__init_\_* ist der Konstruktor.
*self* ist das eigene Objekt. Alle Klassenvariabeln und Funktionen
werden über das Objekt zugegriffen.

::

       self.pub_balls = rospy.Publisher("/ball_candidates", BallsInImage, queue_size=1)
       rospy.Subscriber("/usb_cam/image_raw", Image, self._image_callback, queue_size=1)        

Definieren wir zunächst die Subscriber und Publisher, welche die Ein-
und Ausgaben des Nodes definieren. Die nötigen Klassen liefert das Rospy
Package. Den Publisher speichern wir in einer Klassenvariablen ab, um
später darauf zugreifen zu können. Das erste Argument ist das Topic
unter dem die Messages von anderen Nodes gefunden werden können. Die
,,richtigen” Namen sind in http://data.bit-bots.de/architektur.png zu
finden. Das zweite Argument ist die Referenz auf das Messageobject,
welches gesendet werden wird.

Der Subscriber ist ähnlich aufgebaut, aber hier kommt die Referenz auf
die Callbackmethonde hinzu. Diese Methode definieren wir später, sie
wird aufgerufen, sobald eine Message reinkommt.

::

           self.bridge = CvBridge()

Nun werden weitere Klassenvariablen oder Objekte initialisiert, in dem
Beispiel brauchen wir nur ein Objekt, welches später helfen wird die
Bilddaten in das richtige Format zu bringen. Wird das Objekt im
Konstruktor erzeugt, muss es nicht bei jeder Iteration neu gemacht
werden, was im Normalfall Rechenzeit spart.

::

       rospy.init_node("bitbots_dummyvision")
       rospy.spin()

Nun können wir den Node initialisieren, ab jetzt weiß der Masternode
Bescheid, dass wir existieren und was wir subscriben und publishen.

Mit *rospy.spin()* halten wir das Programm am Laufen, während wir auf
einen Callback warten.

::

     def work(self, img):
       ra = self.bridge.imgmsg_to_cv2(img, "bgr8")

Hier definieren wir eine neue Funktion, in der die eigentlich Arbeit
passiert. Diese bekommt eine Imagenachricht übergeben. Diese
konvertieren wir in ein OpenCV Objekt.

::

         bimg = cv2.GaussianBlur(ra, (9, 9), 0)
         b, g, r = cv2.split(bimg)
         circles = cv2.HoughCircles(g, cv2.HOUGH_GRADIENT, 1, 100, param1=50, param2=43, minRadius=15, maxRadius=200)

Nun können wir in OpenCV damit arbeiten. Um es der Kreiserkennung
leichter zu machen, lassen wir einen Weichzeichner (gaussian) über das
Bild laufen. Wir separieren den Grünkanal, anschließend wird mittels
einer OpenCV Methode versucht alle Kreise zu finden.

::

         msg = BallsInImage()
         msg.header.frame_id = img.header.frame_id
         msg.header.stamp = img.header.stamp
         if circles is not None:
             circles = np.uint16(np.around(circles))
             for i in circles[0, :]:

Nun fangen wir an die Message zu bauen und iterieren über alle gefundene
Kreise. Wir setzten hier noch schnell in den Header ein, zu welchem Bild
die verarbeiten Daten gehören, damit das später wieder zugeordnet werden
kann. Mit Numpy konvertieren wir dies nun, um besser drauf zugreifen zu
können, sofern Kreise gefunden worden.

::

             can = BallInImage()
             can.center.x = i[0]
             can.center.y = i[1]
             can.diameter = (i[2] * 2) + 3
             can.header.frame_id = img.header.frame_id
             can.header.stamp = img.header.stamp
             msg.candidates.append(can)

Wir definieren für jeden Ball ein neues Message Objekt, welches wieder
Teil der eigentlichen Message wird. Hier tragen wir die gefunden Werte
ein (radius \* 2 um den Durchmesser zu bekommen und noch ein Tick mehr
Rand).

Wir setzten noch die frame_id im Header, damit wir später wissen zu
welchem Bild der Frame gehört.

Fügen wir nun den Ball der ursprünglichen Liste hinzu.

::

         self.pub_balls.publish(msg)

Nun sind wir schon fertig und können die Message abschicken sobald alle
Kandidaten hinzugefügt wurden.

::

     def image_callback(self, img):
       self.work(img)

Abschließend einmal den eigentlichen Callback, der bei uns nichts
anderes macht als die *work* Funktion aufzurufen. Das könnte man sich in
diesem Beispiel auch sparen und direkt *work* aufrufen.

::

   if __name__ == "__main__":
       DummyVision()

Abschließend ein typischer Python-Griff, hier geben wir an welche Klasse
ausgeführt wird beim Aufruf der Datei und verhindern, dass dies bei
einem Import passiert.

**Zackferdich!: Rosnode in 50 Zeilen**

Nun noch als ausführbar markieren mit *chmod +x ballerkennung.py* und im
catkin Workspace mit catkin_make neu bauen und *source devel/setup.bash*
ausführen für Tab-Completion. (Und daran denken, den Ordner, wenn nicht
schon getan, zu verlinken.)

Starten
=======

Startet zuerst den Rosmaster (koodinierender Prozess) mit *roscore* Nun
könnt ihr den Node in einer weiteren Konsole starten. Das geht mit
*rosrun bitbots_ballerkennung ballerkennung.py* (Pyenv ggf. vorher
aktivieren).

Noch passiert da nicht viel (wenn kein Fehler kommt ist schonmal gut).
Noch gehen ja keine Daten hinein, die verarbeitet werden können.

Lasst uns drei weitere Nodes starten. Zunächst *rosrun
bitbots_imageviwer bitbots_imageviewer.py* (venv2). Dieser zeigt eure
Ausgabe an.

Desweitern wird ein Classifier benötigt, der die Kandidaten bewertet,
zum Beispiel *rusrun bitbots_ballclassifier
keras_cnn_classifier.py*\ (venv2).

Jetzt brauchen wir noch eine Bildquelle. Entweder ihr startet den
Kamera-Node und hohlt euch Bilder von der Webcam, oder ihr ladet
Testbilder herunter http://data.bit-bots.de/simples_dataset.zip und
startet mit dem entsprechendem Pfad den Imageloader *rosrun
bitbots_imageloader bitbots_imageloader.py PATH/TO/IMAGES*. (Jeweils im
pyenv2)

Wenn alles geklappt hat solltet ihr nun ein Bild sehen mit eurer
Ballerkennung.

Glückwunsch, du hast deinen ersten Rosnode zum Laufen bekommen, du
kannst nun den Code beliebig ändern und den einen Node neustarten, oder
dich an andere setzen.

Später wirst du nicht mehr alle Nodes einzeln starten sonder mittels
Launch-Skripten. In diesen kannst du Parameter definieren und angeben,
welche Nodes gestartet werden sollen. Auch der Rosmaster wird
mitgestartet, es kann sich aber lohnen beim Testen den Roscore manuell
zu starten. So bleiben die Debuggingtools aktiv, auch wenn ihr die
restliche Software neu startet.

Advanced
========

Die Basics sind jetzt hoffentlich klar, dennoch gib es ein paar Punkte,
die einem unter ROS das Arbeiten deutlich leichter machen.

Launchfiles
-----------

Während man theoretisch jeden Knoten einzeln starten kann, ist das im
Normalfall nicht sonderlich sinnvoll, deswegen gibt es Launchfiles.
Hierbei handelt es sich um XML-Dokumente die beschreiben, welche Knoten
gestartet werden sollen. Außerdem können hier gleich die zugehörigen
Parameterfiles eingelesen werden oder eingestellt werden, was passiert,
wenn ein Knoten abstürzt.

::

   <launch>
     <include file="$(find bitbots_vision_common)/launch/vision_processing.launch" />
     <remap from ="usb_cam/image_raw" to="image_raw"/>
     <node name="usb_cam" pkg="usb_cam" type="usb_cam_node" args="" />
   </launch>

In diesem Beispiel wird zunächst ein anderes Launchfile eingebunden, aus
dem Paket *vision_common*, ein Topic umbenannt und ein Node namens
*usb_cam_node* aus dem Package *usb_cam* gestartet.

Starten der Software
--------------------

Wenn alles richtig eingerichtet ist, solltet ihr mit dem Befehl
*roslaunch bitbots_common start_simulator.launch* den Simulator mit der
Software starten können.

Mit dem Tool *./robot_compile.sh* könnt ihr die Software auf einen
Roboter spielen und dort mit *roslaunch bitbots_common
start_robocup_teamplayer.launch* starten.

Rosbag
------

Mit dem Tool *rosbag* kann man Topics aufnehmen und wider abspielen. So
kann man zum Beispiel sehr bequem Testbilder und die Motorpositionen
aufnehmen um so später in Ruhe die Bildverarbeitung zu testen oder unter
gleichen Bedingungen zu vergleichen.

Debugging
---------

Mit *rostopic* könnt ihr euch in der Konsole Infos zu einzelnen Topics
ausgeben lassen. Mit rqt(daten) und rviz(3d) stehen euch grafische
Debuggingtools zur Verfügung, außerdem gibt es viele weitere Tools,
probiert gerne etwas rum oder schaut euch existierende Nodes an. Gerade
rqt ermöglicht es euch den Datenfluss oder Ausgabe von Nodes zu
betrachten. Oder auch selber Daten einzuspeisen.

Weiter geht’s
-------------

Es gibt noch viele weitere Möglichkeiten mit ROS zu arbeiten, schau dir
da am besten die offiziellen ROS Tutorials an, da lernst du alles über
actions, services und viele andere Dinge:
http://wiki.ros.org/ROS/Tutorials

Command Guide
=============

+-----------------------+-----------------------+-----------------------+
| **Beschreibung**      | **Command**           | **Anmerkung**         |
+=======================+=======================+=======================+
| Git-Submodules        | *git submodule update | oder pullall          |
| synchronisieren       | –init*                |                       |
+-----------------------+-----------------------+-----------------------+
| Alle Gits pullen      | *git submodule        | oder per IDE          |
|                       | foreach git pull*     |                       |
+-----------------------+-----------------------+-----------------------+
| Initiales Klonen      | *./pull_all.sh*       |                       |
| aller Repositories    |                       |                       |
+-----------------------+-----------------------+-----------------------+
| Erstellen des VENVs   | *./create_venv.bash*  |                       |
+-----------------------+-----------------------+-----------------------+
| Aktivieren eines      | *.                    | ,,sourcen”            |
| VENVs                 | venv3/bin/activate*   |                       |
+-----------------------+-----------------------+-----------------------+
| Anlegen eines         | *mkdir -p  /ws/src*   |                       |
| Workspaces            |                       |                       |
+-----------------------+-----------------------+-----------------------+
| Globales Installieren | *pip3 install -r      |                       |
| der                   | requierements.txt*    |                       |
| py3-Abhängigkeiten    |                       |                       |
+-----------------------+-----------------------+-----------------------+
| Globales Installieren | *pip2 install -r      |                       |
| der                   | requierements.txt*    |                       |
| py2-Abhängigkeiten    |                       |                       |
+-----------------------+-----------------------+-----------------------+
| Kaputte               | *./repair.sh*         |                       |
| Abhängigkeiten fixen  |                       |                       |
+-----------------------+-----------------------+-----------------------+
| Bauen der Software    | *catkin_make*         | in ws                 |
+-----------------------+-----------------------+-----------------------+
| ROS Bindings sourcen  | *. devel/setup.sh*    | in ws                 |
+-----------------------+-----------------------+-----------------------+
| Starten der           | *rqt*                 |                       |
| DebugUI/rqt           |                       |                       |
+-----------------------+-----------------------+-----------------------+
| Publischen von        | *rostopic pub ...*    |                       |
| Messages              |                       |                       |
+-----------------------+-----------------------+-----------------------+

Wichtige Launchskripts
======================

+-----------------------------------+-----------------------------------+
| **Desc.**                         | **Command**                       |
+===================================+===================================+
| Simulator                         | *roslaunch bitbots_common         |
|                                   | start_simulator.launch*           |
+-----------------------------------+-----------------------------------+
| High-level                        | *roslaunch bitbots_common         |
|                                   | start_robocup_teamplayer.launch   |
|                                   | hcm:=false duty:=TeamPlayer*      |
+-----------------------------------+-----------------------------------+
| kompl. Stack                      | *roslaunch bitbots_common         |
|                                   | start_robocup_teamplayer.launch   |
|                                   | duty:=TeamPlayer*                 |
+-----------------------------------+-----------------------------------+

Checklisten
===========

Einrichtung
-----------

-  ROS installiert

-  ROS-Workspace angelegt

-  Git geclont und im workspace/src Ordner (vorhanden/verlinkt)

-  Subrepos auch geklont

-  Virtualenv installiert oder alles lokal

Testen
------

-  Alles eingerichtet (siehe vorherige Liste)

-  Software bauen (im Workspace) (catkin_make)

-  Repair Skript ausgeführt

-  Workspaces(Global und eigener Workspace) gesourced (je Konsole) (wenn
   nicht in .bashrc eingetragen)

-  Virtualenv aktiviert (je Konsole) (wenn nicht alle Abhängigkeiten
   global)

Entwickeln
----------

-  Architektur Überblick: http://data.bit-bots.de/architektur.png

Roboter
-------

-  Verbinden zum Roboter

-  *robot_compile.sh* zum ,,Flashen” der Software auf den Roboter
