Bitbots Motion
==============

Was ist die Motion?
-------------------

Die Motion bezeichnet die Steuerung der Bewegungen des Roboters. Dabei werden verschiedene
Komponenten der Soft- und Hardware verwendet, die in diesem Artikel beschrieben werden. Außerdem
werden gängige Probleme und zugehörige Lösungsstrategien erklärt.

Das Verhalten der Motion wird in erster Linie durch den sogenannten Control Loop beschrieben. Dabei
handelt es sich um einen Zyklus aus Lesen der aktuellen Motorpositionen und Schreiben neuer
Positionen. Parallel zu diesem Zyklus findet das Berechnen der neuen Positionen, reagieren auf die
gemessenen Werte und so weiter statt.

Damit möglichst schnell auf Probleme in den Motoren, zum Beispiel Overload Errors, reagiert werden
kann, ist es besser, wenn der Control Loop möglichst schnell passiert. Früher hatten wir
beispielsweise eine Frequenz von 100Hz für einen Durchlauf des Zyklus'. Dadurch dauert es mindestens
20ms, bis auf einen Fehler reagiert werden kann, da die Motorpositionen gelesen werden müssen, dann
parallel zu dem Schreibeprozess „überlegt” wird, bis beim nächsten Schreibeprozess die Werte
geschrieben werden können.

Um den Control Loop zu beschleunigen, gibt es im Wesentlichen drei Möglichkeiten:

1. Die Bits schneller über den Bus senden. Dies ist aber durch die Baudrate von höchstens vier
   Megabaud limitiert (Robotis behauptet 4.5 MBaud wären möglich, stimmt aber nicht).
2. Die Daten komprimieren, zum Beispiel statt jeden Motor einzeln zu lesen mit einem Befehl alle
   Motoren zu lesen (s. u. sync read und sync write)
3. Mehr Busse benutzen. In unserem Fall bietet es sich an, einen Bus pro Extremität zu verwenden.

Wie ist sie aufgebaut?
----------------------

Motoren und Busse
~~~~~~~~~~~~~~~~~

Auf der untersten Ebene der Motion befinden sich die Motoren. Bei ihnen handelt es sich um die
Dynamixel-Motoren MX-106 und MX-64 von Robotis. Über den Motorbus sind die Motoren untereinander
und mit dem DXL-Board verbunden. Als Bussystem nutzen unsere Motoren RS-485 (oder TTL).

Die Kabel für RS-485 bestehen aus vier Adern. Dabei sind zwei der Adern für Ground und für Strom
(14.8-16.8V, je nach momentaner Akkuspannung), eins für die Daten (5V) und eins, auf dem die Daten 
invertiert übertragen werden. Es ist besonders wichtig, beim Krimpen der Kabel und beim Anschließen
an einen Logic Analyzer darauf zu achten, dass die Kabel für Strom und Daten nicht vertauscht
werden, da die Motoren mehr als 5V auf der Datenleitung nicht vertragen!

Die Kabel für TTL haben nur drei Adern, bei denen zwei für Ground und Strom und eins für Daten
(auch 5V) ist. Auch hier sollten Kabel nicht falsch gekrimpt werden.

Die Kommunikation mit den Motoren findet über das Bus-Protokoll von Dynamixel statt. Die Details
können direkt in der Spezifikation nachgelesen werden. Wir benutzen die `Version 2
<http://emanual.robotis.com/docs/en/dxl/protocol1/>`_, aber auch die `Version 1
<http://emanual.robotis.com/docs/en/dxl/protocol2/>`_ kann als Referenz
hilfreich sein.

Eine Nachricht nach dem Protokoll besteht im Wesentlichen aus Header, Ziel-Motor-ID, Instruktion,
einer Parameterliste und einer Checksumme. Die wichtigsten Instruktionen sind Ping, Read, Write und
Status sowie Sync Read und Sync Write zum Lesen oder Schreiben mehrerer Motoren zugleich.

Jeder Motor hat dazu eine feste ID, über die er eindeutig angesprochen werden kann. Bei einem neuen / 
neu eingerichteten motor ist die ID 1. Sie kann mit einer write-Instruktion in ein bestimmtes
Register gesetzt werden.

Es ist wichtig, dass nicht zwei Motoren mit der selben ID auf einem Bus angeschlossen sind. Dies führt 
zu Kommunikationsfehlern.

Beim Sync Read antworten die Motoren in der Reihenfolge, in der sie im Sync Read angefragt werden.
Dabei ist zu beachten, dass wenn ein Motor nicht antwortet, auch alle auf ihn folgenden Motoren
nicht antworten werden, da die Motoren auf die Antwort des vor ihnen angefragten Motors warten.

DXL-Board
~~~~~~~~~

Das DXL-Board ist per USB an den NUC und über den Motorbus an die Motoren angeschlossen. Es kümmert
sich also um die Kommunikation zwischen dem NUC und den Motoren, indem es die Nachrichten vom Bus
einliest und über USB an den NUC weiterleitet und umgekehrt. Auf dem DXL-Board befindet sich auch
die IMU, deren Werte wie die eines Motors an den NUC übergeben werden.

Kernel
~~~~~~

Ab hier befinden sich alle Ebenen der Motion auf dem NUC.

Im Kontext der Motion kümmert sich der Linux Kernel darum, dass das über USB angeschlossene
DXL-Board von Programmen verwendet werden kann. Es ist darauf zu achten, dass die Latenz im Kernel
von 16ms auf 0 gesetzt wird, z.B. per:

.. code:: bash

   sudo sh -c "echo '0' > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer"

Dies ist jedoch nur für "echte" serielle Devices notwendig. Das DXL-Board meldet sich zurzeit 
als ACM an, dort ist es nicht nötig. Bei dem neuen QUADDXL wird dies jedoch extrem wichtig sein.

Dynamixel SDK
~~~~~~~~~~~~~

Das Dynamixel SDK implementiert das Dynamixel Protokoll. Es stellt dabei (in verschiedenen Sprachen) 
Methoden zum Versenden von Instruktionen und zum Lesen von Statuspaketen bereit.  Wir benutzen eine
geforkte version, da Robotis vergessen hat, das Sync Read für mehr als ein Register zu
implementieren.

Dynamixel Workbench
~~~~~~~~~~~~~~~~~~~

Die Dynamixel Workbench bietet höhere (abstraktere) Funktionen als das Dynamixel SDK. Beispielsweise
wird die Motorposition im SDK und auf den Motoren als Wert von 0 bis 4096 angegeben (2 Byte) und von 
der Dynamixel Workbench zu Radian umgerechnet. Die Workbench erleichtert so das Arbeiten mit den
Servos auf einer abstrakteren Ebene.

ROS Control Framework
~~~~~~~~~~~~~~~~~~~~~

Das ROS Control Framework ist ein Bestandteil von ROS, der für die Kontrolle von Motoren und
Sensoren verwendet wird. Für ROS Control gibt es Controller, die die Schnittstelle zwischen ROS und
hardwarenäheren Softwareteilen bilden. Die Controller sind hardwareagnostisch, da sie auf Hardware
Interfaces arbeiten. Durch diese wird von der Hardware (hier den Motoren) abstrahiert.
Für die Motoren gibt es den Dynamixel Controller, der etwa das Setzen der Kraft für die Motoren
ermöglicht. Das darunter liegende Hardware Interface ist unser Dynamixel Hardware Interface.

ROS messages
~~~~~~~~~~~~

Nach all diesen Schritten kommt schließlich die Ebene der ROS-Nachrichten. Dabei handelt es sich zum
einen um die Joint States, die die momentanen Positionen der Motoren wiedergeben und zum anderen um die
Joint Goals, über die die gewünschten Positionen der Motoren angegeben werden können.

Auch die Daten von der IMU werden über das Hardware Interface an einen IMU Controller weitergereicht.
Zudem werden auch die Fußsensoren vom Hardware Interface ausgelesen.

Wie verwendet man bitbots_ros_control?
--------------------------------------

Das Paket bitbots_ros_control stellt das Hardware Interface für die Dynamixel-Motoren bereit.

Die wichtigste Konfigurationsdatei ist dafür die wolfgang.yaml-Datei. In ihr gibt es diverse
Einstellungen, um festzulegen, welche Werte aus den Motoren ausgelesen werden sollen (Temperatur,
Geschwindigkeit, Kraft, ...), welche Sensoren verwendet werden sollen (Fußdrucksensoren, IMU) und um
Einstellungen festzulegen (Control-Loop-Frequenz, Baudrate, Port des DXL-Boards, Auto-Torque, ...).

Der ROS-Node dazu kann mit `roslaunch bitbots_ros_control ros_control.launch` gestartet werden.
Dabei werden die folgenden Operationen durchgeführt:

1. Die Motoren werden in alphabetischer Reihenfolge angepingt. Die alphabetische Reihenfolge liegt
   dabei an der Art, wie die YAML-Datei eingelesen wird. Es wird also zuerst der HeadPan (Motor 19)
   und zuletzt der RShoulderRoll (Motor 3). 
2. Dann werden die Werte aus der Konfigurationsdatei in den RAM und ROM der Motoren geschrieben,
   etwa Werte wie die maximale Geschwindigkeit oder die Verzögerung beim Antworten.
3. Anschließend erscheint die Ausgabe „Hardware interface init finished“.
4. Jetzt beginnt der Control Loop mit abwechselndem Sync Read und Sync Write.
5. Schließlich werden die Controller für ROS Control geladen.


Was tun bei Problemen?
----------------------

Error Opening Serial Port
~~~~~~~~~~~~~~~~~~~~~~~~~

Sollte der Fehler „Error opening serial port” auftreten, kann keine Verbindung vom NUC zum DXL-Board
hergestellt werden. Zunächst sollte daher überprüft werden, ob überhaupt der USB-Stecker im NUC
steckt. Anschließend kann über `lsusb` festgestellt werden, ob das Board gefunden wird (Eintrag
„leaf”). Dann kann über `ls /dev/` die Liste der angemeldeten Geräte angezeigt werden. Dort sollte
ein Gerät wie „/dev/ttyACM0” auftauchen. Ist der Name anders, muss er in der
wolfgang.yaml-Konfigurationsdatei angepasst werden oder der Stecker kurz gezogen und wieder in den
NUC gesteckt werden, damit sich das Board unter dem bekannten Namen anmeldet.

Motorprobleme
~~~~~~~~~~~~~

Das erste, was im Falle eines Problems (ausfallende Motoren, „no status from id ...”) geprüft werden
sollte, ist ob die Kabel richtig in den Motoren stecken. Manchmal rutschen Kabel, die nur lose im
Stecker sitzen, beim Bewegen raus. Um die Erreichbarkeit der Motoren zu prüfen, gibt es eine linux software
von Robotis http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/.

Falls so alle Motoren erreichbar sind, sollte überprüft werden, ob sich im DXL-Board eine IMU
befindet. Aufgrund eines Softwarefehlers sind in diesem Fall keine Motoren erreichbar.

Als nächstes kann ein Logic Analyzer benutzt werden, um Fehler auf dem Bus zu finden. Beim Logic
Analyzer handelt es sich um einen kleinen schwarzen Kasten, aus dem viele bunte Kabel schauen 
(siehe `hier <https://eur.saleae.com/products/saleae-logic-pro-16?variant=10963959873579)>`_. Mit
ihm können die Daten vom Bus (bzw. sogar von maximal 16 Bussen zugleich) ausgelesen werden. Dazu
muss das Ground Kabel an den Ground des Busses und eines der anderen Kabel an Data+ angeschlossen
werden. Dabei ist es sehr wichtig, dass diese Kabel nicht vertauscht werden, da sonst ein Schaden an
den Motoren entstehen wird.

Nun kann die Software Saleae Logic benutzt werden, um die Daten auszulesen. Dafür muss auf der
Schaltfläche neben dem Start-Button zunächst 15MB/s und eine Spannung von 5V eingestellt werden.
Dann kann die Aufnahme gestartet werden und das problematische Programm ausgeführt werden, also
beispielsweise ein fehlgeschlagener Ping oder das Starten von bitbots_ros_control. Nach dem Beenden
der Aufnahme kann man sich mit dem Async Serial Analyzer direkt die Bytes der Nachricht anschauen
(dazu sollte die oben verlinkte Referenz auf das Protokoll zurate gezogen werden), oder den
Dynamixel Analyzer nutzen, der die Pakete direkt interpretiert. Der Dynamixel Analyser muss jedoch
zusätlich als Plugin installiert werden (`https://github.com/r3n33/SaleaeDynamixelAnalyzer
<https://github.com/r3n33/SaleaeDynamixelAnalyzer>`_).

Sollte auf diese Weise immer noch kein Fehler festgestellt worden sein, könnte der Fehler im
DXL-Board liegen. Um diesen Fehler festzustellen, gibt es verschiedene Möglichkeiten:

* Man testet die Software auf einem anderen Roboter, da er ein anderes DXL-Board eingebaut hat, aber
  alles weitere an höherer Software identisch ist
* Man schließt ein Ersatz-Board zwischen Motorbus und Nuc an
* Man verwendet Wireshark auf dem Interface, an dem das DXL-Board an den NUC angeschlossen ist
  (vermutlich /dev/ttyACM0), um sicherzustellen, das das DXL-Board die Kommunikation auf dem Bus
  unverändert an den NUC weiterleitet

Falls immer noch kein Fehler festgestellt werden konnte, müssen die höheren Softwareteile des
Software-Stacks verantwortlich sein. Es sollte überprüft werden, ob Updates für DynamixelSDK oder
Dynamixel Workbench verfügbar sind. Sollte das Problem dadurch nicht zu lösen sein, müssen diese
Softwareteile manuell gedebugt werden.
