Die HCM
=======

Die HCM ist der Teil unseres Software-Stacks, der über der :doc:`motion` steht und von dieser
gleichzeitig abstrahiert und einen Mutex darauf bildet.

Motivation
----------

Der Roboter soll dazu fähig sein, verschiedenartige Bewegungen durchzuführen, zum Beispiel Laufen,
Schießen oder dem Hinfallen entgegenwirken. Wenn alle diese Bewegungen gleichzeitig an die Motoren
gesendet werden, führt das zu widersprüchlichen Signalen und einem Zittern der Motoren.

Zur Lösung gibt es zwei verschiedene Ansätze:

1. Der gesamte Code ist in einer großen, monolithischen Blase organisiert, die einen einzigen
   Publisher für Motorpositionen hat und eventuelle Konflikte intern zum Beispiel durch eine State
   Machine verhindert. Dieser Ansatz ist zwar verbreitet, aber sehr schlecht wartbar.
2. Ein Knoten kümmert sich darum zu regeln, wer gerade Motorpositionen schreiben darf,
   beziehungsweise ist selbst der einzige Knoten, der direkt die Motorpositionen schreibt, und alle
   anderen Knoten senden an ihn. Dieser Knoten heißt bei uns Hardware Control Manager (HCM).

Aufgaben der HCM
----------------

In unserer Software gibt es Motorziele, die vom Walking, dem Kopfverhalten oder dem Animation
Server stammen. Diese publishen ihre Ziele jeweils auf eigene Topics. Die HCM entscheidet abhängig
vom aktuellen Status des Roboters (ist er hingefallen? läuft das Spiel gerade?), wer schreiben darf
und leitet die entsprechenden Nachrichten weiter.

Diese Entscheidung wird mithilfe eines DSD getroffen, wobei der genaue Entscheidungsbaum in dessen
DSD-Definition nachgeschlagen werden kann. Im Prinzip werden die einzelnen Komponenten schrittweise
überprüft: Ist eine IMU vorhanden? Besteht eine Verbindung zu den Motoren? Fällt der Roboter? Wird
der Roboter hochgehoben? ...

Die HCM bestimmt so den Status des Roboters, der auch als ROS-Message gepublisht wird
(`/robot_state`). Die Möglichkeiten für den Wert der Nachricht sind beispielsweise Aufstehen,
Hinfallen, Laufen, Hardwareprobleme, Hochgehoben und viele mehr. Die übrigen Werte können
beispielsweise mit einem `rosmsg show humanoid_league_msgs/RobotControlState` abgerufen werden.

Der Status des Roboters kann durch das Publishen als Nachricht auch in anderen Nodes verwendet
werden, zum Beispiel im Verhalten oder in der Lokalisierung. Dies ist zum Beispiel sinnvoll, da
nachdem der Roboter hochgenommen wurde, die Lokalisierung neu initialisiert werden sollte. Auch für
die Teamkameraden ist die Information nützlich, da ein gefallenen Roboter beispielsweise nicht
angespielt werden sollte.

Vorteile der HCM
----------------

Die HCM ermöglicht eine Abstraktion im Verhalten von der Humanoidität des Roboters. Denn da sich
die HCM darum kümmert, dass der gefallene Roboter wieder aufsteht, wirkt der Roboter aus anderen
Teilen der Software wie ein fahrender Roboter und es kann darauf abgestimmte Software wie AMCL oder
MoveBase genutzt werden und das Verhalten kann leichter ausgetauscht werden.

Wie wird die HCM gestartet?
---------------------------

Bevor die HCM gestartet wird, sollte ROS Control (`roslaunch bitbots_ros_control
ros_control_standalone.launch`) und der Animation Server (`roslaunch bitbots_animation_server
animation.launch`) gestartet werden. Die HCM kann dann mit `roslaunch bitbots_hcm hcm.launch`
gestartet werden. Alternativ kann auch das Motion-Launchscript (`roslaunch bitbots_bringup
motion_standalone.launch`) gestartet werden, das zusätzlich zu den drei genannten Nodes auch das
Walking, die Buttons und den Speaker startet. Zum Debuggen sollten die Dateien aber lieber einzeln
gestartet werden, um die Ausgabe zuordnen und die Nodes einzeln neustarten zu können.

Was tun, wenn der Roboter nicht tut, was er soll?
-------------------------------------------------

1. Läuft `ros_control`? Sendet es Joint States? Gibt es keine Fehlermeldungen? Dann liegt es nicht
   an der Hardware.
2. Was ist der Robot State (`rostopic echo /robot_state`)? Da kommt eine Zahl heraus, die man über
   `rosmsg show humanoid_league_msgs/RobotControlState` nachschlagen kann.
3. Die Visualisierung für den DSD der HCM gibt Auskunft darüber, in welcher Entscheidung die HCM
   momentan hängt und wie sie darin gelandet ist. Im Code einer Decision, die eine falsche
   Entscheidung getroffen hat, kann dann nachgeschaut werden, woran das liegt. Der Code einer
   Entscheidung ist üblicherweise sehr kompakt und verständlich.

