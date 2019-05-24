=============
Launchskripte
=============

Um etwas über die Benutzung von Launchfiles zu erfahren siehe :doc:`manual/tutorials/software_overview`.
Im Folgenden sind die Übergreifenden Launchfiles und danach die Launchfiles der einzelnen Packete aufgelistet, sowie die Parameter die angegeben werden können. Dabei werden die Defaultwerte verwendet, außer man gibt mittel parameter:=true/false an, auf was man den Parameter setzen möchte.
Allgemein gilt für alle Launchfiles, dass sie das Load_robot_description Launchfile mit starten, wenn der Dateiname xy_standalone.launch lauetet. Das load_robot_description Launchfile lädt das URDF und sorgt dafür, dass der TF Tree gebaut wird. Startet man mehrere Launchfiles und will man besagtes Launchfile benutzen, darf nur eines der Launchfiles als standalone gestartete werden.

Übergreifende Launchskripte
===========================
Diese Launchskripte sind zu finden in dem Package bitbots_bringup.

Teamplayer (Alles)
__________________
Will man alle relevanten Softwarepakete, zum Beispiel für ein Spiel, starten, launched man den Teamplayer. Hierfür muss der Motorstrom am Roboter angeschaltet werden. Nach dem Start fährt der Roboter die Walkready Position an. Folgende Parameter können dabei gesetzt werden:

+------------------+------------+-------------------------------------------------------------------------------------+
|Parameter         |Defaultwert |  Erklärung                                                                          |
+==================+============+=====================================================================================+
|sim               | false      | Ausführung in Simulation                                                            |
+------------------+------------+-------------------------------------------------------------------------------------+
|motion            | true       | Starten der Motion                                                                  |
+------------------+------------+-------------------------------------------------------------------------------------+
|behave            | true       | Starten des Behaviors                                                               |
+------------------+------------+-------------------------------------------------------------------------------------+
|vision            | false      | Starten der Vision                                                                  |
+------------------+------------+-------------------------------------------------------------------------------------+
|team_comm         | false      | Starten der Team Communication                                                      |
+------------------+------------+-------------------------------------------------------------------------------------+
|localization      | false      | Starten der Lokalisation                                                            |
+------------------+------------+-------------------------------------------------------------------------------------+
|simple            | false      | Benutzen des Simple Behaviors                                                       |
+------------------+------------+-------------------------------------------------------------------------------------+
|use_game_settings | true       | Game Settings an alle Softwareknoten durchreichen                                   |
+------------------+------------+-------------------------------------------------------------------------------------+

Highlevel
_________
Dieses Launchskript startet alle spielrelevanten Komponenten außer der Motion.
Hierfür muss der Motorstrom am Roboter angeschaltet werden. Nach dem Start fährt der Roboter die Walkready Position an.

+------------------+------------+-------------------------------------------------------------------------------------+
|Parameter         |Defaultwert |  Erklärung                                                                          |
+==================+============+=====================================================================================+
|sim               | false      | Ausführung in Simulation                                                            |
+------------------+------------+-------------------------------------------------------------------------------------+
|motion            | true       | Starten der Motion                                                                  |
+------------------+------------+-------------------------------------------------------------------------------------+
|behave            | true       | Starten des Behaviors                                                               |
+------------------+------------+-------------------------------------------------------------------------------------+
|vision            | false      | Starten der Vision                                                                  |
+------------------+------------+-------------------------------------------------------------------------------------+
|team_comm         | false      | Starten der Team Communication                                                      |
+------------------+------------+-------------------------------------------------------------------------------------+
|localization      | false      | Starten der Lokalisation                                                            |
+------------------+------------+-------------------------------------------------------------------------------------+
|simple            | false      | Benutzen des Simple Behaviors                                                       |
+------------------+------------+-------------------------------------------------------------------------------------+
|use_game_settings | true       | Game Settings an alle Softwareknoten durchreichen                                   |
+------------------+------------+-------------------------------------------------------------------------------------+

Motion
______
Diese Launchskript startet alle relevanten Softwarekomponenten um die Motion des Roboters zu gewährleisten.
Ist diese Launchskript gestartet können die Motoren angesteuert werden und es können Bewegungen auf dem Roboter ausgeführt werden, wie Walking oder Animationen.
Hierfür muss der Motorstrom am Roboter angeschaltet werden. Nach dem Start fährt der Roboter die Walkready Position an.

+------------------+------------+-------------------------------------------------------------------------------------+
|Parameter         |Defaultwert |  Erklärung                                                                          |
+==================+============+=====================================================================================+
|sim               | false      | Ausführung in Simulation                                                            |
+------------------+------------+-------------------------------------------------------------------------------------+
|use_game_settings | true       | Game Settings an alle Softwareknoten durchreichen                                   |
+------------------+------------+-------------------------------------------------------------------------------------+


Simulator
_________
Diese Skript startet den Simulator.

+------------------+------------+-------------------------------------------------------------------------------------+
|Parameter         |Defaultwert |  Erklärung                                                                          |
+==================+============+=====================================================================================+
|hcm               | false      | Starten der HCM                                                                     |
+------------------+------------+-------------------------------------------------------------------------------------+
|use_fake_walk     | false      | Nutzen eines Schwebens (Fake Walk) statt des normalen Walking                       |
+------------------+------------+-------------------------------------------------------------------------------------+


Launchskripte einzelner Packete
===============================
Behavior
________
- Body Behavior

Das Body Behavior steuert das Verhalten des Roboters. Dabei geht es grob gesagt um die Taktik die verwendet wird.

- behavior.launch

Startet das Body und das Headbehavior

+------------------+------------+-------------------------------------------------------------------------------------+
|Parameter         |Defaultwert |  Erklärung                                                                          |
+==================+============+=====================================================================================+
|wolfgang          | false      | Ob das Behavior für die Wolfgang Plattform gestartet werden soll                    |
+------------------+------------+-------------------------------------------------------------------------------------+
|use_game_settings | false      | siehe Teamplayer Launchskript                                                       |
+------------------+------------+-------------------------------------------------------------------------------------+
|simple            | false      | Soll das simple Behavior gestartet werden -> nur zum Ball rennen und kicken         |
+------------------+------------+-------------------------------------------------------------------------------------+

-behavior_standalone.launch

+------------------+------------+-------------------------------------------------------------------------------------+
|Parameter         |Defaultwert |  Erklärung                                                                          |
+==================+============+=====================================================================================+
|wolfgang          | true       | Ob das Behavior für die Wolfgang Plattform gestartet werden soll                    |
+------------------+------------+-------------------------------------------------------------------------------------+
|duty              | TeamPlayer | ob Feldspieler Behavior oder Goalie oder Penalty Behavior gestartet werden soll     |
+------------------+------------+-------------------------------------------------------------------------------------+
|simple            | false      | Soll das simple Behavior gestartet werden -> nur zum Ball rennen und kicken         |
+------------------+------------+-------------------------------------------------------------------------------------+

- body_behavior.launch

Startet nur das Body behavior und nicht das Head Behavior

+------------------+------------+-------------------------------------------------------------------------------------+
|Parameter         |Defaultwert |  Erklärung                                                                          |
+------------------+------------+-------------------------------------------------------------------------------------+
|duty              | TeamPlayer | ob Feldspieler Behavior oder Goalie oder Penalty Behavior gestartet werden soll     |
+------------------+------------+-------------------------------------------------------------------------------------+
|use_game_settings | false      | siehe Teamplayer Launchskript                                                       |
+------------------+------------+-------------------------------------------------------------------------------------+

- simple_behavior.launch

Startet das simple Behavior. Nur zum Ball laufen und dann den Ball kicken.

- Head Behavior

- head_behavior.launch

Startet das Head Behavior

+------------------+------------+-------------------------------------------------------------------------------------+
|Parameter         |Defaultwert |  Erklärung                                                                          |
+------------------+------------+-------------------------------------------------------------------------------------+
|depends_only      | false      | startet nur die dependency, für debugging Zwecke in Pycharm (deprecated)            |
+------------------+------------+-------------------------------------------------------------------------------------+
|use_game_settings | false      | siehe Teamplayer Launchskript                                                       |
+------------------+------------+-------------------------------------------------------------------------------------+

head_behavior_standalone.launch

+------------------+------------+-------------------------------------------------------------------------------------+
|Parameter         |Defaultwert |  Erklärung                                                                          |
+------------------+------------+-------------------------------------------------------------------------------------+
|depends_only      | false      | startet nur die dependency, für debugging Zwecke in Pycharm (deprecated)            |
+------------------+------------+-------------------------------------------------------------------------------------+
|wolfgang          | true       | Definiert ob das Head Behavior für die Wolfgang Plattform gestartet werden soll     |
+------------------+------------+-------------------------------------------------------------------------------------+

Lowlevel
________
- Buttons
- Ros Control

Misc
____
- Teleop
- System Monitor
- Bringup

Die übergreifenden Launchskripte aus diesem Kapitel sind bereits oben erklärt. Hier werden noch die restlichen Launchfiles beschrieben.

Motion
_______
- Animation Server
- HCM
- Walking

Navigation
__________
- Localization
- Pathfinding

Vision
______
- Imageloader
- Vision

Base Footprint
______________

Humanoid League Misc
____________________
- Game Controller
- Speaker
- Team Communication
- Transformer

Humanoid League Visualisation
_____________________________
- Interactive Marker

UDP Bridge
__________

Wolfgang robot
______________
- Wolfgang description
- Wolfgang Moveit

Wolves Imageprovider
____________________




