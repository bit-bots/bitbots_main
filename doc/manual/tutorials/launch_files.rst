=============
Launchskripte
=============

Um etwas über die Benutzung von Launchfiles zu erfahren siehe :doc:`manual/tutorials/software_overview`.
Im Folgenden sind die Übergreifenden Launchfiles und danach die Launchfiles der einzelnen Packete aufgelistet, sowie die Parameter die angegeben werden können.
Allgemein gilt für alle Launchfiles, dass sie das Load_robot_description Launchfile mit starten, wenn der Dateiname xy_standalone.launch lauetet. Das load_robot_description Launchfile lädt das URDF und sorgt dafür, dass der TF Tree gebaut wird. Startet man mehrere Launchfiles und will man besagtes Launchfile benutzen, darf nur eines der Launchfiles als standalone gestartete werden.

Übergreifende Launchskripte
===========================
Diese Launchskripte sind zu finden in dem Package bitbots_bringup.

Alles
_________________________________________
Will man alle relevanten Softwarepackete, zum Beispiel für ein Spiel, starten, launched man den Teamplayer. Folgende Parameter können dabei gesetzt werden:

+------------------+------------+
|Parameter         |  Erklärung |
+==================+============+
|sim               | B          |
+------------------+------------+
|motion            | D          |
+------------------+------------+
|behave            | D          |
+------------------+------------+
|vision            | D          |
+------------------+------------+
|team_comm         | D          |
+------------------+------------+
|localization      | D          |
+------------------+------------+
|simple            | D          |
+------------------+------------+
|use_game_settings | D          |
+------------------+------------+





Launchskripte einzelner Packete
===============================
Behavior
________
- Body Behavior
- Head Behavior

Lowlevel
________
- Buttons
- Ros Control

Misc
____
- Teleop
- System Monitor

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




