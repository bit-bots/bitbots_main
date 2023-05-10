=============
Launchskripte
=============

Im Folgenden sind die wichtigsten Launchfiles aufgelistet.
Die Parameter der Launchfiles kann man sich mit ```ros2 launch <package-name> <launch-file-name>.launch -s``` im Terminal anzeigen lassen.
Diese haben Defaultwerte, welche sich mittels ```<parameter>:=<wert>``` überschreiben lassen.

Launchskripte in bitbots_bringup
================================
Diese Launchskripte sind zu finden im bitbots_bringup Packet.

teamplayer.launch
_________________
Wenn man den Roboter für ein Spiel starten möchte wird dieses Launchfile verwendet. Es werden alle relevanten Komponenten gestartet.
Hierfür muss der Motorstrom am Roboter angeschaltet werden. Nach dem Start fährt der Roboter die Walkready-Position an. 


highlevel.launch
________________
Dieses Launchskript startet alle spielrelevanten Komponenten außer der Motion.
Hierfür muss der Motorstrom am Roboter angeschaltet werden. Nach dem Start fährt der Roboter die Walkready-Position an.


motion_standalone.launch
________________________
Diese Launchskript startet die Motion und alle für die Motion relevanten Komponenten.
Ist dieses Launchskript gestartet, können die Motoren angesteuert und Bewegungen auf dem Roboter ausgeführt werden, wie Walking oder Animationen.
Hierfür muss der Motorstrom am Roboter angeschaltet werden. Nach dem Start fährt der Roboter die Walkready-Position an.

vision_standlone.launch
________________________
Diese Launchskript startet die Vision und alle für die Vision relevanten Komponenten. 

visualization.launch

simulator_teamplayer.launch
___________________________
Diese Skript startet den Simulator und den Softwarestack des Roboters.

visualization.launch
____________________
Dieses Skript startet RViz und visualisiert die Sensordaten des Roboters.

