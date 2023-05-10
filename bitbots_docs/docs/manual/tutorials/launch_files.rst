=============
Launchskripte
=============

Im Folgenden sind die wichtigsten Launchfiles aufgelistet.
Die Parameter der Launchfiles kann man sich mit ```ros2 launch <package-name> <launch-file-name>.launch -s``` im Terminal anzeigen lassen.
Diese haben Defaultwerte, welche sich mittels ```<parameter>:=<wert>``` überschreiben lassen.

Launchskripte in bitbots_bringup
================================
Diese Launchskripte sind zu finden in dem bitbots_bringup Packet.

teamplayer.launch
_________________
Will man alle relevanten Softwarepakete beispielsweise für ein Spiel, starten, launched man den Teamplayer. 
Hierfür muss der Motorstrom am Roboter angeschaltet werden. Nach dem Start fährt der Roboter die Walkready-Position an. 


highlevel.launch
________________
Dieses Launchskript startet alle spielrelevanten Komponenten außer der Motion.
Hierfür muss der Motorstrom am Roboter angeschaltet werden. Nach dem Start fährt der Roboter die Walkready-Position an.


motion.launch
_____________
Diese Launchskript startet alle relevanten Softwarekomponenten um die Motion des Roboters zu gewährleisten.
Ist dieses Launchskript gestartet, können die Motoren angesteuert und Bewegungen auf dem Roboter ausgeführt werden, wie Walking oder Animationen.
Hierfür muss der Motorstrom am Roboter angeschaltet werden. Nach dem Start fährt der Roboter die Walkready-Position an.




simulator_teamplayer.launch
___________________________
Diese Skript startet den Simulator und den Softwarestack des Roboters.




