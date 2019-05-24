==========
Elektronik
==========

Powerboard
==========

Das Powerboard regelt die Stromversorgung durch zwei Dioden (in einem Package), regelt die Stromversorgung der Motoren durch einen Mechanischen Schalter, und verteilt den Strom auf mehrere Stecker.

* Schaltplan :download:`pdf <elektronik/wolfgang_power.pdf>`
* PCB combined :download:`pdf <elektronik/wolfgang_power_pcb.pdf>`
* PCB Top :download:`pdf <elektronik/wolfgang_power_pcb_top.pdf>`
* PCB Bottom :download:`pdf <elektronik/wolfgang_power_pcb_bottom.pdf>`
* PCB Silk :download:`pdf <elektronik/wolfgang_power_pcb_silk.pdf>`

DXLBoard
========

Das DXL Board schreibt die Kommandos welches es von bitbots_ros_control bekommt auf den Dynamixel Bus. Enwickelt wurde es von RHoban

* Schaltplan :download:`pdf <elektronik/dxlboard.pdf>`
* PCB :download:`pdf <elektronik/dxlboard_pcb.pdf>`

Wolfgang Core
=============

Wolfgang Core (COntrolling and Regulating Electronics) befindet sich derzeit in Entwicklung. Es wird das DXLBoard und das Powerboard ersetzten.
Folgende Features sind geplant:

* Bus mit einer höheren Taktrate betreiben
* 3 Busse für jeweils die beine und den Rest des Roboters haben
* Anschlüsse für bis zu 2 IMUs
* GPIO (Buttons, LEDs etc.) über einen I2C Bus an dem ein Arduino hängt (sowie die beiden IMUs)
* Stromregulierung für 5V komponenten (Odroid, Verstärker)
* Softwareseitige Motorstromsteuerung

Due Date ist ende Dezember 2019

Bit Foot
========

Der Bit Foot ist ein verbesserter fußandrucksensor basierend auf dem Rhoban ForceFoot.

* Schaltplan :download:`pdf <elektronik/bitfoot.pdf>`
* PCB :download:`pdf <elektronik/bitfoot_pcb.pdf>`

Auf der ursprüngliche Variante des Boards ist der DXL Bus an USART3 angeschlossen. Die alte version sinde grüne Boards, die aktuelle (V 1.0) sind schwarze Boards.

Derzeit werden die Dioden (D1 und D2) und R9 nicht bestückt.

R11 und R15 werden nur bestückt wenn TTL statt RS485 als BUS verwendet wird.
