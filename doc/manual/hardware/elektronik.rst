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

Due Date ist ende November 2018

ForceFoot2
==========

Basierend auf RHoban's ForceFoot wird ForceFoot2 eine verbesserte Version mit höheren updaterate der Wägezellen und der möglichkeit auf bis zu 4.5 MBaud mit dem Bus zu kommunizieren. Due Date ist mitte Oktober 2018