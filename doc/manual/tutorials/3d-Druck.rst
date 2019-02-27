==========
3D-Drucken
==========

Autodesk
========
* Das zu druckende Objekt ist in Autodesk zu designen
* Die Datei speichern und exportieren als CAD-Format (.stl)

Slic3r
======
* Das Programm Slic3r öffnen (zu finden im bitbots Ordner-> Sclic3r -> slic3r.exe)
* In Slic3r die Config laden
    * Zu finden in bitbots -> printing -> lab prusali3 -> slic3r profiles -> pla_1.75 auswählen
    * fast
    * bei printer config den black Drucker auswählen
    * alternativ zu finden hier: https://redmine.mafiasi.de/projects/printing-lab
* Oben rechts in Slicer folgende Settings auswählen:
    * print settings: fast
    * filament: 1.75
    * printer: black
* Objektdatei (stl) in slic3r öffnen
* falls nötig: Objekte rechtsklicken und rotieren
* oben arrange anklicken
* preview unten ansehen-> dabei können zu dünne Teile auffallen
* export als g-code auswählen

Octoprint
=========
* ggf. Drucker einschalten
* im Webbrowser den 3D-Drucker öffnen:
* tamsprusa2.informatik.uni-hamburg.de (nur im Uni Netz)
* Datei hochladen über Web UI
* hochgeladene Datei anklicken
* Drucken klicken