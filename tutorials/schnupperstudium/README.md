# Schnupperstudium: einen Ball im Kamerabild finden

Öffne `aufgabe.ipynb`. Du entwickelst mit Histogrammen, Farbvergleichen und
Mittelwerten eine Bounding Box für den Ball. Danach fasst du deine Schritte in
`evaluation.ipynb` zu einer Funktion zusammen und prüfst sie auf weiteren Bildern.
Der `spickzettel.md` hilft beim Nachschlagen der benötigten Befehle.

## Start

In VS Code die Erweiterungen **Python** und **Jupyter** installieren und als
Notebook-Kernel `.pixi/envs/default/bin/python` des Repositorys auswählen.
Die Notebooks verwenden ihren Ordner als Arbeitsverzeichnis. Führe die Zellen
von oben nach unten aus und nach Änderungen auch die davon abhängigen Zellen.

Alternativ vom Repository-Root starten:

```sh
pixi run -e default jupyter lab tutorials/schnupperstudium/aufgabe.ipynb
```

## Bilder und Hilfsdateien

`bilder_laden()` lädt eine gemischte Auswahl echter und simulierter Trainingsbilder.
Fehlende Datensätze werden automatisch heruntergeladen; vorhandene Daten unter
`datasets/` werden offline wiederverwendet. Die Download-Adressen stehen in
`werkzeuge.py`. Quellen- und Lizenzangaben des TORSO-Datensatzes bleiben bei den Daten.

Die Dateien `werkzeuge.py` und `download_datasets.py` müssen
neben den Notebooks liegen. Die HSV-Abbildung unter `abbildungen/` ist lokal verfügbar;
dort stehen auch Urheber und Lizenz.

Die Übungsbilder sind RGB. Der Detektor in `evaluation.ipynb` erhält BGR und nutzt
`zu_hsv(image_bgr, bgr=True)`. Er liefert ein Tupel `(links, oben, rechts, unten)`
oder `None`, wenn er keinen Ball findet. Rechts und unten liegen direkt hinter
den eingeschlossenen Pixeln. Wir erwarten höchstens eine Box pro Bild.

## Auswerten

Entwickle auf den Trainingsbildern und lege deine Lösung fest, bevor du die
Testbilder freischaltest. Betrachte die Ergebnisse für Kamera und Simulation
getrennt. Die Auswertung vergleicht deine Box mit den vorgegebenen Groundtruth-Boxen;
verändere diese und die Auswertungsregeln nicht. Probiere die fertige Funktion
anschließend unverändert auf Bildern mit andersfarbigen Bällen aus.

## Eigene Vision und orange Simulation

Die benötigte ROS-Anbindung und die orange Simulationsszene gehören zu diesem
Branch. `src/bitbots_vision/schnupper_vision/detector.py` ist ein ausfüllbarer Einstieg;
ohne eure Erkennung liefert er noch keine Treffer.

Wie du deine Funktion überträgst und die orange Simulation auswählst, steht in
[der Vision-Anleitung](../../src/bitbots_vision/README.schnupper.md).
Die Notebooks selbst starten keine Roboterprozesse und bewegen keine Hardware.
