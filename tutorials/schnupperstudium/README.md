# Schnupperstudium: einen Ball im Kamerabild finden

Öffne `aufgabe.ipynb` und führe die Zellen von oben nach unten aus.
Du entwickelst mit OpenCV einen eigenen Balldetektor und untersuchst seine Treffer
und Fehler auf echten und simulierten Bildern.

## Start mit VS Code

Die Erweiterungen **Python** und **Jupyter** installieren. Im Notebook oben
**Select Kernel → Python Environments** auswählen und den Interpreter
`.pixi/envs/default/bin/python` des Repositorys verwenden.
VS Code muss dafür nicht aus einer Pixi-Shell gestartet werden.

Alternativ vom Repository-Root JupyterLab starten:

```sh
pixi run -e default jupyter lab tutorials/schnupperstudium/aufgabe.ipynb
```

## Dateien und Datensätze

`vision_workshop.py` und `download_datasets.py` müssen neben dem Notebook liegen.
Sie laden Bilder, zeichnen Debugansichten und berechnen die Kennzahlen.
Die Download-URLs sind im Notebook bereits eingetragen. Fehlende Datensätze werden
mit `wget` von `https://data.bit-bots.de/Schnupperstudium2026/` geladen und entpackt.
Vorhandene lokale Daten unter `datasets/` werden offline wiederverwendet.
Quellen-, Lizenz- und Autorenangaben des TORSO-Subsets bleiben beim Datensatz.

Jeder Eintrag in `manifest.json` beschreibt ein Bild:

- `image`: relativer Bildpfad.
- `boxes`: Ballrechtecke als `[links, oben, rechts, unten]` in Pixeln.
  Die rechte und untere Grenze liegen direkt hinter dem letzten zugehörigen Pixel.
  Eine leere Liste bedeutet: kein Ball.
- `source`: Kameraaufnahme (`real`) oder Simulation (`simulation`).
- `split`: Trainingsbilder (`train`) oder Abschlusstest (`test`).

## Arbeiten und auswerten

Entwickle deine Funktion zunächst auf Trainingsbildern. Verwende die Masken und
Overlays, um Fehlentscheidungen zu verstehen. Lege deine Einstellungen fest,
bevor du den Abschlusstest aktivierst. Probiere den fertigen Detektor anschließend
unverändert auf dem Datensatz mit normalen Bällen aus.

Ändere nur deinen Detektor; Referenzlabels und Auswertung bleiben unverändert.
Die Notebooks starten keine Roboterprozesse.
