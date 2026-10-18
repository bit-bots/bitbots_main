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

`vision_workshop.py`, `workshop_plots.py` und `download_datasets.py` müssen neben
den Notebooks liegen. Sie laden Bilder, zeichnen Ansichten und berechnen die Kennzahlen.
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

## Lernweg und Musterlösung

`aufgabe.ipynb` führt von Pixelkoordinaten und Bounding Boxes über RGB und HSV
zur eigenen Farbmaske und zu einem ersten Detektor. Die Zeichenhilfen in
`workshop_plots.py` übernehmen die Darstellung; die Erkennungslogik bleibt im
Notebook sichtbar. Vorbereitung und längere Anzeigehilfen sind mit
`jupyter.source_hidden` und `hide-input` zum Einklappen markiert. Ob sie automatisch
verborgen werden, hängt von der Notebook-Oberfläche ab.

`loesung.ipynb` folgt demselben Lernweg mit ausgefüllten Schritten. Die
Maskenbereinigung und Formprüfung sind Vertiefungen. Die endgültige Referenz
nutzt die Formprüfung, aber keine Maskenbereinigung. Die Maskenanzeige verwendet
immer die aktuelle `make_mask`-Funktion, keine separate Referenzpipeline.
Nach Änderungen an Grenzen oder Funktionen alle abhängigen Zellen erneut ausführen.

## Parametervergleich für Betreuende

`tune_reference.py` vergleicht Farb- und Formgrenzen ausschließlich auf der
vollständigen orangefarbenen Trainingsmenge. Auswahlkriterium ist das Mittel der
F1-Werte für Kamera und Simulation; bei Gleichstand bleibt der zuerst geprüfte
Kandidat. Der Suchraum steht im Skript. Maskenbereinigung ist in den Kandidaten
deaktiviert. Die ursprüngliche Referenz bleibt als Vergleich im Skript erhalten.

```sh
pixi run -e default python tutorials/schnupperstudium/tune_reference.py --output /tmp/workshop-tuning.json
```

Mit `--evaluate-test` werden erst nach der Auswahl auch Abschlusstest und Transfer
ausgewertet. Diese Ergebnisse beeinflussen die Parameterwahl nicht. Die bereits
vor der Überarbeitung betrachteten Testergebnisse sind bekannt; dieser Vergleich
ist daher kein neu angelegter Blindtest. Es wurden keine Testbilder zur Auswahl
von Grenzwerten verwendet.

Der erzeugte Bericht [tuning_results.json](tuning_results.json) enthält Suchraum,
gewählte Parameter, Datensatzmanifest-Prüfsummen und getrennte Ergebnisse für
Kamera und Simulation. Er dokumentiert eine Messung auf den vorhandenen Daten,
keine allgemeine Leistungszusage. Der Transfer auf andersfarbige Bälle bleibt
eine bewusste Grenze dieser Methode. Ein neuer Suchlauf verändert die
Notebook-Parameter nicht automatisch.

Offline-Prüfungen für die Notebook-Detektoren:

```sh
pixi run -e default pytest tutorials/schnupperstudium/test_notebooks.py
```
