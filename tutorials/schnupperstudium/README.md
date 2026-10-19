# Schnupperstudium: einen Ball im Kamerabild finden

Mit Histogrammen, Farbvergleichen und Mittelwerten entwickelt ihr eine kleine Ball-Vision.
Der Lernweg und die Bonusaufgaben folgen der bereitgestellten Ballsuche-Vorlage des Kollegen.
Die allgemeine Python-Grundlageneinführung entfällt; benötigte Befehle stehen im Spickzettel.
`aufgabe.ipynb` enthält die Aufgaben bis zur einzelnen Bounding Box.
`evaluation.ipynb` enthält Teil 5 mit dem auszufüllenden Detektor und der Auswertung;
`loesung.ipynb` zeigt eine erklärbare Referenzlösung ohne neuronales Netz.
Beide Notebooks vergleichen echte und simulierte Bilder sowie Fehlerfälle.

## Start mit VS Code

Im Repository arbeiten und die Extensions **Python** und **Jupyter** installieren.
Das gewünschte Notebook öffnen und oben **Select Kernel → Python Environments**
auswählen. Als Interpreter die Pixi-Umgebung
`.pixi/envs/default/bin/python` des Repositorys verwenden.
Falls sie nicht angeboten wird, den Interpreter über seinen Pfad auswählen.
Ein `pixi shell` oder manuelles Sourcen ist nicht nötig.
Optional startet `pixi run -e default code .` VS Code mit Pixi-Aktivierung.

## Alternativ: JupyterLab

Vom Repository-Root starten:

```sh
pixi run -e default jupyter lab tutorials/schnupperstudium/aufgabe.ipynb
```

Danach die Zellen von oben nach unten ausführen.
Für die Referenzlösung entsprechend `loesung.ipynb` öffnen.

## Datensätze laden

`bilder_laden()` verwendet unsere vorhandenen Datensätze. Die Download-URLs für
**orange** und **realistic** sind in `werkzeuge.py` als `DATASET_URLS` eingetragen.
Die Hilfsfunktion lädt nur fehlende Datensätze mit `wget` herunter.
Vollständige lokale Datensätze werden geprüft und offline wiederverwendet.
Bei Fehlern bleibt ein vorhandener Datensatz unverändert; die Meldung erklärt,
welche URL oder lokale Datei geprüft werden muss.

Für Betreuende: Die vorbereiteten Datensätze lassen sich unverändert packen:

```sh
tar -czf orange.tar.gz -C tutorials/schnupperstudium/datasets orange
tar -czf realistic.tar.gz -C tutorials/schnupperstudium/datasets realistic
```

Die Archive unter den in `werkzeuge.py` eingetragenen URLs bereitstellen.
Die Auswertung zeigt den jeweils geprüften Datenumfang.

## Wie sind die Labels aufgebaut?

Jeder Datensatz enthält `images/` und eine `manifest.json`: eine Liste, in der
jeder Eintrag ein Bild beschreibt. Es gibt nur folgende Angaben:

- `image`: relativer Pfad zum Bild.
- `boxes`: Ballrechtecke als `[links, oben, rechts, unten]` in Bildpixeln.
  Eine leere Liste bedeutet: kein Ball. Rechts und unten liegt die Grenze
  direkt hinter dem letzten zugehörigen Pixel.
- `source`: `real` für Kameraaufnahmen oder `simulation` für simulierte Bilder.
- `split`: `train` zum Entwickeln oder `test` für den Abschlusstest.

Die Notebooks benötigen keine gespeicherten Segmentierungsmasken.

Erzeugungsdetails liegen außerhalb der Datensätze unter `preparation/`; sie müssen
nicht mit in die Archive. Lizenz und Quellenangaben des TORSO-Datensatzes
bleiben enthalten.

## Fair auswerten und weiterarbeiten

Nur auf Trainingsbildern entwickeln; den Abschlusstest erst danach freischalten.
Precision, Recall, F1 und Box-Overlays zeigen Treffer und Fehlentscheidungen.
Die Masken in den Lernschritten werden direkt aus dem Schülercode angezeigt.
Den unveränderten Detektor anschließend auf normalen Bällen ausprobieren.
Die Referenz ist eine Farbregel, kein allgemeiner Ballerkennungs-Benchmark.
Sie sucht vom Mittelwert der markierten Pixel aus zusammenhängende Bereiche im
Zeilen- und Spaltenhistogramm und gibt höchstens eine Bounding Box zurück.
Mehrere Bälle und fremde orange Flächen können die Mitte verschieben. Der Bonus
macht diese Grenze sichtbar. Die Rückgabe ist ein einzelnes Tupel
`(links, oben, rechts, unten)` beziehungsweise `None`, wenn kein Ball gefunden wird.
Die Messergebnisse des früheren Konturdetektors gelten nicht für diese Methode.

Die Übungsbilder werden als RGB angezeigt; `zu_hsv` verwendet HSV_FULL.
Der abschließende `detect_balls` erhält dagegen BGR und liefert ein Box-Tupel oder `None`.
Die Hilfsfunktion `auswerten` passt dies intern an die unveränderte Mehrbox-Auswertung an.
Der Funktionskörper verwendet `np` und die Hilfsfunktion `zu_hsv`.
`zu_hsv(image_bgr, bgr=True)` berücksichtigt die BGR-Kanalreihenfolge.
Die fertige Lösung behandelt leere Masken und stoppt die Suche an allen Bildrändern.

Die HSV-Abbildung liegt lokal unter `abbildungen/`; Urheber und Lizenz sind dort
und in den Lernnotebooks angegeben. Die Notebooks werden mit dem Notebook-Ordner
als Arbeitsverzeichnis ausgeführt; es gibt keine automatische Pfadsuche mehr.

Für den ROS-Wechsel den Backend-Schalter `vision_backend:=schnupper` verwenden.
Startoptionen, Debug-Topics und Grenzen erklärt
[die Vision-Anleitung](../../src/bitbots_vision/README.schnupper.md).
Die Notebooks selbst starten keine Roboterprozesse und bewegen keine Hardware.

## Welche Dateien werden gebraucht?

Für die Durchführung mit vorbereiteten Datensätzen genügt dieser Notebook-Ordner
mit den folgenden Dateien:

- `aufgabe.ipynb`: Arbeitsblatt für die Schülerinnen und Schüler.
- `loesung.ipynb`: vollständige Referenz einschließlich Detektor und Auswertung.
- `evaluation.ipynb`: Teil 5 und Auswertung zum Übertragen der eigenen Lösung.
  Mit eigenem Kernel ausführbar; Variablen aus dem Aufgaben-Notebook werden nicht übernommen.
- `werkzeuge.py`: einfache RGB-Anzeigen, HSV-Umrechnung und Zugang zu unseren Datensätzen.
- `spickzettel.md`: benötigte Befehle zum Nachschlagen.
- `abbildungen/`: HSV-Zylinder und Quellen-/Lizenzangaben für die Offline-Anzeige.
- `vision_workshop.py`: lädt Bilder und Labels und berechnet die unveränderten Kennzahlen.
- `download_datasets.py`: lädt und entpackt die in `werkzeuge.py` angegebenen Archive.
- `README.md`: Einstieg und Durchführung.
- `datasets/orange/` und `datasets/realistic/`: Bilder und `manifest.json`;
  alternativ werden diese Ordner aus den veröffentlichten Archiven geladen.
  Beim TORSO-Subset gehören die Quellen-, Lizenz- und Autorenangaben dazu.

Die Hilfsmodule müssen neben den Notebooks liegen. Zusätzlich wird eine passende
Python-Umgebung benötigt; im Repository stellt Pixi sie bereit.
Die Archive enthalten ausschließlich die Datensätze, keine Notebooks oder Hilfsmodule.

Für den optionalen Schritt vom Notebook zur laufenden ROS-Vision bleiben unter
`src/bitbots_vision/` erforderlich:

- `schnupper_vision/detector.py`: hier die entwickelte Funktion einsetzen.
- `schnupper_vision/node.py`, `schnupper_vision/__init__.py` und
  `schnupper_vision_node`: ROS-Anbindung und ausführbarer Einstieg.
- `launch/vision_backend.launch.py`: Auswahl der Vision durch die vorhandenen Launch-Dateien.
- `README.schnupper.md`: Anleitung für diesen Schritt.

Dazu gehören die Anpassungen an Build, Paketabhängigkeiten und bestehenden
Vision-/Bringup-Launch-Dateien. Für reine Notebook-Arbeit sind diese nicht nötig.

Für das optionale Ausprobieren im Simulator werden unter
`src/bitbots_simulation/bitbots_mujoco_sim/xml/` außerdem
`kid_field_orange.xml`, `kid_field_environment.xml` und `kid_ball.xml` gebraucht.
Die gemeinsamen Includes werden auch von der Originalszene verwendet und dürfen
bei der aktuellen Implementierung nicht einzeln entfernt werden.
Die zugehörigen Anpassungen an Simulation und Launch-Dateien gehören ebenfalls dazu.

Die übrigen Dateien werden während des Workshops nicht ausgeführt:

- `generate_sim_dataset.py` sowie `xml/dataset_field.xml` und `xml/dataset_orange.xml`
  im Simulationspaket: neue Offline-Simulationsbilder erzeugen.
- `prepare_real_dataset.py`, `real_annotations.json`, `real_exclusions.json`:
  Kameraaufnahmen importieren und die Dunkelphase ausschließen.
- `prepare_torso_dataset.py`: das TORSO-Subset erneut zusammenstellen.
- `assemble_orange_dataset.py`: vorbereitete Real- und Simulationsdaten zusammenführen.
- `SIM_DATASET.md`: Dokumentation der Datenerzeugung und Simulationsvariante.
- `render_notebooks.py`: Beispielausgaben für die Notebooks vorbereiten.
- `test_notebooks.py`: Histogramm-Detektor, RGB/BGR-Vertrag und Bildrandfälle.
- `test_workshop.py`, `test_dataset_download.py`, `test_dataset_preparation.py`,
  `test_real_exclusions.py`: Tests der Notebook-Helfer und Datenvorbereitung.
- `test/test_schnupper_vision.py` im Vision-Paket sowie
  `test/test_dataset_generation.py` und `test/test_robot_ball_styles.py` im Simulationspaket:
  Tests der ROS-Anbindung und Simulationsszenen.
- `preparation/`: lokale Erzeugungsdetails, außerhalb der Schülerdatensätze.
- `.gitignore`: hält heruntergeladene Daten und lokale Vorbereitungsdateien aus Git heraus.

Diese Vorbereitungs- und Testdateien können zur Wartung im Repository bleiben;
für die Durchführung müssen sie nicht verteilt werden. Die manuell geprüften
Labels in `datasets/*/manifest.json` sind der aktuelle Stand für die Archive.
Ein erneuter Import oder Zusammenbau aus älteren Vorbereitungsdaten kann manuelle
Korrekturen überschreiben und ist zum Packen nicht erforderlich.

## Vergleich des Value-Filters

`value_filter_comparison.json` dokumentiert den Vergleich derselben Histogramm-Lösung
mit und ohne die Helligkeitsbedingung. Farbton-, Sättigungs- und Geometriegrenzen
wurden dabei nicht verändert. Die Entscheidung, den Filter beizubehalten, wird bereits
durch die vollständige Trainingsmenge gestützt; auch die Testauswertung fällt mit
Filter besser aus. Die Ergebnisse gelten für die vorhandenen Datensätze und diese
Parameter, nicht für beliebige Kameras oder Farbgrenzen.
