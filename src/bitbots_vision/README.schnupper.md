# Schnupper-Vision

Diese Python-Vision verbindet eure Ballerkennung mit ROS. Der Einstieg in
`schnupper_vision/detector.py` ist zum Ausfüllen vorbereitet und liefert zunächst
keine Treffer. Setzt eure Lösung aus `evaluation.ipynb` ein.
In den Teamplayer-Einstiegen ist die Schnupper-Vision vorausgewählt. Sie erkennt
nur Bälle; Roboter, Torpfosten und Linien werden nicht erkannt.

## Funktion aus dem Notebook übernehmen

Kopiert eure vollständige Funktion `detect_balls` aus `evaluation.ipynb` nach
`src/bitbots_vision/schnupper_vision/detector.py` und ersetzt dort die vorbereitete
Funktion. Die Eingabe und Rückgabe stimmen mit dem Notebook überein:

```python
def detect_balls(image_bgr: np.ndarray) -> tuple[int, int, int, int] | None:
    ...
```

Die Eingabe ist ein BGR-Bild. Die Funktion liefert ein Tupel
`(links, oben, rechts, unten)` oder `None`, wenn kein Ball gefunden wurde.
Rechts und unten sind exklusive Grenzen. Die Box muss endliche Koordinaten
und eine positive Breite und Höhe haben. Die ROS-Anbindung beschränkt sie
auf das Bild und übersetzt sie in eine Ballnachricht.

`np` und die Hilfsfunktion `zu_hsv(image_bgr, bgr=True)` stehen in `detector.py`
bereits bereit. Die HSV-Umrechnung ist dieselbe wie im Notebook; ihr müsst keine
Rückgaben oder Farbgrenzen für ROS umschreiben. Übernehmt keine Notebook-Variablen
und verändert das Eingabebild nicht.

Nach Änderungen neu bauen und den laufenden Vision-Node neu starten.

## Bauen und offline prüfen

Alle Befehle im Workspace ausführen. Es werden keine neuen Dependencies benötigt;
die Python-, ROS-, NumPy- und OpenCV-Pakete der vorhandenen Pixi-Umgebung werden benutzt.

```bash
pixi run -e default build --packages-select bitbots_vision bitbots_bringup
pixi run -e default test --packages-select bitbots_vision
```

## Umschalten

Mit einer **bereits separat gestarteten** Simulation oder einem passenden Bildstrom
(diese Befehle starten weder Kamera noch Motoren):

```bash
pixi run -e default ros2 launch bitbots_bringup vision.launch sim:=true camera:=false vision_backend:=schnupper debug:=true
```

Zurückschalten mit `vision_backend:=original`. Nicht beide Visionen parallel auf
denselben Topics starten. `sim` wird als `use_sim_time` weitergegeben; in Simulation
muss `/clock` vorhanden sein. Außerhalb der Simulation `sim:=false` verwenden.
Bei Nutzung des Zenoh-RMW muss wie im restlichen Stack ein Zenoh-Router laufen;
dieser lässt sich separat mit `pixi run -e default zenoh` starten.

## Simulation mit der eigenen Vision

Die Simulations-Einstiege wählen standardmäßig den orangefarbenen Ball.
`teamplayer.launch`, `simulator_teamplayer.launch` und die Teamplayer-Stacks aus
`mujoco_simulation.launch.py` wählen standardmäßig `vision_backend:=schnupper`.
`teamplayer.launch` startet den Roboter-Stack; die Simulation wird über einen
der Simulations-Einstiege gestartet.

Eine Simulation mit einem Roboter samt Stack startet ihr mit:

```bash
pixi run -e default ros2 launch bitbots_bringup mujoco_simulation.launch.py vision_debug:=true
```

Für mehrere Roboter ergänzt ihr `num_robots`; für zwei gegnerische Roboter
verwendet ihr `one_versus_one:=true`. Die Pixi-Aufgaben `sim` und `sim-one`
verwenden dieselben Voreinstellungen. Mit `teamplayer:=false` kann die Simulation
separat laufen und der Stack später in der ROS-Domain des jeweiligen Roboters
über `teamplayer.launch sim:=true` gestartet werden.

Mit `ball_style:=original` und `vision_backend:=original` könnt ihr die bisherige
Szene und Vision auswählen. Bei `simulator_teamplayer.launch` lässt sich die
Ballfarbe ebenfalls mit `ball_style` wählen. `vision_debug:=true` aktiviert die
Boxanzeige bei den Teamplayer-Einstiegen; bei `vision.launch` heißt der Schalter
`debug:=true`.

Diese Befehle starten Simulations- und gegebenenfalls Bewegungsprozesse.
Verwendet sie für die Simulation mit `sim:=true`, wenn der Einstieg dieses
Argument verlangt.

## ROS-Vertrag

Der ausführbare Adapter heißt `schnupper_vision_node`, der ROS-Node wie bisher
`bitbots_vision`. Er liest dieselbe `config/visionparams.yaml` wie das Original.
Die Parameter `topics.image`, `topics.balls`, `topics.debug_image` und
`components.debug_active` behalten ihre Bedeutung. Die konkreten Standard-Topics
stehen in dieser Konfiguration und werden nicht separat im Tutorial gepflegt.
Remappings sind wie bei anderen ROS-Nodes möglich.

Für jedes erfolgreich dekodierte Eingabebild erscheint eine
`soccer_vision_2d_msgs/msg/BallArray`-Nachricht, auch ohne Treffer.
`bb`, `center` und `confidence` werden befüllt; der Adapter setzt eine feste
Konfidenz, da der Schülervertrag keine Modellwahrscheinlichkeit liefert.
Bild-Zeitstempel und Frame-ID werden unverändert in BallArray und Debugbild übernommen.
Die Debugausgabe ist ein BGR-Bild mit eingezeichneten Boxen.
Bei fehlerhaften Schülerergebnissen wird ein Fehler geloggt und eine leere Ballliste
veröffentlicht, statt alte Erkennungen weiterzureichen.

Die Bild-Subscription akzeptiert Best-Effort- und Reliable-Publisher; Ball- und
Debugausgabe sind wie beim Original Reliable. Der Adapter lädt kein ONNX-Modell.
