# Schnupper-Vision

Diese kleine Python-Vision erkennt mit HSV-Farbmaske und Konturen orange Bälle.
Sie ist eine verständliche Referenz, kein Ersatz für die vollständige Vision:
Roboter, Torpfosten und Linien werden nicht veröffentlicht. Die Farbauswahl muss
gegebenenfalls an die Bilder angepasst werden. Die Original-Vision bleibt Standard.

## Vertrag mit dem Notebook

Datei: `src/bitbots_vision/schnupper_vision/detector.py`.
Einstieg: `detect_balls(image_bgr) -> numpy.ndarray`.

- Eingabe: OpenCV-Bild in BGR-Reihenfolge, `uint8`, Form `(H, W, 3)`.
- Ausgabe: numerisches Array `(N, 4)` mit Zeilen `[xmin, ymin, xmax, ymax]`
  in Pixelkoordinaten; rechts und unten sind die exklusiven Boxgrenzen.
- Keine Erkennung: `np.empty((0, 4))`, nicht `[]` oder `None`.
- Koordinaten müssen endlich sein, die Boxen positive Breite und Höhe haben.
  Der ROS-Adapter beschränkt Boxen auf das Bild und entfernt außen liegende Boxen.
- Das Notebook liefert inzwischen **ein Box-Tupel oder `None`**. Für diese ROS-Schnittstelle
  beim Übertragen des Funktionskörpers die Rückgaben anpassen: `None` wird zu
  `np.empty((0, 4))`, ein Tupel `box` zu `np.asarray([box], dtype=float)`.
  Den Notebook-Aufruf `zu_hsv(image_bgr, bgr=True)` durch
  `cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV_FULL)` ersetzen; die Hilfsdatei wird
  nicht auf dem Roboter benötigt. `cv2` und `np` sind bereits importiert.
  Keine Notebook-Variablen oder ROS-Aufrufe
  voraussetzen. Das Eingabebild nicht verändern.

Die mitgelieferte Funktion erfüllt diesen Vertrag ohne externe Hilfsfunktionen.
Nach Änderungen neu bauen und den laufenden Node neu starten.

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

Die Auswahl wird auch von `vision_standalone.launch`, `highlevel.launch`,
`teamplayer.launch`, `simulator_teamplayer.launch`, `mujoco_simulation.launch.py`
und `demo.launch` weitergereicht. Bei Highlevel-, Teamplayer-, Demo- und
Simulations-Einstiegen heißt der Overlay-Schalter `vision_debug:=true`.
Der direkte Einstieg `bitbots_vision vision.launch` akzeptiert `debug:=true`.
Beim MuJoCo-Einstieg wird die Auswahl nur mit `teamplayer:=true` an die gestarteten
Stacks übergeben. Zusätzlich wählt `ball_style:=orange` dort das Template
`bitbots_mujoco_sim/xml/kid_field_orange.xml` statt `kid_field.xml`;
`ball_style:=original` bleibt der Standard. Beide Templates haben dieselbe
Platzhalter-Schnittstelle. Die Ballfarbe und das Vision-Backend sind unabhängig
wählbar; für die mitgelieferte HSV-Referenz ist das orange Template vorgesehen.
**Diese vollständigen Bringup-/Simulations-Launches können
Bewegungsprozesse starten; nur in einer ausdrücklich freigegebenen Simulation nutzen.**

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
