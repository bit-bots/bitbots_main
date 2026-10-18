# Orange-Simulationsdaten

Der Generator verwendet ausschließlich die vorhandene Pixi-Umgebung. Er startet
weder ROS-Nodes noch einen Viewer und steuert keine Hardware an:

```sh
pixi run -e default python tutorials/schnupperstudium/generate_sim_dataset.py
```

Die Defaults und konfigurierbaren Bildzahlen, Auflösung, Seed und Testanteile
stehen in `generate_sim_dataset.py` beziehungsweise dessen `--help`.
`--backend osmesa` bietet eine alternative Headless-Implementierung, sofern sie
lokal verfügbar ist. Vorhandene Simulationsdaten werden nur mit `--overwrite`
ersetzt; bei geänderten Bildzahlen bleiben alte, nicht mehr referenzierte Bilder
liegen. Maßgeblich ist immer die aktuelle Teilliste.

## Szenen und Groundtruth

`dataset_orange.xml` ist eine separate robotfreie MuJoCo-Szenenvariante.
`dataset_field.xml` nutzt die Feldtextur, Tore und Panorama-Assets der bestehenden
Kid-Feld-Simulation. Das Verhalten der normalen Simulationsszene und ihrer Roboter
bleibt unverändert. Die vorhandene Paketinstallation nimmt die zusätzlichen XML-Dateien
über ihren Glob automatisch auf.

## Orange-Ball im ROS-Simulator

Für das Ausprobieren der ROS-Vision gibt es zusätzlich `kid_field_orange.xml`.
Anders als die Dataset-Szene enthält sie die regulären Roboter mit Kameras,
Sensoren und Aktuatoren. Sie und die Originalszene binden dieselbe
`kid_field_environment.xml` und `kid_ball.xml` ein. Nur das Ballmaterial
unterscheidet sich; Ballnamen, Joint, Kollisionsgeometrie und Physik bleiben
kompatibel mit dem Simulator. Ohne explizite Auswahl bleibt der Originalball aktiv.

Nach dem Paketbuild kann die Variante bewusst gestartet werden. Der folgende
Befehl startet ROS-Prozesse und den Simulator und gehört nicht zur Offline-Datengenerierung:

```sh
pixi run -e default build --packages-select bitbots_mujoco_sim
pixi run -e default ros2 launch bitbots_mujoco_sim simulator.launch ball_style:=orange
```

Das Launch-Argument setzt den Simulatorparameter `ball_style` für die
automatisch erzeugte Default-Welt. Bei explizitem `world_file` bestimmt stattdessen
die übergebene Szene das Material. Ein übergeordnetes Bringup muss für Orange
`kid_field_orange.xml` als Welt-Template wählen; die Roboter-Platzhalter entsprechen
denen in `kid_field.xml`.

Die Modelltests kompilieren beide Robotervarianten ohne ROS-Nodes oder Viewer:

```sh
pixi run -e default python -m pytest src/bitbots_simulation/bitbots_mujoco_sim/test/test_robot_ball_styles.py
```

## Gerenderte Dataset-Bilder

Jedes Bild entsteht durch MuJoCos Renderer nach Physikschritten. Variiert werden
Ballposition, Radius und Orange-Farbton, Kameraposition und Blickrichtung,
Feldtönung und Beleuchtung. Es gibt keine gemalten oder nachträglich eingesetzten
Bälle. Negative werden mit einem separat kompilierten Modell gerendert, aus dem
der gesamte Ballkörper samt Geometrie und Joint entfernt wurde. Sie enthalten
deshalb auch keinen verbliebenen Ballschatten; Feld und Tore bleiben erhalten.

Positive Boxen stammen aus MuJoCos Geometrie-Segmentierung derselben Szene und
Kamera, nicht aus einer Farbschwelle oder geschätzter Projektion. Sie umschließen
die sichtbaren Ballpixel; rechts und unten sind die Pixelgrenzen exklusiv.
Die Segmentierung wird nur intern verwendet und nicht als Datei gespeichert. Nicht ausreichend sichtbare
positive Beispiele werden neu gezogen. Negative haben eine leere Boxenliste.

## Vertrag und Reproduzierbarkeit

Die Teilliste `preparation/orange/sim_manifest.json` enthält eine JSON-Liste mit
`id`, relativem `image`, `boxes`, `source: "simulation"`, `split` und `provenance`.
Die finale `manifest.json` wird separat zusammengeführt und enthält nur Bildpfad, Boxen, Quelle und Split.
Der Realistic-Datensatz wird von diesem Generator nicht angefasst.

Train und Test verwenden getrennte deterministische NumPy-Seed-Sequenzen;
auch die Klassen besitzen eigene Zufallsströme. Der Split wird vor dem Rendern
festgelegt und nicht aus nachträglich gemischten Bildern abgeleitet.
Die Provenienz enthält Seed-Entropie, Bildindex, Renderparameter, Backend,
MuJoCo-Version sowie Prüfsummen von Generator und verwendeten Szenen/Assets.
Gleiche Argumente und Software erzeugen dieselbe Szenenfolge. Bitidentische
Pixel über andere Grafiktreiber oder MuJoCo-Versionen hinweg sind nicht garantiert.
Die Bilder sind synthetische Trainingsbeispiele, kein Nachweis für die
Erkennungsleistung auf einer echten Kamera.
