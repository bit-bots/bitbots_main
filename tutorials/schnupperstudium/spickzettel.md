# 📋 Spickzettel – Ballsuche

Hier stehen alle Befehle, die du in `aufgabe.ipynb` und `evaluation.ipynb` brauchst. Schau hier nach, wenn du nicht weiterweißt.

## Python

| Befehl | Bedeutung | Beispiel |
|---|---|---|
| `print(a, b)` | gibt Werte aus | `print("Anzahl:", 5)` |
| `len(liste)` | Anzahl der Einträge | `len([4, 7, 9])` → 3 |
| `range(n)` | die Zahlen 0, 1, …, n−1 | `range(3)` → 0, 1, 2 |
| `for x in liste:` | wiederholt für jeden Eintrag | `for bild in bilder:` |
| `while bedingung:` | wiederholt, solange die Bedingung gilt | `while y < 10:` |
| `if bedingung:` | nur ausführen, wenn die Bedingung gilt | `if anzahl < 100:` |
| `and` | beide Bedingungen müssen gelten | `x > 0 and y > 0` |
| `liste.append(wert)` | hängt einen Wert hinten an die Liste an | `liste = []` dann `liste.append(5)` → `[5]` |
| `liste[2:5]` | Ausschnitt: Einträge 2, 3, 4 (5 nicht mehr) | `[10, 11, 12, 13, 14, 15][2:5]` → `[12, 13, 14]` |
| `int(zahl)` | ganze Zahl (Nachkommastellen fallen weg) | `int(12.8)` → 12 |
| `def name(…):` | eigene Funktion schreiben | `def histogramm(kanal):` |
| `return wert` | Ergebnis der Funktion zurückgeben | `return x, y` |
| `None` | noch kein Ergebnis oder kein Ball gefunden | `return None` |
| `a is None`, `a is not None` | prüft, ob ein Ergebnis fehlt oder vorhanden ist | `if box is not None:` |
| `(a, b, c, d)` | ein Tupel: mehrere Werte zusammen | `box = (links, oben, rechts, unten)` |
| `min(a, b)`, `max(a, b)` | kleinerer beziehungsweise größerer Wert | `links = max(0, links)` |

## numpy (für Bilder und Masken)

| Befehl | Bedeutung | Beispiel |
|---|---|---|
| `bild[y, x]` | ein Pixel: erst Zeile `y`, dann Spalte `x` | `bild[240, 320]` |
| `bild[:, :, 0]` | Kanal 0 aller Pixel (`:` heißt „alle“) | `h = hsv[:, :, 0]` |
| `bild[y1:y2, x1:x2]` | Ausschnitt: Zeilen `y1` bis `y2 - 1`, Spalten `x1` bis `x2 - 1` | `bild[100:200, 300:400]` |
| `h == 17`, `h >= 100` | vergleicht **jedes** Pixel: `True` oder `False` | `maske = h >= 100` |
| `(a) & (b)` | beide Bedingungen gelten, für jedes Pixel (und) | `(h >= 100) & (h <= 120)` |
| `np.sum(…)` | Summe; bei `True`/`False`: Anzahl der `True` | `np.sum(h == 17)` |
| `np.nonzero(maske)` | Zeilen und Spalten aller markierten Pixel | `zeilen, spalten = np.nonzero(maske)` |
| `zahlen.mean()` | Mittelwert | `spalten.mean()` |
| `maske.size` | Anzahl aller Pixel | `h.size` |
| `bild.shape` | Bildgröße: Höhe, Breite, bei Farbbildern auch Kanäle | `hoehe, breite = maske.shape` |
| `np.zeros_like(kanal)` | gleich großes Array, zunächst nur Nullen | `s = np.zeros_like(h)` |
| `np.iinfo(bild.dtype)` | Wertebereich des Ganzzahl-Datentyps | `np.iinfo(bild.dtype).max` |
| `np.full((hoehe, breite, 3), farbe, dtype=np.uint8)` | einfarbiges RGB-Bild aus einem Farbwert | `farbfeld = np.full((16, 16, 3), bild[y, x], dtype=np.uint8)` |

## werkzeuge.py (unsere Hilfsfunktionen)

| Befehl | Bedeutung |
|---|---|
| `bilder_laden()` | lädt eine gemischte Auswahl unserer orangefarbenen Trainingsbilder als RGB-Liste |
| `zu_hsv(bild)` | rechnet ein RGB-Bild in H, S, V um; alle Kanäle verwenden denselben Ganzzahl-Wertebereich wie das Eingabebild |
| `zu_hsv(image_bgr, bgr=True)` | dieselbe HSV-Umrechnung für den BGR-Detektor |
| `bild_anzeigen(bild, titel, x, y)` | zeigt ein Bild, einen Kanal oder eine Maske; mit `x`, `y` zusätzlich ein Kreuz |
| `bild_anzeigen(bild, box=box)` | zeichnet ein einzelnes Bounding-Box-Tupel ein; `None` lässt das Bild ohne Box |
| `histogramm_anzeigen(histogramm, titel)` | zeigt ein Histogramm als Balkendiagramm |
| `detektor_anzeigen(bild, detect_balls)` | wandelt das RGB-Bild für den BGR-Detektor um und zeigt seine Box |
| `auswerten(detect_balls)` | prüft auf der vollständigen Trainingsmenge und zeigt Fehlerbilder |
| `ball_verdoppeln(bild, maske, dx, dy)` | kopiert die markierten Pixel um `dx` nach rechts und `dy` nach unten (Bonus) |

## Eine Bounding Box zurückgeben

`detect_balls(image_bgr)` erhält ein BGR-Bild. Im Notebook sind die Übungsbilder
RGB; `detektor_anzeigen` übernimmt die Umwandlung beim Testen.
Für dieselben HSV-Bereiche wie in `zu_hsv` verwende
`zu_hsv(image_bgr, bgr=True)`.

- `return (links, oben, rechts, unten)` liefert genau eine Bounding Box als Tupel.
- `return None` meldet, dass kein Ball gefunden wurde.
- Rechts und unten sind die exklusiven Grenzen.
- Ist `rechts` noch der letzte eingeschlossene Pixel, verwende `rechts + 1`;
  entsprechend gilt das für `unten`.
- `max(0, links)` und `min(breite, rechts)` beschränken einen Ausschnitt auf das Bild.


## Vom Notebook zur Simulation

Ersetze die vorbereitete Funktion in
[`detector.py`](../../src/bitbots_vision/schnupper_vision/detector.py) durch deine
Funktion aus `evaluation.ipynb`. Typangaben, Tupel und `None` bleiben gleich.
`np` und `zu_hsv` sind dort bereits verfügbar.
Die Befehle zum Bauen und Starten stehen in der
[Vision-Anleitung](../../src/bitbots_vision/README.schnupper.md).
