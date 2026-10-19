# 📋 Spickzettel – Ballsuche

Hier stehen alle Befehle, die du im Notebook `aufgabe.ipynb` brauchst. Schau hier nach, wenn du nicht weiterweißt.

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
| `None` | kein markierter Punkt für die Anzeige | `x, y = None, None` |

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

## werkzeuge.py (unsere Hilfsfunktionen)

| Befehl | Bedeutung |
|---|---|
| `bilder_laden()` | lädt eine gemischte Auswahl unserer orangefarbenen Trainingsbilder als RGB-Liste |
| `zu_hsv(bild)` | rechnet ein Bild in H, S, V um (je 0–255) |
| `bild_anzeigen(bild, titel, x, y)` | zeigt ein Bild, einen Kanal oder eine Maske; mit `x`, `y` zusätzlich ein Kreuz |
| `histogramm_anzeigen(histogramm, titel)` | zeigt ein Histogramm als Balkendiagramm |
| `detektor_anzeigen(bild, detect_balls)` | wandelt das RGB-Bild für den BGR-Detektor um und zeigt seine Boxen |
| `auswerten(detect_balls)` | prüft auf der vollständigen Trainingsmenge und zeigt Fehlerbilder |
| `ball_verdoppeln(bild, maske, dx, dy)` | kopiert die markierten Pixel um `dx` nach rechts und `dy` nach unten (Bonus) |

## Bounding Boxes und Roboterschnittstelle

`detect_balls(image_bgr)` erhält ein BGR-Bild. Im Notebook sind die Übungsbilder
RGB; `detektor_anzeigen` übernimmt die Umwandlung beim Testen.
Für dieselben HSV-Bereiche wie in `zu_hsv` verwende
`cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV_FULL)`.

- `np.array([[links, oben, rechts, unten]], dtype=float)` erzeugt das Boxarray.
- `np.empty((0, 4), dtype=float)` liefert ein leeres Boxarray, wenn kein Ball gefunden wird.
- Jede Zeile ist eine Box. Rechts und unten sind die exklusiven Grenzen.
- Ist `rechts` noch der letzte eingeschlossene Pixel, verwende `rechts + 1`;
  entsprechend gilt das für `unten`.
- `max(0, links)` und `min(breite, rechts)` beschränken einen Ausschnitt auf das Bild.
