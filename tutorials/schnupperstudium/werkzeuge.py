"""Laden und Anzeigen für die Ballsuche, nach der Vorlage aus ballsuche/werkzeuge.py."""

import cv2
import matplotlib.pyplot as plt
import numpy as np
from download_datasets import ensure_datasets
from matplotlib.patches import Rectangle
from vision_workshop import as_boxes, evaluate, load_dataset, plot_gallery, print_evaluation

DATASET_URLS = {
    "orange": "https://data.bit-bots.de/Schnupperstudium2026/orange.tar.gz",
    "realistic": "https://data.bit-bots.de/Schnupperstudium2026/realistic.tar.gz",
}


def bilder_laden(limit=12):
    """Lade eine gemischte Auswahl unserer Trainingsbilder in RGB-Reihenfolge."""
    ensure_datasets(DATASET_URLS)
    records = load_dataset("orange", split="train", limit=limit)
    return [cv2.cvtColor(record["image_bgr"], cv2.COLOR_BGR2RGB) for record in records]


def zu_hsv(bild: np.ndarray, *, bgr: bool = False) -> np.ndarray:
    """Wandle ein RGB- oder ausdrücklich als BGR markiertes Bild nach HSV um."""
    conversion = cv2.COLOR_BGR2HSV_FULL if bgr else cv2.COLOR_RGB2HSV_FULL
    return cv2.cvtColor(np.asarray(bild, dtype=np.uint8), conversion)


def bild_anzeigen(bild, titel="", x=None, y=None, box=None, groesse=(8, 4), achsen=True):
    """Zeige RGB-Bild, Kanal oder Maske; optional mit Kreuz und einer Bounding Box."""
    bild = np.asarray(bild)
    if bild.ndim not in (2, 3) or bild.size == 0:
        print("Noch kein darstellbares Bild. Prüfe die Aufgabe für diesen Ausschnitt oder Kanal.")
        return
    figure, axis = plt.subplots(figsize=groesse, layout="constrained")
    vmax = 1 if bild.dtype == bool else 255
    axis.imshow(bild, cmap="gray", vmin=0, vmax=vmax)
    if x is not None and y is not None and np.isfinite([x, y]).all():
        axis.plot(x, y, "w+", markersize=30, markeredgewidth=3)
    if box is not None:
        for left, top, right, bottom in box_als_array(box):
            axis.add_patch(Rectangle((left, top), right - left, bottom - top, fill=False, edgecolor="magenta"))
    axis.set(title=titel, xlabel="x → nach rechts", ylabel="y → nach unten")
    if not achsen:
        axis.axis("off")
    plt.show()
    plt.close(figure)


def histogramm_anzeigen(histogramm, titel=""):
    """Zeige eine bereits berechnete Zählliste."""
    figure, axis = plt.subplots(figsize=(8, 3), layout="constrained")
    axis.bar(range(len(histogramm)), histogramm, width=1)
    axis.set(title=titel, ylabel="Anzahl Pixel")
    plt.show()
    plt.close(figure)


def ball_verdoppeln(bild, maske, dx, dy):
    """Kopiere markierte Pixel verschoben ins Bild, ohne das Original zu verändern."""
    neues_bild = bild.copy()
    hoehe, breite = maske.shape
    zeilen, spalten = np.nonzero(maske)
    for y, x in zip(zeilen, spalten, strict=True):
        if 0 <= y + dy < hoehe and 0 <= x + dx < breite:
            neues_bild[y + dy, x + dx] = bild[y, x]
    return neues_bild


def detektor_anzeigen(bild, detector):
    """Passe das RGB-Übungsbild an den BGR-Vertrag des Roboterdetektors an."""
    box = detector(cv2.cvtColor(bild, cv2.COLOR_RGB2BGR))
    bild_anzeigen(bild, "Deine Bounding Box", box=box)
    return box


def box_als_array(box):
    """Passe genau ein Box-Tupel oder None an die interne Mehrbox-Auswertung an."""
    if box is None:
        return np.empty((0, 4), dtype=float)
    if not isinstance(box, tuple) or len(box) != 4:
        raise ValueError("Erwartet wird eine Bounding Box als Tupel (links, oben, rechts, unten) oder None.")
    return as_boxes([box])


def auswerten(detector, datensatz="orange", split="train"):
    """Prüfe unverändert auf dem gesamten Split, getrennt nach Bildquelle."""
    records = load_dataset(datensatz, split=split, limit=None)
    result = evaluate(records, lambda image: box_als_array(detector(image)), threshold=0.5)
    print_evaluation(result, f"{datensatz}/{split}")
    failures = [item for item in result["items"] if item["fp"] or item["fn"]]
    if failures:
        figure = plot_gallery(failures, limit=6)
        plt.show()
        plt.close(figure)
    else:
        print("In diesen Bildern sind keine Fehler aufgetreten.")
    return result
