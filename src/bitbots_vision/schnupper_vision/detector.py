"""Einstieg für die selbst erarbeitete Ballerkennung aus dem Notebook."""

import cv2
import numpy as np


def zu_hsv(bild: np.ndarray, *, bgr: bool = False) -> np.ndarray:
    """Dieselbe HSV-Umrechnung wie in werkzeuge.py, ohne Notebook-Abhängigkeiten."""
    conversion = cv2.COLOR_BGR2HSV_FULL if bgr else cv2.COLOR_RGB2HSV_FULL
    return cv2.cvtColor(np.asarray(bild, dtype=np.uint8), conversion)


def detect_balls(image_bgr: np.ndarray) -> tuple[int, int, int, int] | None:
    h_von, h_bis = (0, 28)
    s_min, v_min = (140, 80)
    min_pixel = 100
    min_pro_zeile = 3
    hsv = zu_hsv(image_bgr, bgr=True)
    h = hsv[:, :, 0]
    s = hsv[:, :, 1]
    v = hsv[:, :, 2]
    maske = (h >= h_von) & (h <= h_bis) & (s >= s_min) & (v >= v_min)
    zeilen, spalten = np.nonzero(maske)
    if len(spalten) < min_pixel:
        return None
    zeilen_histogramm = []
    for zeile in range(maske.shape[0]):
        zeilen_histogramm.append(np.sum(maske[zeile, :]))
    spalten_histogramm = []
    for spalte in range(maske.shape[1]):
        spalten_histogramm.append(np.sum(maske[:, spalte]))
    x = int(spalten.mean())
    y = int(zeilen.mean())
    if zeilen_histogramm[y] < min_pro_zeile or spalten_histogramm[x] < min_pro_zeile:
        return None
    links = rechts = x
    oben = unten = y
    while unten + 1 < len(zeilen_histogramm) and zeilen_histogramm[unten + 1] >= min_pro_zeile:
        unten += 1
    while oben > 0 and zeilen_histogramm[oben - 1] >= min_pro_zeile:
        oben -= 1
    while rechts + 1 < len(spalten_histogramm) and spalten_histogramm[rechts + 1] >= min_pro_zeile:
        rechts += 1
    while links > 0 and spalten_histogramm[links - 1] >= min_pro_zeile:
        links -= 1
    return (links, oben, rechts + 1, unten + 1)
