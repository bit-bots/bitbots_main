"""Einstieg für die selbst erarbeitete Ballerkennung aus dem Notebook."""

import cv2
import numpy as np


def zu_hsv(bild: np.ndarray, *, bgr: bool = False) -> np.ndarray:
    """Dieselbe HSV-Umrechnung wie in werkzeuge.py, ohne Notebook-Abhängigkeiten."""
    conversion = cv2.COLOR_BGR2HSV_FULL if bgr else cv2.COLOR_RGB2HSV_FULL
    return cv2.cvtColor(np.asarray(bild, dtype=np.uint8), conversion)


def detect_balls(image_bgr: np.ndarray) -> tuple[int, int, int, int] | None:
    """Liefere eine Bounding Box (links, oben, rechts, unten) oder None."""
    # TODO: Eigene Funktion aus evaluation.ipynb einsetzen.
    return None
