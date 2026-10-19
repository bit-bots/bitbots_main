"""Einstieg für die selbst erarbeitete Ballerkennung aus dem Notebook."""

import cv2  # noqa: F401 - für den selbst ergänzten Detektor
import numpy as np


def detect_balls(image_bgr: np.ndarray) -> np.ndarray:
    """Liefere Ballboxen im Arrayformat der ROS-Anbindung."""
    # TODO: Eigene Erkennung aus evaluation.ipynb einsetzen.
    # Die Anleitung beschreibt die Umwandlung vom Notebook-Tupel zum ROS-Array.
    return np.empty((0, 4), dtype=float)
