"""Notebook-Vertrag: Nur der Funktionsbody muss ausgetauscht werden."""

import cv2
import numpy as np


def detect_balls(image_bgr: np.ndarray) -> np.ndarray:
    """Erkenne orange Bälle und liefere xyxy-Pixelboxen, ohne das Bild zu ändern."""
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([5, 100, 80]), np.array([30, 255, 255]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), dtype=np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boxes = []
    for contour in contours:
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        if area < 30 or perimeter == 0:
            continue
        if 4 * np.pi * area / perimeter**2 < 0.75:
            continue
        left, top, width, height = cv2.boundingRect(contour)
        if not 0.75 <= width / height <= 1.33:
            continue
        boxes.append([left, top, left + width, top + height])
    return np.asarray(boxes, dtype=np.float64).reshape(-1, 4)
