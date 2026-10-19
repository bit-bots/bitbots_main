"""Darstellungen für die Lernschritte; die Erkennungsregeln stehen im Notebook."""

import cv2
import matplotlib.pyplot as plt
import numpy as np
from werkzeuge import draw_overlay


def show_image(image_bgr, title="Kamerabild"):
    """Zeige Bildkoordinaten und wandle nur für die Anzeige nach RGB um."""
    figure, axis = plt.subplots(figsize=(9, 5), layout="constrained")
    axis.imshow(cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB))
    axis.set(title=title, xlabel="x → nach rechts", ylabel="y → nach unten")
    return figure


def show_boxes(record, boxes):
    """Vergleiche die vorgegebenen Rechtecke mit den eigenen Rechtecken."""
    return show_image(draw_overlay(record, boxes), "Vorgegeben: grün · Eigene Boxen: magenta")


def show_mask(image_bgr, mask):
    """Zeige genau die vom Notebook übergebene Maske neben dem Bild."""
    figure, axes = plt.subplots(1, 2, figsize=(11, 4), layout="constrained")
    axes[0].imshow(cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB))
    axes[0].set_title("Kamerabild")
    axes[1].imshow(mask, cmap="gray", vmin=0, vmax=255)
    axes[1].set_title("Deine Maske: hell = ausgewählt")
    for axis in axes:
        axis.axis("off")
    return figure


def show_channels(image_bgr):
    """Zeige die Farbanteile mit derselben Helligkeitsskala."""
    rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    figure, axes = plt.subplots(1, 3, figsize=(12, 4), layout="constrained")
    for channel, (axis, title) in enumerate(zip(axes, ("Rot", "Grün", "Blau"), strict=True)):
        axis.imshow(rgb[:, :, channel], cmap="gray", vmin=0, vmax=255)
        axis.set_title(f"{title}: hell = großer Anteil")
        axis.axis("off")
    return figure


def show_hsv_cake():
    """Zeige HSV-Farbscheiben bei unterschiedlichem Value in OpenCV-Koordinaten."""
    coordinates = np.linspace(-1, 1, 401)
    x, y = np.meshgrid(coordinates, coordinates)
    radius = np.hypot(x, y)
    angle = np.mod(np.arctan2(y, x), 2 * np.pi)
    hsv = np.zeros((*radius.shape, 3), dtype=np.uint8)
    hsv[:, :, 0] = np.minimum(angle / (2 * np.pi) * 180, 179).astype(np.uint8)
    hsv[:, :, 1] = (np.minimum(radius, 1) * 255).astype(np.uint8)
    figure, axes = plt.subplots(1, 3, figsize=(13, 5), layout="constrained")
    for axis, value in zip(axes, (255, 160, 80), strict=True):
        hsv[:, :, 2] = value
        rgb = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
        rgb[radius > 1] = 255
        axis.imshow(rgb, origin="lower", extent=(-1, 1, -1, 1))
        for hue in range(0, 180, 30):
            theta = hue / 180 * 2 * np.pi
            axis.text(1.16 * np.cos(theta), 1.16 * np.sin(theta), f"H={hue}", ha="center", va="center")
        axis.set(xlim=(-1.45, 1.45), ylim=(-1.35, 1.35), title=f"Helligkeit V={value}")
        axis.axis("off")
    figure.suptitle("HSV-Torte: H rundherum · S von innen nach außen\nMitte: S=0 · Rand: S=255")
    return figure
