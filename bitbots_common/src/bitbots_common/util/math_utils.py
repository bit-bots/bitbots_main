"""
math
^^^^

Hier sind einige Mathematische Hilfsfunktionen gesammelt
"""
from math import sin, cos, sqrt, atan2, degrees
import math


def convert_uv2polar(u, v):
    """
    Konvertiert unsere lokalen u,v Werte in Polarkordinaten, wobei
    distance=radius und angel=teta
    """
    distance = sqrt(u ** 2 + v ** 2)
    angel = atan2(v, u)
    return distance, angel


def convert_polar2uv(distance, angel):
    """
    Konvertiert eine polarkoordinate zu in u und v
    Siehe auch :func:`convert_uv2polar`
    """
    v = sin(angel) * distance
    u = cos(angel) * distance
    return u, v


def convert_uv2angular(u, v):
    """
    Konvertiert u und v zu einem Winkel
    :rtype float
    :return Winkel zu uv objekt in Degree
    """
    if u != 0:
        winkel = degrees(math.atan2(v, u))
    else:
        winkel = 0
    return winkel


def convert_uv2distance(u, v):
    """
    Berechnet den abstand zu einem u-v punkt
    :param u: x1 achse abstand
    :param v: x2 achse abstand
    :return: euklidische distanz
    """
    return sqrt(u**2 + v**2)