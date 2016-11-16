#-*- coding:utf-8 -*-
"""
FileManagement
^^^^^^
Stellt funktionen zur Behandlung von Datein bereit.

30.06.14: Marc:
Erstellt und Funktionen aus dem bin/vision Skript extrahiert um sie auch im Kameramodul zu benutzen.

"""
import gzip
import json
import numpy
import os
from bitbots.robot.pypose import PyPose as Pose
from bitbots.util.pydatavector import PyDataVector
import glob

def load_folder_filenames(path):
    filelist = []
    decs = os.listdir(path)
    for dec in decs:
        if ".gz" in dec:
            filelist.append(path + dec)
    images = load_filenames(filelist)
    return images

def load_filenames(images):
    files = []
    for name in images:
        if not os.path.exists(name):
            continue

        if os.path.isdir(name):
            images = sorted(glob.glob(os.path.join(name, "*.gz")))
            files.extend(images)
        else:
            files.append(name)
    return files

def read_yuyv_file(name):
    sizes = (
        (800, 1280 * 2),
        (720, 1280 * 2),
        (480, 640 * 2),
        (600, 800 * 2)
    )

    def estimate_size_for_data(data):
        for height, width in sizes:
            if len(data) == width * height:
                return (height, width)

        raise ValueError("Keine Bildgröße für %d Pixel" % (len(data) / 2))

    oo = gzip.GzipFile if name.endswith(".gz") else open
    with oo(name) as fp:
        data = fp.read()

    size = estimate_size_for_data(data)
    return numpy.fromstring(data, dtype=numpy.uint8).reshape(size)

def get_yuyv_image_size(image):
        height, width = image.shape
        return (width / 2, height)

def read_pose_file(base, default=None):
    names = (base + ".json", base[:-3] + ".json")
    for name in names:
        if not os.path.exists(name):
            continue

        with open(name) as fp:
            pose = Pose()
            json_content = json.load(fp)
            if isinstance(json_content, dict):
                pose.positions = [(k.encode("utf8"), v) for k, v in json_content["positions"]]
                angle = PyDataVector(*json_content["robot_angle"])
            else:
                pose.positions = [(k.encode("utf8"), v) for k, v in json_content]
                angle=PyDataVector(0,0,0)
            return (pose, angle)

    if default is None:
        raise IOError("Pose nicht gefunden: %s" % base)

    return default
