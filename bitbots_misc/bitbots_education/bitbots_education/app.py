import csv
import os
import time

from ament_index_python import get_package_share_directory
from flask import Flask, render_template, request

template_dir = os.path.join(get_package_share_directory("bitbots_education"), "templates")
static_folder = os.path.join(get_package_share_directory("bitbots_education"), "static")

app = Flask(__name__, template_folder=template_dir, static_folder=static_folder)


def write_csv(vp, button):
    file_exists = os.path.isfile("logs.csv")

    with open("logs.csv", "a", newline="") as file:
        writer = csv.writer(file)

        # Kopfzeile nur schreiben, wenn die Datei neu ist
        if not file_exists or os.stat("logs.csv").st_size == 0:
            writer.writerow(["VP", "Button", "Timestamp"])
        curr_time = time.strftime("%H:%M:%S", time.localtime())  # TODO use unix timestamp
        writer.writerow([vp, button, curr_time])


@app.route("/")
def index():
    return render_template("pages/dashboard.html", logging=bool(request.args.get("logging", False)))


@app.route("/imu")
def imu():
    return render_template("pages/imu.html")


@app.route("/vision")
def vision():
    return render_template("pages/vision.html")


@app.route("/motors")
def motors():
    return render_template("pages/motors.html")


@app.route("/behavior")
def behavior():
    return render_template("pages/behavior.html")


def main():
    app.run(host="0.0.0.0", port=8000)
