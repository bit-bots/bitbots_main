import csv
import os
import time

from ament_index_python import get_package_share_directory
from flask import Flask, render_template, request

template_dir = os.path.join(get_package_share_directory("bitbots_education"), "templates")
static_folder = os.path.join(get_package_share_directory("bitbots_education"), "static")
csv_file_path = os.path.realpath("logs.csv")

app = Flask(__name__, template_folder=template_dir, static_folder=static_folder)


def write_csv(vp, button):
    file_exists = os.path.isfile(csv_file_path)
    with open(csv_file_path, "a+", newline="") as file:
        writer = csv.writer(file)
        # Kopfzeile nur schreiben, wenn die Datei neu ist
        if not file_exists or os.stat("logs.csv").st_size == 0:
            writer.writerow(["VP", "Button", "Timestamp"])
        curr_time = time.time()
        writer.writerow([vp, button, curr_time])


def vp_track(button):
    username = request.cookies.get("vp_number")
    if username is not None:
        write_csv(username, button)


@app.route("/")
def index():
    vp_track("dashboard")
    return render_template("pages/dashboard.html", logging=bool(request.args.get("logging", False)))


@app.route("/imu")
def imu():
    vp_track("imu")
    return render_template("pages/imu.html")


@app.route("/vision")
def vision():
    vp_track("vision")
    return render_template("pages/vision.html")


@app.route("/motors")
def motors():
    vp_track("motors")
    return render_template("pages/motors.html")


@app.route("/behavior")
def behavior():
    vp_track("behavior")
    return render_template("pages/behavior.html")


def main():
    app.run(host="0.0.0.0", port=8000)
