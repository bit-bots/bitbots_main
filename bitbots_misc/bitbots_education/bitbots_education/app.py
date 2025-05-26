import os
from flask import Flask, render_template
from ament_index_python import get_package_share_directory


template_dir = os.path.join(
    get_package_share_directory("bitbots_education"), "templates"
)
static_folder = os.path.join(
    get_package_share_directory("bitbots_education"), "static"
)

app = Flask(__name__, template_folder=template_dir, static_folder=static_folder)


@app.route("/")
def index():
    return render_template("pages/dashboard.html")

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
