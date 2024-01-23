#!/usr/bin/env python3
import os
import socket

import pyttsx3
from netifaces import AF_INET, ifaddresses, interfaces

ip_address = "not set"
for iface_name in interfaces():
    addresses = [i["addr"] for i in ifaddresses(iface_name).setdefault(AF_INET, [{"addr": "No IP addr"}])]
    if iface_name == "wlp3s0":
        ip_address = " ".join(addresses)

hostname = socket.gethostname()
ip_parts = ip_address.split(".")
ip = ip_parts[0] + " point " + ip_parts[1] + " point " + ip_parts[2] + " point " + ip_parts[3]

engine = pyttsx3.init()
engine.setProperty("rate", 175)
robot_name = os.getenv("ROBOT_NAME")
engine.say("Startup complete")
engine.runAndWait()
engine.setProperty("rate", 175)
engine.say(f"{robot_name}")
engine.say(f"My IP adress is {ip}")
engine.runAndWait()
