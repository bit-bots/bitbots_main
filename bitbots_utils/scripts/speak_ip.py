#!/usr/bin/env python3
#  -*- coding: utf8 -*-
from ipaddress import ip_address
import socket
from time import sleep
import pyttsx3
import os
from netifaces import interfaces, ifaddresses, AF_INET

ip_address = "not set"
for ifaceName in interfaces():
    addresses = [i['addr'] for i in ifaddresses(ifaceName).setdefault(AF_INET, [{'addr': 'No IP addr'}])]
    if ifaceName=="wlp3s0":
        ip_adress = ' '.join(addresses)

hostname = socket.gethostname()
IPAddr = socket.gethostbyname(hostname)
ip_parts = IPAddr.split(".")
ip = ip_parts[0] + " point " + ip_parts[1] + " point " + ip_parts[2] + " point " + ip_parts[3]

engine = pyttsx3.init()
engine.setProperty('rate', 175)
robot_name = os.getenv("ROBOT_NAME")
engine.say(f"Startup complete")
engine.runAndWait()
engine.setProperty('rate', 175)
engine.say(f"{robot_name}")
engine.say(f"My IP adress is {ip}")
engine.runAndWait()