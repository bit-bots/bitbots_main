#!/usr/bin/env python3

import os
from socket import gethostname

from bitbots_tts.tts import say
from netifaces import AF_INET, ifaddresses, interfaces

ip_address = "not set"

wifi_interface = next(filter(lambda i: i.startswith("wlp"), interfaces()), None)
if wifi_interface and AF_INET in ifaddresses(wifi_interface):
    ip_address = ifaddresses(wifi_interface)[AF_INET][0]["addr"]

ip = " - dot - ".join(ip_address.split("."))

robot_name = os.getenv("ROBOT_NAME") or gethostname()
msg = f"Startup complete: - {robot_name}. My wifi IP is {ip}."
say(msg)
