#!/usr/bin/env python
# -*- coding: utf-8 -*-

from bitbots_dynamixel_debug.connector import Connector
import sys
import time

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--p1", help="use old protocol version", action="store_true")
parser.add_argument("id")
args = parser.parse_args()


id = args.id
if args.p1:
    protocol = 1
else:
    protocol = 2
baudrate = 1000000
device ="/dev/ttyUSB0".encode('utf-8')

c = Connector(protocol, device, baudrate)

print("The disco will only stop if you hit ctrl+c")
try:
    while True:
        c.writeLED(id, True)
        time.sleep(0.25)
        c.writeLED(id, False)
        time.sleep(0.25)
except KeyboardInterrupt:
    c.closePort()

c.closePort()


