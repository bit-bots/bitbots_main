#!/usr/bin/env python
# -*- coding: utf-8 -*-

from bitbots_dynamixel_debug.connector import Connector
import sys


import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--p1", help="use old protocol version", action="store_true")
parser.add_argument("maxId")
args = parser.parse_args()


maxid = args.maxId
if args.p1:
    protocol = 1
else:
    protocol = 2
protocol = 2
baudrate = 1000000
device ="/dev/ttyUSB0".encode('utf-8')

c = Connector(protocol, device, baudrate)

c.broadcast_ping(maxId, doPrint=True)

c.closePort()


