#!/usr/bin/env python
# -*- coding: utf-8 -*-

from bitbots_dynamixel_debug.connector import Connector
import sys

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--p1", help="use old protocol version", action="store_true")
parser.add_argument("id")
parser.add_argument("position")
args = parser.parse_args()


id = int(args.id)
position = args.position
if args.p1:
    protocol = 1
else:
    protocol = 2
baudrate = 2000000
device ="/dev/ttyUSB0".encode('utf-8')

c = Connector(protocol, device, baudrate)

c.writeTorque(id, True)
c.writePosition(id, position, doPrint=True)
c.writeTorque(id, False)

c.closePort()


