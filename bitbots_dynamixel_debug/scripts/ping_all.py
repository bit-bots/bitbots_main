#!/usr/bin/env python
# -*- coding: utf-8 -*-

from bitbots_dynamixel_debug.connector import Connector
import sys

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--p1", help="use old protocol version", action="store_true")
args = parser.parse_args()



if args.p1:
    protocol = 1
else:
    protocol = 2
baudrate = 2000000
device ="/dev/ttyACM0".encode('utf-8')

c = Connector(protocol, device, baudrate)

for i in range(1, 21):
	c.ping(i, doPrint=True)

c.closePort()


