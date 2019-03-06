#!/usr/bin/env python
# -*- coding: utf-8 -*-

from bitbots_dynamixel_debug.connector import Connector
import sys

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--on", help="activate cm730 motor power", action="store_true")
parser.add_argument("--off", help="deactivate cm730 motor power", action="store_true")
args = parser.parse_args()


protocol = 1
baudrate = 1000000
device ="/dev/ttyUSB0".encode('utf-8')

c = Connector(protocol, device, baudrate)

if args.on:
    c.cm730Power(True)
if args.off:
    c.cm730Power(False)
    
c.closePort()


