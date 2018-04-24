#!/usr/bin/env python
# -*- coding: utf-8 -*-

from bitbots_dynamixel_debug.connector import Connector
import sys

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

successful_pings = 0
error_pings = 0
numberPings = 100
for i in range(numberPings):
    sucess = c.ping(id)
    if sucess:
        successful_pings += 1
    else:
        error_pings += 1

print("Servo " + str(id) + " got pinged " + str(numberPings) + "\n Successful pings: " + str(successful_pings) + "\n Error pings: " + str(error_pings))

c.closePort()


