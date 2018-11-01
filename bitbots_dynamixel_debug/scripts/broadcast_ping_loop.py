#!/usr/bin/env python
# -*- coding: utf-8 -*-

from bitbots_dynamixel_debug.connector import Connector
import sys


import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--p1", help="use old protocol version", action="store_true")
parser.add_argument("maxId")
args = parser.parse_args()


maxId = int(args.maxId)
if args.p1:
    protocol = 1
else:
    protocol = 2
protocol = 2
baudrate = 2000000
device ="/dev/ttyUSB0".encode('utf-8')

c = Connector(protocol, device, baudrate)

successful_pings = 0
error_pings = 0
numberPings = 100
for i in range(numberPings):
    sucess = c.broadcast_ping(maxId, doPrint=True)
    if sucess:
        successful_pings += 1
    else:
        error_pings += 1

print("Servos 1 to " + str(maxId) + " got pinged " + str(numberPings) + "times \n Successful pings: " + str(successful_pings) + "\n Error pings: " + str(error_pings))



c.closePort()


