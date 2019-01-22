#!/usr/bin/env python
# -*- coding: utf-8 -*-

from bitbots_dynamixel_debug.connector import SingleConnector
from bitbots_dynamixel_debug.parser import parse
import sys

args = parse(id_req = False, register_req = False)

print("Searching on baud 4000000")
c = SingleConnector(args['protocol'], args['device'], 4000000)
for i in range(0, 254):
	c.ping(i, doPrint=True)
c.closePort()

print("Searching on baud 2000000")
c = SingleConnector(args['protocol'], args['device'], 2000000)
for i in range(0, 254):
	c.ping(i, doPrint=True)
c.closePort()

print("Searching on baud 1000000")
c = SingleConnector(args['protocol'], args['device'], 1000000)
for i in range(0, 254):
	c.ping(i, doPrint=True)
c.closePort()

print("Searching on baud 57600")
c = SingleConnector(args['protocol'], args['device'], 57600)
for i in range(0, 254):
	c.ping(i, doPrint=True)
c.closePort()






