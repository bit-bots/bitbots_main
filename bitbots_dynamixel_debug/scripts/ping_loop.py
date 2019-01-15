#!/usr/bin/env python
# -*- coding: utf-8 -*-

from bitbots_dynamixel_debug.connector import Connector
from bitbots_dynamixel_debug.parser import parse
import sys

args = parse(id_req = True, register_req = False)

c = Connector(args['protocol'], args['device'], args['baudrate'])

successful_pings = 0
error_pings = 0
numberPings = 100
for i in range(numberPings):
    sucess = c.ping(args['id'])
    if sucess:
        successful_pings += 1
    else:
        error_pings += 1

print("Servo " + str(args['id']) + " got pinged " + str(numberPings) + "\n Successful pings: " + str(successful_pings) + "\n Error pings: " + str(error_pings))

c.closePort()


