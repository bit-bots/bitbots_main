#!/usr/bin/env python
# -*- coding: utf-8 -*-

from bitbots_dynamixel_debug.connector import SingleConnector
from bitbots_dynamixel_debug.parser import parse
import sys

args = parse(id_req = True, register_req = False)

c = SingleConnector(args['protocol'], args['device'], args['baudrate'])

successful_pings = 0
error_pings = 0
numberPings = 100
for i in range(numberPings):
    sucess = c.broadcast_ping(args['id'], doPrint=True)
    if sucess:
        successful_pings += 1
    else:
        error_pings += 1

print("Servos 1 to " + str(args['id']) + " got pinged " + str(numberPings) + "times \n Successful pings: " + str(successful_pings) + "\n Error pings: " + str(error_pings))



c.closePort()


