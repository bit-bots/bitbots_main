#!/usr/bin/env python
# -*- coding: utf-8 -*-

from bitbots_dynamixel_debug.connector import Connector
from bitbots_dynamixel_debug.parser import parse
import sys
import time

args = parse(id_req = True, register_req = False)

c = Connector(args['protocol'], args['device'], args['baudrate'])

print("The disco will only stop if you hit ctrl+c")
try:
    while True:
        c.writeLED(args['id'], True)
        time.sleep(0.25)
        c.writeLED(args['id'], False)
        time.sleep(0.25)
except KeyboardInterrupt:
    c.closePort()

c.closePort()


