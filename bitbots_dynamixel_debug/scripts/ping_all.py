#!/usr/bin/env python
# -*- coding: utf-8 -*-

from bitbots_dynamixel_debug.connector import Connector
from bitbots_dynamixel_debug.parser import parse
import sys

args = parse(id_req = False, register_req = False)

c = Connector(args['protocol'], args['device'], args['baudrate'])

for i in range(1, 21):
	c.ping(i, doPrint=True)

c.closePort()


