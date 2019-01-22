#!/usr/bin/env python
# -*- coding: utf-8 -*-

from bitbots_dynamixel_debug.connector import SingleConnector
from bitbots_dynamixel_debug.parser import parse
import sys

args = parse(id_req = True, register_req = False)

c = SingleConnector(args['protocol'], args['device'], args['baudrate'])

c.broadcast_ping(args['id'], doPrint=True)

c.closePort()


