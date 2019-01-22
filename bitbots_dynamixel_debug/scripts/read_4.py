#!/usr/bin/env python
# -*- coding: utf-8 -*-

from bitbots_dynamixel_debug.connector import Connector
from bitbots_dynamixel_debug.parser import parse
import sys

args = parse(id_req = True, register_req = True)

c = Connector(args['protocol'], args['device'], args['baudrate'])

c.read_4(args['id'], args['reg'], doPrint=True)

c.closePort()


