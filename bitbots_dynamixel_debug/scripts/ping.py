#!/usr/bin/env python
# -*- coding: utf-8 -*-

from bitbots_dynamixel_debug.connector import Connector
from bitbots_dynamixel_debug.parser import parse
import sys

args = parse(id_req = True)

c = Connector(args['protocol'], args['device'], args['baudrate'])

c.ping(args['id'], doPrint=True)

c.closePort()


