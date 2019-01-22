#!/usr/bin/env python
# -*- coding: utf-8 -*-

from bitbots_dynamixel_debug.connector import SingleConnector
from bitbots_dynamixel_debug.parser import parse
import sys

args = parse(id_req = True, register_req = True, data_req = True)

c = SingleConnector(args['protocol'], args['device'], args['baudrate'])

c.write_1(args['id'], args['reg'], args['data'], doPrint=True)


c.closePort()


