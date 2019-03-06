#!/usr/bin/env python
# -*- coding: utf-8 -*-

from bitbots_dynamixel_debug.connector import Connector
from bitbots_dynamixel_debug.parser import parse
import sys

args = parse(id_req = True, data_req = True)

c = Connector(args['protocol'], args['device'], args['baudrate'])

max_id = args['id']
for i in range(max_id +1):
    c.write_baud(i, args['data'], doPrint=True)

c.closePort()