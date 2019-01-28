#!/usr/bin/env python
# -*- coding: utf-8 -*-

from bitbots_dynamixel_debug.connector import SingleConnector
from bitbots_dynamixel_debug.parser import parse
import sys

args = parse(id_req = True, data_req = True)

c = SingleConnector(args['protocol'], args['device'], args['baudrate'])

max_id = args['id']
for i in range(max_id):
    c.write_baud(i, args['data'], True)

c.closePort()