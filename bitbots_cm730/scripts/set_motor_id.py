#!/usr/bin/env python3

from bitbots_cm730.lowlevel.controller.controller import Controller, get_mx28_register_table, get_cm730_register_table
from bitbots_cm730.lowlevel.serial import Serial
import time

ctrl = Controller(Serial('/dev/ttyUSB0'))

cm = get_cm730_register_table()
mx = get_mx28_register_table()

ctrl.write_register(200, cm.dxl_power, 1)

print("Ths Scripts sets the Motorid from old_id to new_id")
old_cid = int(input("The old ID:"))
new_cid = int(input("The new ID:"))

ctrl.write_register(old_cid, mx.cid, new_cid)

time.sleep(1)
ctrl.write_register(200, cm.dxl_power, 0)


