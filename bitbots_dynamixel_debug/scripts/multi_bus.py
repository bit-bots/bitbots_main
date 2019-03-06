import subprocess
#subprocess.Popen(["rosrun bitbots_dynamixel_debug ping.py", "1", "--port", "ttyUSB0"])
import os
#os.spawnl(os.P_NOWAIT, "rosrun bitbots_dynamixel_debug ping.py 1 --port ttyUSB0")
import multiprocessing
from bitbots_dynamixel_debug.connector import Connector


c = Connector(2, ["/dev/ttyUSB0".encode('utf-8'), "/dev/ttyUSB1".encode('utf-8'), "/dev/ttyUSB2".encode('utf-8'), "/dev/ttyUSB3".encode('utf-8')], 2000000)#4615384)
jobs = []


ids =[]
ids.append([1, 2, 3, 4, 5])
ids.append([6, 7, 8, 9, 10])
ids.append([11, 12, 13, 14, 15])
ids.append([16, 18, 19, 20])

def write_1_to_all(port, reg, val):
    for i in ids[port]:
        c.write_1(i, reg, val, port=i, doPrint=True)

for i in range(0,4):
    #p = multiprocessing.Process(target=c.sync_read_loop, args=(i, ids[i], 9, 1, True))
    p = multiprocessing.Process(target=write_1_to_all, args=(i, 68, 1))
    jobs.append(p)

for i in range(0,4):
    jobs[i].start()

c.closePort()