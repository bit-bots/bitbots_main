import subprocess
#subprocess.Popen(["rosrun bitbots_dynamixel_debug ping.py", "1", "--port", "ttyUSB0"])
import os
#os.spawnl(os.P_NOWAIT, "rosrun bitbots_dynamixel_debug ping.py 1 --port ttyUSB0")
import multiprocessing
from bitbots_dynamixel_debug.connector import MultiConnector


c = MultiConnector(2, ["/dev/ttyUSB0".encode('utf-8'), "/dev/ttyUSB1".encode('utf-8'), "/dev/ttyUSB2".encode('utf-8'), "/dev/ttyUSB3".encode('utf-8')], 4615384)
jobs = []
for i in range(0,4):    
    p = multiprocessing.Process(target=c.sync_read, args=(i, [1,2,3,4], 2, 1, True))
    jobs.append(p)

for i in range(0,4):
    jobs[i].start()

c.closePort()