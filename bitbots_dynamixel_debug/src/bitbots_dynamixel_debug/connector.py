#!/usr/bin/env python

# Part of the code was taken from orginal author: Ryu Woon Jung (Leon)
# original code can be found at: https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/master/python

import os, ctypes
import time

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

# we do some ROS magic to find the path where the python file is which gives us the interface to the SDK
from rospkg import RosPack
rp = RosPack()
path = rp.get_path('dynamixel_sdk') + "/../python/dynamixel_functions_py"
print("Searching for DynamixelSDK Python Cariables in " + path)
os.sys.path.append(path)             # Path setting

import dynamixel_functions as dynamixel                     # Uses Dynamixel SDK library


COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed


class MultiConnector(object):
    def __init__(self, protocol, devices, baudrate):
        self.protocol = protocol
        self.devices = devices
        self.baudrate = baudrate


        # Initialize PortHandler Structs
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.port_num = []
        for device in self.devices:
            port = dynamixel.portHandler(device)            
            # remember port
            self.port_num.append(port)

        for i in range(0, len(self.devices)):
         # open port
            if dynamixel.openPort(self.port_num[i]):
                print("Succeeded to open the port " + str(self.devices[i]))
            else:
                print("Failed to open the port "  + str(self.devices[i]))
                print("Press any key to terminate...")
                getch()
                quit()

            # set baudrate
            if dynamixel.setBaudRate(self.port_num[i], self.baudrate):
                print("Succeeded to change the baudrate to " + str(self.baudrate))
            else:
                print("Failed to change the baudrate to " + str(self.baudrate))
                print("Press any key to terminate...")
                getch()
                quit()

        # Initialize PacketHandler Structs
        dynamixel.packetHandler()

    def reboot(self, port, id):
        print("The LED should flicker")
        
        dynamixel.reboot(self.port_num[port], self.protocol, id)
        dxl_comm_result = dynamixel.getLastTxRxResult(self.port_num[port], self.protocol)
        dxl_error = dynamixel.getLastRxPacketError(self.port_num[port], self.protocol)
        if dxl_comm_result != COMM_SUCCESS:
            print(dynamixel.getTxRxResult(self.protocol, dxl_comm_result))
            return False
        elif dxl_error != 0:
            print(dynamixel.getRxPacketError(self.protocol, dxl_error))
            return False

        print("[ID:%03d] reboot Succeeded" % (id))
        return True

    def ping(self, port, id, doPrint=False):
        dxl_model_number = dynamixel.pingGetModelNum(self.port_num[port], self.protocol, id)
        dxl_comm_result = dynamixel.getLastTxRxResult(self.port_num[port], self.protocol)
        dxl_error = dynamixel.getLastRxPacketError(self.port_num[port], self.protocol)
        if dxl_comm_result != COMM_SUCCESS:
            if doPrint:
                print(dynamixel.getTxRxResult(self.protocol, dxl_comm_result))
            return False
        elif dxl_error != 0:
            if doPrint:
                print(dynamixel.getRxPacketError(self.protocol, dxl_error))
            return False
        else:
            if doPrint:
                print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (id, dxl_model_number))
            return True

    def ping_loop(self, port, id, doPrint=False):
        successful_pings = 0
        error_pings = 0
        numberPings = 100
        for i in range(numberPings):
            sucess = self.ping(port, id)
            if sucess:
                successful_pings += 1
            else:
                error_pings += 1

        print("Servo " + str(id) + " got pinged " + str(numberPings) + "\n Successful pings: " + str(successful_pings) + "\n Error pings: " + str(error_pings))

    def broadcast_ping(self, port, maxId, doPrint=False):
        # Try to broadcast ping the Dynamixel
        dynamixel.broadcastPing(self.port_num[port], self.protocol)
        dxl_comm_result = dynamixel.getLastTxRxResult(self.port_num[port], self.protocol)
        if dxl_comm_result != COMM_SUCCESS:
            if doPrint:
                print(dynamixel.getTxRxResult(self.protocol, dxl_comm_result))
            return False

        if doPrint:
            print("Detected Dynamixel : ")
        nb_detected = 0
        for id in range(1, int(maxId)+1):
            if ctypes.c_ubyte(dynamixel.getBroadcastPingResult(self.port_num[port], self.protocol, id)).value:
                nb_detected += 1
                if doPrint:
                    print("[ID:%03d]" % (id))
        if nb_detected == maxId:
            return True

    def read_1(self, port, id, reg, doPrint=False):
        read_res = dynamixel.read1ByteTxRx(self.port_num[port], self.protocol, id, reg)
        dxl_comm_result = dynamixel.getLastTxRxResult(self.port_num[port], self.protocol)
        dxl_error = dynamixel.getLastRxPacketError(self.port_num[port], self.protocol)
        if dxl_comm_result != COMM_SUCCESS:
            print(dynamixel.getTxRxResult(self.protocol, dxl_comm_result))            
        elif dxl_error != 0:
            print(dynamixel.getRxPacketError(self.protocol, dxl_error))

        if doPrint:
            print("[ID:%03d] Regist %03d: %03d" % (id, reg, read_res))
        return read_res

    def read_4(self, port, id, reg, doPrint=False):
        read_res = dynamixel.read4ByteTxRx(self.port_num[port], self.protocol, id, reg)
        dxl_comm_result = dynamixel.getLastTxRxResult(self.port_num[port], self.protocol)
        dxl_error = dynamixel.getLastRxPacketError(self.port_num[port], self.protocol)
        if dxl_comm_result != COMM_SUCCESS:
            print(dynamixel.getTxRxResult(self.protocol, dxl_comm_result))            
        elif dxl_error != 0:
            print(dynamixel.getRxPacketError(self.protocol, dxl_error))

        if doPrint:
            print("[ID:%03d] Regist %03d: %03d" % (id, reg, read_res))
        return read_res

    def write_1(self, port, id, reg, value, doPrint=False):
        dynamixel.write1ByteTxRx(self.port_num[port], self.protocol, id, reg, value)
        dxl_comm_result = dynamixel.getLastTxRxResult(self.port_num[port], self.protocol)
        dxl_error = dynamixel.getLastRxPacketError(self.port_num[port], self.protocol)
        if dxl_comm_result != COMM_SUCCESS:
            print(dynamixel.getTxRxResult(self.protocol, dxl_comm_result))
            return False
        elif dxl_error != 0:
            print(dynamixel.getRxPacketError(self.protocol, dxl_error))
            return False
        return True

    def write_4(self, port, id, reg, value, doPrint=False):
        dynamixel.write4ByteTxRx(self.port_num[port], self.protocol, id, reg, value)
        dxl_comm_result = dynamixel.getLastTxRxResult(self.port_num[port], self.protocol)
        dxl_error = dynamixel.getLastRxPacketError(self.port_num[port], self.protocol)
        if dxl_comm_result != COMM_SUCCESS:
            print(dynamixel.getTxRxResult(self.protocol, dxl_comm_result))
            return False
        elif dxl_error != 0:
            print(dynamixel.getRxPacketError(self.protocol, dxl_error))
            return False
        return True

    def write_id(self, port, id, new_id, doPrint=False):
        self.write_1(port, id, 7, new_id, doPrint)
    
    def writeTorque(self, port, id, enable, doPrint=False):
        if enable:
            self.write_1(port, id, 64, 1)
        else:
            self.write_1(port, id, 64, 0)

    def write_baud(self, port, id, baud, doPrint=False):        
        if baud == 9600:
            val = 0
        elif baud == 57600:
            val = 1
        elif baud == 115200:
            val = 2
        elif baud == 1000000:
            val = 3
        elif baud == 2000000:
            val = 4
        elif baud == 3000000:
            val = 5
        elif baud == 4000000:
            val = 6
        elif baud == 4500000:
            val = 7
        else:
            print("Baud rate %d is not possible" % (baud))
            return
        self.write_1(port, id, 8, val, doPrint)

    def remove_return_delay_time(self, port, id, doPrint=False):
        self.write_1(port, id, 9, 0)
    
    def writeGoalPosition(self, port, id, position):
        self.write_4(port, id, 116, position)

    def writeLED(self, port, id, enable):
        if enable:
            self.write_1(port, id, 65, 1)
        else:
            self.write_1(port, id, 65, 0)

    def sync_read(self, port, ids, reg, length, doPrint=False):
        groupread_num = dynamixel.groupSyncRead(self.port_num[port], self.protocol, reg, length)
        for dxl_id in ids:
            # Add parameter storage for Dynamixel#1 present position value
            dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncReadAddParam(groupread_num, dxl_id)).value
            if dxl_addparam_result != 1:
                print("[ID:%03d] groupSyncRead addparam failed" % (dxl_id))
                return False

        # Syncread present position
        dynamixel.groupSyncReadTxRxPacket(groupread_num)
        dxl_comm_result = dynamixel.getLastTxRxResult(self.port_num[port], self.protocol)
        if dxl_comm_result != COMM_SUCCESS:
            print(dynamixel.getTxRxResult(self.protocol, dxl_comm_result))

        for dxl_id in ids:
            # Check if groupsyncread data of Dynamixels is available
            dxl_getdata_result = ctypes.c_ubyte(dynamixel.groupSyncReadIsAvailable(groupread_num, dxl_id, reg, length)).value
            if dxl_getdata_result != 1:
                print("[ID:%03d] groupSyncRead getdata failed" % (dxl_id))
                return False

        for dxl_id in ids:
            dxl_result = dynamixel.groupSyncReadGetData(groupread_num, dxl_id, reg, length)
            print("[ID:%03d] result is %d" % (dxl_id, dxl_result))

        return True
    
    def sync_read_loop(self, port, ids, reg, length, doPrint=False):
        successful_reads = 0
        error_reads = 0
        number_reads = 100
        for i in range(0,number_reads):
            if self.sync_read(port, ids, reg, length, doPrint):
                successful_reads +=1
            else:
                error_reads += 1
        time.sleep(1)
        print("Bus " + str(port) + " got sync read " + str(number_reads) + "\n Successful reads: " + str(successful_reads) + "\n Error reads: " + str(error_reads))        

    def closePort(self):
        for port in self.port_num:
            dynamixel.closePort(port)

class SingleConnector(MultiConnector):
    
    def __init__(self, protocol, device, baudrate):
        MultiConnector.__init__(self, protocol, [device], baudrate)        

    def ping(self, id, doPrint=False):
        super(SingleConnector, self).ping(0, id, doPrint)

    def ping_loop(self, id, doPrint=False):
        super(SingleConnector, self).ping_loop(0, id, doPrint)

    def broadcast_ping(self, maxId, doPrint=False):
        super(SingleConnector, self).broadcast_ping(0, maxId, doPrint)

    def reboot(self, id):
        super(SingleConnector, self).reboot(0, id)

    def read_1_s(self, id, reg, doPrint=False):
        super(SingleConnector, self).read_1(0, id, reg, doPrint)

    def read_4_s(self, id, reg, doPrint=False):
        super(SingleConnector, self).read_4(0, id, reg, doPrint)

    def write_1_s(self, id, reg, data, doPrint=False):
        super(SingleConnector, self).write_1(0, id, reg, data, doPrint)

    def write_id(self, id, new_id, doPrint=False):
        super(SingleConnector, self).write_id(0, id, new_id, doPrint)

    def writeTorque(self, id, enable, doPrint=False):
        super(SingleConnector, self).writeTorque(0, id, enable, doPrint)

    def write_baud(self, id, baud, doPrint=False):
        super(SingleConnector, self).write_baud(0, id, baud, doPrint)

    def remove_return_delay_time(self, id, doPrint=False):
        super(SingleConnector, self).remove_return_delay_time(0, id, doPrint)

    def writeGoalPosition(self, id, position):
        super(SingleConnector, self).writeGoalPosition(0, id, position)

    def writeLED(self, id, enable):
        super(SingleConnector, self).writeLED(0, id, enable)
