#!/usr/bin/env python

# Part of the code was taken from orginal author: Ryu Woon Jung (Leon)
# original code can be found at: https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/master/python

import os, ctypes

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


class Connector(object):
    
    def __init__(self, protocol, device, baudrate):
        self.protocol = protocol
        self.device = device
        self.baudrate = baudrate

        # Initialize PortHandler Structs
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.port_num = dynamixel.portHandler(self.device)

        # Initialize PacketHandler Structs
        dynamixel.packetHandler()

        self.dxl_comm_result = COMM_TX_FAIL                              # Communication result

        # Open port
        if dynamixel.openPort(self.port_num):
            print("Succeeded to open the port " + self.device)
        else:
            print("Failed to open the port "  + self.device)
            print("Press any key to terminate...")
            getch()
            quit()

        # Set port baudrate
        if dynamixel.setBaudRate(self.port_num, self.baudrate):
            print("Succeeded to change the baudrate to " + self.baudrate)
        else:
            print("Failed to change the baudrate to " + self.baudrate)
            print("Press any key to terminate...")
            getch()
            quit()

    def closePort(self):
        dynamixel.closePort(self.port_num)

    def ping(self, id, doPrint=False):
        dxl_model_number = dynamixel.pingGetModelNum(self.port_num, self.protocol, id)
        dxl_comm_result = dynamixel.getLastTxRxResult(self.port_num, self.protocol)
        dxl_error = dynamixel.getLastRxPacketError(self.port_num, self.protocol)
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
    
    def broadcast_ping(self, maxId, doPrint=False):
        # Try to broadcast ping the Dynamixel
        dynamixel.broadcastPing(self.port_num, self.protocol)
        dxl_comm_result = dynamixel.getLastTxRxResult(self.port_num, self.protocol)
        if dxl_comm_result != COMM_SUCCESS:
            print(dynamixel.getTxRxResult(self.protocol, dxl_comm_result))

        print("Detected Dynamixel : ")
        for id in range(0, maxId):
            if ctypes.c_ubyte(dynamixel.getBroadcastPingResult(self.port_num, self.protocol, id)).value:
                print("[ID:%03d]" % (id))

    def reboot(self, id):
        print("The LED should flicker")
        
        dynamixel.reboot(self.port_num, self.protocol, id)
        dxl_comm_result = dynamixel.getLastTxRxResult(self.port_num, self.protocol)
        dxl_error = dynamixel.getLastRxPacketError(self.port_num, self.protocol)
        if dxl_comm_result != COMM_SUCCESS:
            print(dynamixel.getTxRxResult(self.protocol, dxl_comm_result))
            return False
        elif dxl_error != 0:
            print(dynamixel.getRxPacketError(self.protocol, dxl_error))
            return False

        print("[ID:%03d] reboot Succeeded" % (id))
        return True

    def writeTorque(self, id, enable, doPrint=False):
        dynamixel.write1ByteTxRx(self.port_num, self.protocol, id, 64, enable)
        dxl_comm_result = dynamixel.getLastTxRxResult(self.port_num, self.protocol)
        dxl_error = dynamixel.getLastRxPacketError(self.port_num, self.protocol)
        if dxl_comm_result != COMM_SUCCESS:
            print(dynamixel.getTxRxResult(self.protocol, dxl_comm_result))
            return False
        elif dxl_error != 0:
            print(dynamixel.getRxPacketError(self.protocol, dxl_error))        
            return False
        return True

    def writeGoalPosition(self, id, position):
        dynamixel.write4ByteTxRx(self.port_num, self.protocol, id, 116, position)
        dxl_comm_result = dynamixel.getLastTxRxResult(self.port_num, self.protocol)
        dxl_error = dynamixel.getLastRxPacketError(self.port_num, self.protocol)
        if dxl_comm_result != COMM_SUCCESS:
            print(dynamixel.getTxRxResult(self.protocol, dxl_comm_result))
            return False
        elif dxl_error != 0:
            print(dynamixel.getRxPacketError(self.protocol, dxl_error))
            return False
        return True

    def readGoalPosition(self, id, doPrint=False):
        dxl_present_position = dynamixel.read4ByteTxRx(self.port_num, self.protocol, id, 132)
        dxl_comm_result = dynamixel.getLastTxRxResult(self.port_num, self.protocol)
        dxl_error = dynamixel.getLastRxPacketError(self.port_num, self.protocol)
        if dxl_comm_result != COMM_SUCCESS:
            print(dynamixel.getTxRxResult(self.protocol, dxl_comm_result))            
        elif dxl_error != 0:
            print(dynamixel.getRxPacketError(self.protocol, dxl_error))

        if doPrint:
            print("[ID:%03d] PresPos:%03d" % (id, dxl_present_position))
        return dxl_present_position

    def writeLED(self, id, enable):
        dynamixel.write1ByteTxRx(self.port_num, self.protocol, id, 65, enable)
        dxl_comm_result = dynamixel.getLastTxRxResult(self.port_num, self.protocol)
        dxl_error = dynamixel.getLastRxPacketError(self.port_num, self.protocol)
        if dxl_comm_result != COMM_SUCCESS:
            print(dynamixel.getTxRxResult(self.protocol, dxl_comm_result))
            return False
        elif dxl_error != 0:
            print(dynamixel.getRxPacketError(self.protocol, dxl_error))
            return False
        return True