#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import asyncore, socket, rospy
from bitbots_live_tool_rqt.msg import LiveMessage

class Client(asyncore.dispatcher):
    """Client class that receives the UDP messages and provides the data to our process_data method of class HardwareUI"""

    def __init__(self, ip, port, parent=None):
        """Initializes a new thread that listens for UDP messages"""
        asyncore.dispatcher.__init__(self)
        self.create_socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.bind((ip, int(port)))


        self.pub = rospy.Publisher('live_info', LiveMessage, queue_size=1)

        self.run()

    def run(self):
        """Method needed to keeep asyncore running"""
        while not rospy.is_shutdown():
            asyncore.loop(count=1)



    def handle_read(self):
        """Receives the messages and emits a signal containing the data"""
        data = self.recv(8192)
        msg = LiveMessage()
        msg.data = data
        self.pub.publish(msg)


    def handle_connect(self):
        request_handler(self)

    def handle_close(self):
        self.close()


if __name__ =="__main__":
    rospy.init_node('live_info', anonymous=True)
    client = Client(ip="127.0.0.1", port=5006)