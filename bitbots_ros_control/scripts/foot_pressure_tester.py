#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from bitbots_msgs.msg import FootPressure
import argparse
import random
import time
import tkinter
import threading

parser = argparse.ArgumentParser()

parser.add_argument("-r", "--rate", help="Publish rate", dest="rate", type=int, default=200)
parser.add_argument("-n", "--noise", help="Amount of noise on the signal", type=float, default=1e6)
parser.add_argument("-s", "--seed", help="random seed for zero and scale", dest="seed")

args = parser.parse_args()

if args.seed is not None:
    random.seed(args.seed)

zeroes = [2e8 * random.random() - 1e8 for i in range(8)]
scales = []
for i in range(8):
    if random.random() < 0.5:
        m = 1
    else:
        m = -1
    scales.append((random.random() * 2e7 + 1e7) * m)  # random value between 1e7 and 3e7 or -1e7 and -3e7

force_values = [0] * 8


# this is ugly but I dont have patience to make it properly


def update_force_0(force):
    force_values[0] = float(force)


def update_force_1(force):
    force_values[1] = float(force)


def update_force_2(force):
    force_values[2] = float(force)


def update_force_3(force):
    force_values[3] = float(force)


def update_force_4(force):
    force_values[4] = float(force)


def update_force_5(force):
    force_values[5] = float(force)


def update_force_6(force):
    force_values[6] = float(force)


def update_force_7(force):
    force_values[7] = float(force)


force_functions = [update_force_0, update_force_1, update_force_2, update_force_3,
                   update_force_4, update_force_5, update_force_6, update_force_7]
master = tkinter.Tk()
master.title = "Foot Pressure test gui"
labels = ["l_l_back", "l_l_front", "l_r_back", "l_r_front", "r_l_back", "r_l_front", "r_r_back", "r_r_front", ]
scalers = []
for i in range(8):
    scalers.append(tkinter.Scale(master,
                                 from_=-0.2,
                                 to=10,
                                 orient=tkinter.HORIZONTAL,
                                 resolution=0.05,
                                 label=labels[i],
                                 length=300,
                                 width=30,
                                 command=force_functions[i]))
    scalers[i].pack()


def zero():
    for s in scalers:
        s.set(0.0)


b = tkinter.Button(master, command=zero, text="Zero")
b.pack()

rclpy.init(args=None)
pub_r = self.create_publisher(FootPressure, "/foot_pressure_right/raw", 1, tcp_nodelay=True)
pub_l = self.create_publisher(FootPressure, "/foot_pressure_left/raw", 1, tcp_nodelay=True)

rate = self.create_rate(args.rate)
msg_l = FootPressure()
msg_r = FootPressure()


def publish(timer):
    msg_l.header.stamp = msg_r.header.stamp = rospy.get_rostime()
    msg_l.left_back = zeroes[0] + scales[0] * force_values[0] + random.random() * args.noise
    msg_l.left_front = zeroes[1] + scales[1] * force_values[1] + random.random() * args.noise
    msg_l.right_back = zeroes[2] + scales[2] * force_values[2] + random.random() * args.noise
    msg_l.right_front = zeroes[3] + scales[3] * force_values[3] + random.random() * args.noise

    msg_r.left_back = zeroes[4] + scales[4] * force_values[4] + random.random() * args.noise
    msg_r.left_front = zeroes[5] + scales[5] * force_values[5] + random.random() * args.noise
    msg_r.right_back = zeroes[6] + scales[6] * force_values[6] + random.random() * args.noise
    msg_r.right_front = zeroes[7] + scales[7] * force_values[7] + random.random() * args.noise
    pub_l.publish(msg_l)
    pub_r.publish(msg_r)


rospy.Timer(Duration(seconds=1) / args.rate, publish)
tkinter.mainloop()
rospy.signal_shutdown("gui closed")
