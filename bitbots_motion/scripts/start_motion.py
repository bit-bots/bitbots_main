#!/usr/bin/env python
# -*- coding:utf-8 -*-

import argparse
import sys
import rospy
from bitbots_motion.motion import Motion


def main():
    parser = argparse.ArgumentParser(description='Start the motion node')
    parser.add_argument('--no', dest='dieflag', action='store_false',
                        help='Supress the autmatical deactivating of the motion after some time without updates')
    parser.add_argument('--nostandup', dest='standup', action='store_false',
                        help='Surpress automatical stand up')
    parser.add_argument('--softoff', dest='soft', action='store_true',
                        help='Only deactivate motors when robot is not moving')
    parser.add_argument('--softstart', dest='softstart', action='store_true',
                        help='Direclty start in softoff')
    parser.add_argument('--starttest', dest='starttest', action='store_true',
                        help='Ping motors on startup')
    args = parser.parse_args()

    motion = Motion(dieflag=args.dieflag, standupflag=args.standup,
                    softoff_flag=args.soft, softstart=args.softstart,
                    start_test=args.starttest)


if __name__ == "__main__":
    main()
