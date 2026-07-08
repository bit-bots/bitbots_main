#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def welcome():
    """Print Bit-Bot on the terminal."""
    bl = BetterLaunch()
    art_file = bl.find("bitbots_utils", "welcome_art.txt", "config")
    bl.exec(["cat", art_file])
