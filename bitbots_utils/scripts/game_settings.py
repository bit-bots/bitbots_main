#!/usr/bin/env python3

import sys
import os

sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src"))

from bitbots_bringup import game_settings


if __name__ == '__main__':
    game_settings.main()
