#!/usr/bin/env python3
import sys

from deploy.deploy_game import DeployGame
from deploy.misc import print_error

if __name__ == "__main__":
    try:
        DeployGame()
    except KeyboardInterrupt:
        print_error("Interrupted by user")
        sys.exit(1)
