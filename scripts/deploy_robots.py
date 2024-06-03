#!/usr/bin/env python3
import sys

from deploy.deploy_robots import DeployRobots
from deploy.misc import print_error

if __name__ == "__main__":
    try:
        DeployRobots()
    except KeyboardInterrupt:
        print_error("Interrupted by user")
        sys.exit(1)
