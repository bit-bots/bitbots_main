#!/usr/bin/env python3
from deploy.deploy_robots import DeployRobots
from deploy.misc import print_err

if __name__ == "__main__":
    try:
        DeployRobots()
    except KeyboardInterrupt:
        print_err("Interrupted by user")
        exit(1)
