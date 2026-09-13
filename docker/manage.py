#!/usr/bin/env python3
import sys
from pathlib import Path

# Add docker directory to sys.path if not present so manage package can be imported
DOCKER_DIR = Path(__file__).resolve().parent
if str(DOCKER_DIR) not in sys.path:
    sys.path.insert(0, str(DOCKER_DIR))

from manage.manager import ContainerManager  # noqa: E402
from manage.misc import print_error  # noqa: E402

if __name__ == "__main__":
    try:
        ContainerManager()
    except KeyboardInterrupt:
        print_error("Interrupted by user")
        sys.exit(1)
