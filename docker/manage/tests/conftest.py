import sys
from pathlib import Path

# Add docker directory to sys.path so 'manage' package can be imported directly
DOCKER_DIR = Path(__file__).resolve().parents[2]
if str(DOCKER_DIR) not in sys.path:
    sys.path.insert(0, str(DOCKER_DIR))
