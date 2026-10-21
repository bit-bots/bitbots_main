#!/usr/bin/env python3
"""Convenience runner script for the dedicated Blockly server."""

import sys
from pathlib import Path

# Add package source directories to sys.path so it runs standalone with plain python3 or in ROS
_pkg_root = Path(__file__).resolve().parents[2]
_module_dir = _pkg_root / "bitbots_body_behavior"

for path_entry in (str(_module_dir), str(_pkg_root)):
    if path_entry not in sys.path:
        sys.path.insert(0, path_entry)

try:
    import blockly_server  # type: ignore[import-not-found]

    main = blockly_server.main
except (ImportError, ModuleNotFoundError):
    from bitbots_body_behavior.blockly_server import main

if __name__ == "__main__":
    main()
