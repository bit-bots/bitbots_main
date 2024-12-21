from pathlib import Path

from ament_mypy.main import main

import pytest


@pytest.mark.mypy
def test_mypy():
    rc = main(argv=["--config", str((Path(__file__).parent / "mypy.ini").resolve())])
    assert rc == 0, 'Found code style errors / warnings'
