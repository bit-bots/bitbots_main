"""Regenerate distance maps from saved generator inputs.

Run through Pixi with the soccer_field_map_generator package on PYTHONPATH.
Legacy fields without generator inputs retain their existing map geometry.
"""

from pathlib import Path

import cv2
import numpy as np
import yaml
from soccer_field_map_generator.generator import generate_map_image

ROOT = Path(__file__).resolve().parents[2] / "src/bitbots_misc/bitbots_parameter_blackboard/config/fields"


def main():
    for directory in sorted(ROOT.iterdir()):
        source = directory / "gui-config.yaml"
        if directory.name == "hsl_kid":
            source = directory / "hsl_kid.yaml"
        output = directory / "lines.png"
        if source.exists():
            parameters = yaml.safe_load(source.read_text())["parameters"]
            image = generate_map_image(parameters)
        elif output.exists():
            image = cv2.imread(str(output), cv2.IMREAD_UNCHANGED)
            if image.max() == 100:
                image = np.rint(image.astype(float) * 255 / 100).astype(np.uint8)
        else:
            continue
        if not cv2.imwrite(str(output), image):
            raise OSError(f"Could not write {output}")


if __name__ == "__main__":
    main()
