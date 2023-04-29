from __future__ import annotations

import yaml

from os.path import join
from typing import Dict, List


class ModelConfig:
    def __init__(self):
        self._config: Dict = {}

    def load(self, model_path: str) -> None:
        with open(join(model_path, "model_config.yaml"), 'r') as f:
            self._config = yaml.safe_load(f)

    def get_detection_classes(self) -> List[str]:
        return self._config['detection']['classes']

    def get_segmentation_classes(self) -> List[str]:
        return self._config['segmentation']['classes']

    def team_color_detection_supported(self) -> bool:
        return self._config['detection']['team_colors'] if 'team_colors' in self._config['detection'].keys() else False

    def get_robot_classes(self) -> List[str]:
        robot_classes = []
        for c in self.get_detection_classes():
            if "robot" in c:
                robot_classes.append(c)

        return robot_classes
