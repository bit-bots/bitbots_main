import yaml

from os.path import join
from typing import Dict, List, Optional


class ModelConfig:
    def __init__(self, config: Optional[Dict] = None):
        self._config: Dict = config if config else {}

    def get_detection_classes(self) -> List[str]:
        return self._config['detection']['classes']

    def get_segmentation_classes(self) -> List[str]:
        return self._config['segmentation']['classes']

    def team_colors_are_provided(self) -> bool:
        return self._config['detection'].get('team_colors', False)

    def get_robot_class_ids(self) -> List[int]:
        ids = []
        for i, c in enumerate(self.get_detection_classes()):
            if "robot" in c:
                ids.append(i)
        return ids


class ModelConfigLoader:
    @staticmethod
    def load_from(model_path: str) -> ModelConfig:
        with open(join(model_path, "model_config.yaml"), 'r') as f:
            return ModelConfig(yaml.safe_load(f))
