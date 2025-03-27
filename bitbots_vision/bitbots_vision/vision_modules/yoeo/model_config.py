from os.path import join
from typing import Optional

import yaml  # type: ignore[import-untyped]


class ModelConfig:
    def __init__(self, config: Optional[dict] = None):
        self._config: dict = config if config else {}

    def get_detection_classes(self) -> list[str]:
        return self._config["detection"]["classes"]

    def get_segmentation_classes(self) -> list[str]:
        return self._config["segmentation"]["classes"]

    def team_colors_are_provided(self) -> bool:
        return self._config["detection"].get("team_colors", False)

    def get_robot_class_ids(self) -> list[int]:
        ids = []
        for i, c in enumerate(self.get_detection_classes()):
            if "robot" in c:
                ids.append(i)
        return ids


class ModelConfigLoader:
    @staticmethod
    def load_from(model_path: str) -> ModelConfig:
        with open(join(model_path, "model_config.yaml")) as f:
            return ModelConfig(yaml.safe_load(f))
