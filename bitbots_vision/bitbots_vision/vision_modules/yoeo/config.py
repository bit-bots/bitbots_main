from typing import Optional, Dict, List
import toml


class Config:
    def __init__(self):
        self._config: Optional[Dict] = None

    def load(self, config_path: str) -> None:
        self._config = toml.load(config_path)

    def get_detection_classes(self) -> List[str]:
        return self._config['detection']['classes']

    def get_segmentation_classes(self) -> List[str]:
        return self._config['segmentation']['classes']

    def team_color_detection_supported(self) -> bool:
        return self._config['detection']['team_colors']

    def get_robot_classes(self) -> List[str]:
        robot_classes = []
        for c in self.get_detection_classes():
            if "robot" in c:
                robot_classes.append(c)

        return robot_classes
