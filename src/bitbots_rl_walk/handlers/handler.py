import yaml


class Handler:
    def __init__(self, config_file: str):
        self._config = self._load_config(self, config_file)

    def _load_config(self, path: str):
        with open(path) as f:
            return yaml.safe_load(f)

    def get_data(self):
        pass
