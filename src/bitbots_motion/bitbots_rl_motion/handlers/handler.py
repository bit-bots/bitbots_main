from abc import ABC, abstractmethod

class Handler(ABC):
    def __init__(self, config):
        self._config = config

    @abstractmethod
    def has_data(self):
        pass
