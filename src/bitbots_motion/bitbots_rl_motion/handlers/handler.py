from abc import ABC, abstractmethod


class Handler(ABC):
    @abstractmethod
    def has_data(self):
        pass
