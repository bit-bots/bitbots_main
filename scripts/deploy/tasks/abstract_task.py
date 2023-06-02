import abc

from fabric import Group, Result

class AbstractTask(abc.ABC):
    def __init__(self) -> None:
        """
        Abstract task class that all tasks should inherit from.
        """
        pass

    @abc.abstractmethod
    def run(self, connections: Group) -> Result:
        """
        Abstract method that should be implemented by all tasks.

        :param connections: The connections to remote servers.
        :return: The result of the task.
        """
        pass
