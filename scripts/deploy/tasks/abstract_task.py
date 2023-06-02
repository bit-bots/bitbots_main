import abc

from fabric import Connection, Result

class AbstractTask(abc.ABC):
    def __init__(self) -> None:
        """
        Abstract task class that all tasks should inherit from.
        """
        pass

    @abc.abstractmethod
    def run(self, connection: Connection, server_identifier: str) -> Result:
        """
        Abstract method that should be implemented by all tasks.

        :param connection: The connection to the remote server.
        :param server_identifier: The identifier of the remote server.
        :return: The result of the task.
        """
        pass
