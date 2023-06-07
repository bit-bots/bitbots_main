import abc

from fabric import Group, GroupResult

from misc import *

class AbstractTask(abc.ABC):
    def __init__(self) -> None:
        """
        Abstract task class that all tasks should inherit from.
        """
        self._show_status = True

    def run(self, task_prefix: str, connections: Group) -> GroupResult:
        """
        Abstract method that should be implemented by all tasks.

        :param task_prefix: The prefix for status display.
        :param connections: The connections to remote servers.
        :return: The results of the task.
        """
        if self._show_status:
            with CONSOLE.status(f"{task_prefix} {self.__class__.__name__}"):
                return self._run(connections)
        else:
            return self._run(connections)

    @abc.abstractmethod
    def _run(self, connections: Group) -> GroupResult:
        """
        Abstract method that should be implemented by all tasks.

        :param connections: The connections to remote servers.
        :return: The results of the task.
        """
        pass

    def _resutls_hosts(self, results) -> list[str]:
        """
        Get a list of hosts that have results.

        :param results: The results of the task.
        :return: The list of hosts that have results.
        """
        return [connection.host for connection in results.keys()]

    def _succeded_hosts(self, results: GroupResult) -> list[str]:
        """
        Get a list of hosts that succeeded.

        :param results: The results of the task.
        :return: The list of hosts that succeeded.
        """
        return self._resutls_hosts(results.succeeded)

    def _failed_hosts(self, results: GroupResult) -> list[str]:
        """
        Get a list of hosts that failed.

        :param results: The results of the task.
        :return: The list of hosts that failed.
        """
        return self._resutls_hosts(results.failed)
