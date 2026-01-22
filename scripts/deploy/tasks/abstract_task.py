import abc

from deploy.misc import CONSOLE
from fabric import Group, GroupResult


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
            with CONSOLE.status(f"{task_prefix}", spinner="point"):
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

    def _results_hosts(self, results) -> list[str]:
        """
        Get a list of hosts that have results.

        :param results: The results of the task.
        :return: The list of hosts that have results.
        """
        return [connection.original_host for connection in results.keys()]

    def _succeeded_hosts(self, results: GroupResult) -> list[str]:
        """
        Get a list of hosts that succeeded.

        :param results: The results of the task.
        :return: The list of hosts that succeeded.
        """
        return self._results_hosts(results.succeeded)

    def _failed_hosts(self, results: GroupResult) -> list[str]:
        """
        Get a list of hosts that failed.

        :param results: The results of the task.
        :return: The list of hosts that failed.
        """
        return self._results_hosts(results.failed)


class AbstractTaskWhichRequiresSudo(AbstractTask):
    def __init__(self) -> None:
        super().__init__()
        self._sudo_password: str | None = None

    def set_sudo_password(self, sudo_password: str) -> None:
        self._sudo_password = sudo_password
