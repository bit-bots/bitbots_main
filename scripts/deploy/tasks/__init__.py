INTERNET_TIMEOUT: float = 2.0


from deploy.tasks.abstract_task import AbstractTask, AbstractTaskWhichRequiresSudo  # noqa: E402
from deploy.tasks.build import Build  # noqa: E402
from deploy.tasks.check_repos import CheckReposTask  # noqa: E402
from deploy.tasks.configure import Configure  # noqa: E402
from deploy.tasks.launch import Launch  # noqa: E402
from deploy.tasks.sync import Sync  # noqa: E402

__all__ = [
    "INTERNET_TIMEOUT",
    "AbstractTask",
    "AbstractTaskWhichRequiresSudo",
    "Build",
    "CheckReposTask",
    "Configure",
    "Launch",
    "Sync",
]
