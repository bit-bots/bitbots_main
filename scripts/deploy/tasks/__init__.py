from deploy.tasks.abstract_task import AbstractTask, AbstractTaskWhichRequiresSudo
from deploy.tasks.build import Build
from deploy.tasks.check_local_main_repo import CheckLocalMainRepoTask
from deploy.tasks.configure import Configure
from deploy.tasks.install import Install
from deploy.tasks.launch import Launch
from deploy.tasks.sync import Sync

__all__ = [
    "AbstractTask",
    "AbstractTaskWhichRequiresSudo",
    "Build",
    "CheckLocalMainRepoTask",
    "Configure",
    "Install",
    "Launch",
    "Sync",
]
