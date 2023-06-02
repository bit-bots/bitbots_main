from typing import List

import os
import subprocess

import yaml
from fabric import Group, Result

from tasks.abstract_task import AbstractTask
from misc import *

class Sync(AbstractTask):
    def __init__(
            self,
            local_workspace: str,
            remote_workspace: str,
            package: str = '',
            pre_clean: bool = False
        ) -> None:
        """
        Sync task that synchronizes (copies) the local source directory to the remote server.

        :param local_workspace: Path to the local workspace to sync
        :param remote_workspace: Path to the remote workspace to sync to
        :param package: Limit to file from this package, if empty, all files are included
        :param pre_clean: Whether to clean the source directory before syncing
        """
        super().__init__()
        self._local_workspace = local_workspace
        self._remote_workspace = remote_workspace
        self._package = package
        self._pre_clean = pre_clean

        self._includes = self._get_includes_from_file(
            os.path.join(self._local_workspace, f"sync_includes_wolfgang_nuc.yaml"),
            self._package
        )

    def _get_includes_from_file(self, file_path: str, package: str = '') -> List[str]:
        """
        Retrieve a list of file to sync from and includes-file

        :param file_path: Path of the includes-file
        :param package: Limit to file from this package, if empty, all files are included
        :returns: List of files to sync
        """
        includes = list()
        with open(file_path) as file:
            data = yaml.safe_load(file)
            # Exclude files
            for entry in data['exclude']:
                includes.append(f'--include=- {entry}')
            # Include files
            for entry in data['include']:
                if isinstance(entry, dict):
                    for folder, subfolders in entry.items():
                        if package == '':
                            includes.append(f'--include=+ {folder}')
                            for subfolder in subfolders:
                                includes.append(f'--include=+ {folder}/{subfolder}')
                                includes.append(f'--include=+ {folder}/{subfolder}/**')
                        elif package in subfolders:
                            includes.append(f'--include=+ {folder}')
                            includes.append(f'--include=+ {folder}/{package}')
                            includes.append(f'--include=+ {folder}/{package}/**')
                elif isinstance(entry, str):
                    if package == '' or package == entry:
                        includes.append(f'--include=+ {entry}')
                        includes.append(f'--include=+ {entry}/**')
        includes.append('--include=- *')
        return includes

    def run(self, connections: Group) -> Result:
        """
        Synchronize (copy) the local source directory to the given Target using the rsync tool.

        :param connections: The connections to remote servers.
        :return: The result of the task.
        """
        if self._pre_clean and self._package:
            print_warn("Cleaning selected packages is not supported. Will clean all packages instead.")

        if self._pre_clean:
            print_debug("Cleaning source directory")
            src_path = os.path.join(self._remote_workspace, 'src')
            clean_result = connections.run(f"rm -rf {src_path} ; mkdir -p {src_path}")
            if not clean_result.ok:
                print_warn(f"Cleaning of source directory failed. Continuing anyways")

        for connection in connections:
            cmd = [
                "rsync",
                "--checksum",
                "--archive",
                "--delete",
            ]

            if not be_quiet():
                cmd.append("--verbose")

            cmd.extend(self._includes)
            cmd.extend([os.path.join(self._local_workspace, "/"),  f"bitbots@{connection.host}:{self._remote_workspace}/src/"])

            print_debug(f"Calling {' '.join(cmd)}")
            sync_result = subprocess.run(cmd)
            if sync_result.returncode != 0:
                print_err(f"Synchronizing task failed with error code {sync_result.returncode}")
                exit(sync_result.returncode)

        # return Result()  # TODO: Implement
