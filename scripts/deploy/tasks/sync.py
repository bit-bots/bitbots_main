import os
import concurrent.futures

import yaml
from fabric import Group, GroupResult, Result

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

        self._remote_src_path = os.path.join(self._remote_workspace, 'src')

        self._includes = self._get_includes_from_file(
            os.path.join(self._local_workspace, f"sync_includes_wolfgang_nuc.yaml"),
            self._package
        )

    def _get_includes_from_file(self, file_path: str, package: str = '') -> str:
        """
        Retrieve the include and exclude arguments for rsync from a yaml file.

        :param file_path: Path of the includes-file
        :param package: Limit to file from this package, if empty, all files are included
        :returns: Include and exclude arguments for rsync
        """
        with open(file_path) as file:
            data = yaml.safe_load(file)

        # Exclude files
        excludes: list[str] = data['exclude']

        # Include files
        includes: list[str] = []
        for entry in data['include']:
            if isinstance(entry, str):
                if package == '' or package == entry:
                    includes.append(f'{entry}/**')
                    includes.append(f'{entry}/')
            elif isinstance(entry, dict):
                for folder, subfolders in entry.items():
                    if package == '':
                        includes.append(f'{folder}/')
                        for subfolder in subfolders:
                            includes.append(f'{folder}/{subfolder}/')
                            includes.append(f'{folder}/{subfolder}/**')
                    elif package in subfolders:
                        includes.append(f'{folder}/')
                        includes.append(f'{folder}/{package}/')
                        includes.append(f'{folder}/{package}/**')
        includes.append('*')

        # Encase in quotes
        excludes = [f'"{exclude}",' for exclude in excludes]
        includes = [f'"{include}",' for include in includes]

        return f"--exclude={{{''.join(excludes)}}} --include={{{''.join(includes)}}}"

    def _run(self, connections: Group) -> GroupResult:
        """
        Synchronize (copy) the local source directory to the remotes using the rsync tool.

        :param connections: The connections to remote servers.
        :return: The results of the task.
        """
        # Optional: Clean the remote workspace
        if self._pre_clean:
            clean_results = self._clean(connections)
            if not clean_results.succeeded:
                return clean_results
            connections = get_connections_from_succeeded(clean_results)

        # Synchronize the local source directory to the remote workspace with rsync
        rsync_results = self._rsync(connections)
        return rsync_results

    def _clean(self, connections: Group) -> GroupResult:
        """
        Clean the remote workspace by removing the src/ directory and recreating it.

        :return: The results of the task.
        """
        if self._package:
            print_warn("Cleaning selected packages is not supported. Will clean all packages instead.")

        # First, remove the source directory
        print_debug(f"Removing source directory: '{self._remote_src_path}'")
        rm_cmd = f"rm -rf {self._remote_src_path}"
        print_debug(f"Calling '{rm_cmd}'")
        rm_result = connections.run(rm_cmd, hide=hide_output())
        if rm_result.succeeded:
            print_debug(f"Cleaning of source directory succeeded for hosts {self._succeded_hosts(rm_result)}")
        if rm_result.failed:
            print_err(f"Cleaning of source directory failed for hosts {self._failed_hosts(rm_result)}")
            return rm_result

        # Second, create an empty directory again
        print_debug(f"Creating source directory: '{self._remote_src_path}'")
        mkdir_cmd = f"mkdir -p {self._remote_src_path}"
        print_debug(f"Calling '{mkdir_cmd}'")
        mkdir_result = connections.run(mkdir_cmd, hide=hide_output())
        if mkdir_result.succeeded:
            print_debug(f"Recreation of source directory succeeded for hosts {self._succeded_hosts(mkdir_result)}")
        if mkdir_result.failed:
            print_err(f"Recreation of source directory failed for hosts {self._failed_hosts(mkdir_result)}")
        return mkdir_result

    def _rsync(self, connections: Group) -> GroupResult:
        """
        Synchronize (copy) the local source directory to the remotes using the rsync tool.
        This happens in parallel for all connections.

        :param connections: The connections to remote servers.
        :return: The results of the task.
        """
        def _sync_single(connection: Connection) -> Optional[Result]:
            # Construct the rsync command
            cmd = [
                "rsync",
                "--checksum",
                "--archive",
                "--delete",
            ]
            if not be_quiet():
                cmd.append("--verbose")
            cmd.append(self._includes)
            cmd.extend([
                self._local_workspace + "/",  # NOTE: The trailing slash is important for rsync
                f"{connection.user}@{connection.host}:{self._remote_src_path}/"  # NOTE: The trailing slash is important for rsync
            ])
            cmd = ' '.join(cmd)

            print_debug(f"Calling {cmd}")
            invoke_result = connection.local(cmd)
            if invoke_result is None:
                return
            # The result is from invoke, not fabric, so we need to convert it
            return Result(
                stdout=invoke_result.stdout,
                stderr=invoke_result.stderr,
                encoding=invoke_result.encoding,
                command=invoke_result.command,
                shell=invoke_result.shell,
                env=invoke_result.env,
                exited=invoke_result.exited,
                pty=invoke_result.pty,
                hide=invoke_result.hide,
                connection=connection,
            )

        print_debug(f"Synchronizing local source directory ('{self._local_workspace}') to  the remote directory: '{self._remote_src_path}'")

        # Collect results of the group
        results = GroupResult()

        # Create a ThreadPoolExecutor with a maximum of 5 worker threads
        with concurrent.futures.ThreadPoolExecutor(max_workers=len(connections)) as executor:
            # Submit the _sync_single function for each connection to the executor
            futures = [executor.submit(_sync_single, connection) for connection in connections]

        # Wait for all futures to complete and collect the results
        for connection, future in zip(connections, futures):
            results[connection] = future.result()

        if results.succeeded:
            print_debug(f"Synchronization succeeded for hosts {self._succeded_hosts(results)}")
        if results.failed:
            print_err(f"Synchronization failed for hosts {self._failed_hosts(results)}")
        return results
