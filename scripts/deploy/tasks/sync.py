import concurrent.futures
import os
from typing import Optional

import yaml
from deploy.misc import Connection, be_quiet, get_connections_from_succeeded, hide_output, print_debug, print_err
from deploy.tasks.abstract_task import AbstractTask
from fabric import Group, GroupResult, Result
from fabric.exceptions import GroupException


class Sync(AbstractTask):
    def __init__(
        self,
        local_src_bitbots_main: str,
        remote_workspace: str,
        package: str = "",
        pre_clean: bool = False,
        fast: bool = False,
    ) -> None:
        """
        Sync task that synchronizes (copies) the local source directory to the remote server.

        :param _local_src_bitbots_main: Path to the local src/bitbots_main directory to sync
        :param remote_workspace: Path to the remote workspace to sync to
        :param package: Limit to file from this package, if empty, all files are included
        :param pre_clean: Whether to clean the source directory before syncing
        :param fast: Whether to sync the current local colcon workspace including build artifacts
        """
        super().__init__()
        self._local_src_bitbots_main = local_src_bitbots_main
        self._remote_workspace = remote_workspace
        self._package = package
        self._pre_clean = pre_clean
        self._fast = fast

    def _get_includes_from_file(self, file_path: str, package: str = "") -> list[str]:
        """
        Retrieve the include and exclude arguments for rsync from a yaml file.

        :param file_path: Path of the includes-file
        :param package: Limit to file from this package, if empty, all files are included
        :returns: Include and exclude arguments for rsync
        """
        with open(file_path) as file:
            data = yaml.safe_load(file)

        # Exclude files
        excludes: list[str] = data["exclude"]

        # Include files
        includes: list[str] = []
        for entry in data["include"]:
            if isinstance(entry, str):
                if package == "" or package == entry:
                    includes.append(f"{entry}")
                    includes.append(f"{entry}/**")
            elif isinstance(entry, dict):
                for directory, subdirectories in entry.items():
                    if package == "":
                        includes.append(f"{directory}")
                        for subdirectory in subdirectories:
                            includes.append(f"{directory}/{subdirectory}")
                            includes.append(f"{directory}/{subdirectory}/**")
                    elif package in subdirectories:
                        includes.append(f"{directory}")
                        includes.append(f"{directory}/{package}")
                        includes.append(f"{directory}/{package}/**")

        # Encase in quotes
        excludes = [f"'--include=- {exclude}'" for exclude in excludes]
        includes = [f"'--include=+ {include}'" for include in includes]

        return excludes + includes + ["'--include=- *'"]

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

        if self._fast:
            rsync_results = self._rsync(
                connections,
                os.path.dirname(os.path.dirname(self._local_src_bitbots_main))
                + "/",  # NOTE: The trailing slash is important for rsync
                [
                    "'--include=- log'",
                    "'--include=+ build'",
                    "'--include=+ build/**'",
                    "'--include=+ install'",
                    "'--include=+ install/**'",
                    "'--include=+ src'",
                    "'--include=+ src/**'",
                    "'--include=- *'",
                ],
                self._remote_workspace,
            )
        else:
            # Synchronize the local source directory to the remote workspace with rsync
            includes = self._get_includes_from_file(
                os.path.join(self._local_src_bitbots_main, "sync_includes_wolfgang_nuc.yaml"), self._package
            )
            rsync_results = self._rsync(
                connections,
                self._local_src_bitbots_main + "/",  # NOTE: The trailing slash is important for rsync
                includes,
                os.path.join(self._remote_workspace, "src"),
            )
        return rsync_results

    def _clean(self, connections: Group) -> GroupResult:
        """
        Clean the remote workspace by removing the src/ directory and recreating it.

        :return: The results of the task.
        """
        remote_src_path = os.path.join(self._remote_workspace, "src")

        def _remove_directories_on_single_host(connection: Connection, result: Result) -> Optional[Result]:
            """Removes directories given in the output of the previous find command.

            :param connection: The connection to the remote server.
            :param result: The result of the previous find command.
            :return: The result of the removal command.
            """
            # Remove the found directories on the succeeded hosts
            print_debug(f"Removing found directories on host {connection.host}")
            directories = " ".join(
                result.stdout.split("\n")
            )  # Directories are given line by line, we concatenate them with spaces to get a single command
            rm_cmd = f"rm -rf {directories}"
            print_debug(f"Calling '{rm_cmd}' on host {connection.host}")
            try:
                rm_results = connection.run(rm_cmd, hide=hide_output())
                print_debug(f"Cleaning of package succeeded for hosts {connection.host}")
            except Exception as e:
                print_err(f"Cleaning of package failed for hosts {Connection.host}")
                return Result(connection=connection, cmd=rm_cmd, exited=1, stdout="", stderr=str(e))
            return rm_results

        if self._package and not self._fast:  # Package given, clean only the package directory
            # Only clean the package directory
            # To do this, we find sub-directories with the package name and remove them
            print_debug(
                f"Searching for package '{self._package}' in remote source directory: '{remote_src_path} to clean it'"
            )
            find_cmd = f'find {remote_src_path} -type d -not -path "*/.git/*" -name "{self._package}"'
            print_debug(f"Calling '{find_cmd}'")
            try:
                find_results = connections.run(find_cmd, hide=hide_output())
                print_debug(f"Finding of package succeeded for hosts {self._succeeded_hosts(find_results)}")
            except GroupException as e:
                print_err(f"Finding of package failed for hosts {self._failed_hosts(e.result)}")
                if not e.result.succeeded:  # TODO: Does run immediately if one host fails? Do we even get here?
                    return e.result
                find_results = e.result

            # Collect removal results from the succeeded hosts
            remove_results = GroupResult()

            # Remove the found directories on the succeeded hosts concurrently
            with concurrent.futures.ThreadPoolExecutor(max_workers=len(find_results.succeeded)) as executor:
                futures = [
                    executor.submit(_remove_directories_on_single_host, connection, result)
                    for connection, result in find_results.succeeded.items()
                ]

            for future in futures:
                remove_result = future.result()
                if remove_result is not None:
                    remove_results[remove_result.connection] = remove_result

            # Re-Bifurcate results into succeeded and failed results
            # This is necessary, as the GroupResult does this prematurely, before we could collect the results from all hosts
            for key, value in remove_results.items():
                if value.exited != 0:
                    remove_results.failed[key] = value
                else:
                    remove_results.succeeded[key] = value
            return remove_results

        else:  # No package given, clean the entire source directory
            # First, remove the source directory
            print_debug(f"Removing source directory: '{remote_src_path}'")
            rm_cmd = f"rm -rf {remote_src_path}"

            print_debug(f"Calling '{rm_cmd}'")
            try:
                rm_results = connections.run(rm_cmd, hide=hide_output())
                print_debug(f"Cleaning of source directory succeeded for hosts {self._succeeded_hosts(rm_results)}")
            except GroupException as e:
                print_err(f"Cleaning of source directory failed for hosts {self._failed_hosts(e.result)}")
                if not e.result.succeeded:  # TODO: Does run immediately if one host fails? Do we even get here?
                    return e.result
                rm_results = e.result

            # Only continue on succeeded hosts
            connections = get_connections_from_succeeded(rm_results)

            # Second, create an empty directory again
            print_debug(f"Creating source directory: '{remote_src_path}'")
            mkdir_cmd = f"mkdir -p {remote_src_path}"

            print_debug(f"Calling '{mkdir_cmd}'")
            try:
                mkdir_results = connections.run(mkdir_cmd, hide=hide_output())
                print_debug(
                    f"Recreation of source directory succeeded for hosts {self._succeeded_hosts(mkdir_results)}"
                )
            except GroupException as e:
                print_err(f"Recreation of source directory failed for hosts {self._failed_hosts(e.result)}")
                return e.result
            return mkdir_results

    def _rsync(self, connections: Group, local_path: str, includes: list[str], remote_path: str) -> GroupResult:
        """
        Synchronize (copy) the local directory to the remotes using the rsync tool.
        This happens in parallel for all connections.

        :param connections: The connections to remote servers.
        :param local_path: The path to the local directory to sync from.
        :param includes: The include and exclude arguments for rsync.
        :param remote_path: The path to the remote directory to sync to.
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
            cmd.extend(includes)
            cmd.extend(
                [
                    local_path,
                    f"{connection.user}@{connection.host}:{remote_path}",
                ]
            )
            cmd = " ".join(cmd)

            print_debug(f"Calling {cmd}")
            invoke_result = connection.local(cmd, hide=hide_output())

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

        print_debug(f"Synchronizing local source directory ('{local_path}') to  the remote directory: '{remote_path}'")

        # Collect results of the group
        results = GroupResult()

        # Create a ThreadPoolExecutor to run the _sync_single function for each connection in parallel
        with concurrent.futures.ThreadPoolExecutor(max_workers=len(connections)) as executor:
            # Submit the _sync_single function for each connection to the executor
            futures = [executor.submit(_sync_single, connection) for connection in connections]

        # Wait for all futures to complete and collect the results
        for future in futures:
            result: Result = future.result()  # type: ignore
            results[result.connection] = result

        if results.succeeded:
            print_debug(f"Synchronization succeeded for hosts {self._succeeded_hosts(results)}")
        if results.failed:
            print_err(f"Synchronization failed for hosts {self._failed_hosts(results)}")
        return results
