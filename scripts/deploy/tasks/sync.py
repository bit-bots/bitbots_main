import concurrent.futures
import os

from deploy.misc import Connection, be_quiet, hide_output, print_debug, print_error
from deploy.tasks.abstract_task import AbstractTask
from fabric import Group, GroupResult, Result


class Sync(AbstractTask):
    def __init__(self, local_workspace: str, remote_workspace: str) -> None:
        """
        Sync task that synchronizes (copies) the local workspace directory to the remote server.
        Excludes files and directories as specified in the .rsyncignore file in the local workspace.

        :param local_workspace: Path to the local workspace to sync
        :param remote_workspace: Path to the remote workspace to sync to
        """
        super().__init__()
        self._local_workspace = local_workspace
        self._remote_workspace = remote_workspace

    def _run(self, connections: Group) -> GroupResult:
        """
        Synchronize (copy) the workspace directory to the remotes using the rsync tool.

        :param connections: The connections to remote servers.
        :return: The results of the task.
        """
        # Synchronize the local source directory to the remote workspace with rsync
        rsync_results = self._rsync(connections)
        return rsync_results

    def _rsync(self, connections: Group) -> GroupResult:
        """
        Synchronize (copy) the local source directory to the remotes using the rsync tool.
        This happens in parallel for all connections.

        :param connections: The connections to remote servers.
        :return: The results of the task.
        """

        def _sync_single(connection: Connection) -> Result | None:
            # Construct the rsync command
            cmd = [
                "rsync",
                "--checksum",
                "--archive",
                "--delete",
                "-e",
                '"ssh -o StrictHostKeyChecking=no"',
                f"--exclude-from={os.path.join(self._local_workspace, '.rsyncignore')}",
            ]
            if not be_quiet():
                cmd.append("--verbose")
            cmd.extend(
                [
                    self._local_workspace + "/",  # NOTE: The trailing slash is important for rsync
                    f"{connection.user}@{connection.host}:{self._remote_workspace}",
                ]
            )
            cmd = " ".join(cmd)

            print_debug(f"Calling '{cmd}'")
            invoke_result = connection.local(cmd, hide=hide_output())

            if invoke_result is None:
                print_error(f"Rsync command failed to execute on host {connection.host}")
                return

            # Verify syncing succeeded and system architectures matches by running "pixi install"
            cmd = f"cd {self._remote_workspace} && pixi install"
            print_debug("Installing dependencies on remote host to verify synchronization and architecture match.")
            print_debug(f"Calling '{cmd}' on: {connection.host}")
            verify_result = connection.run(cmd, hide=hide_output())

            return verify_result

        print_debug(
            f"Synchronizing local source directory ('{self._local_workspace}') to  the remote directory: '{self._remote_workspace}'"
        )

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
            print_error(f"Synchronization failed for hosts {self._failed_hosts(results)}")
        return results
