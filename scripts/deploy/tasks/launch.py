import re

from deploy.misc import get_connections_from_succeeded, hide_output, print_debug, print_err, print_success
from deploy.tasks.abstract_task import AbstractTask
from fabric import Group, GroupResult, Result
from fabric.exceptions import GroupException


class Launch(AbstractTask):
    def __init__(self, tmux_session_name: str) -> None:
        """
        Launch the teamplayer ROS software on a remote machine in a new tmux session.

        :param tmux_session_name: Name of the fresh tmux session to launch the teamplayer in
        """
        super().__init__()

        self._tmux_session_name = tmux_session_name

    def _run(self, connections: Group) -> GroupResult:
        """
        Launch the teamplayer ROS software on a remote machine in a new tmux session.
        Fails if ROS 2 nodes are already running or
        a tmux session called "teamplayer" is already running.
        This is, as we want to make sure, nothing else interferes with
        the newly launched teamplayer software.

        :param connections: The connections to remote servers.
        :return: The results of the task.
        """
        # Check if ROS 2 nodes are already running
        node_running_results = self._check_nodes_already_running(connections)
        if not node_running_results.succeeded:
            return node_running_results

        # Some nodes have no ROS 2 nodes already running, continuing
        # Check if tmux session is already running
        tmux_session_running_results = self._check_tmux_session_already_running(
            get_connections_from_succeeded(node_running_results)
        )
        if not tmux_session_running_results.succeeded:
            return tmux_session_running_results

        # Some hosts are ready to launch teamplayer
        launch_results = self._launch_teamplayer(get_connections_from_succeeded(tmux_session_running_results))
        return launch_results

    def _check_nodes_already_running(self, connections: Group) -> GroupResult:
        """
        Check if ROS 2 nodes are already running on the remote machines.

        :param connections: The connections to remote servers.
        :return: Results, with success if ROS 2 nodes are not already running
        """
        print_debug("Checking if ROS 2 nodes are already running")
        cmd = "ros2 node list -c"

        print_debug(f"Calling {cmd}")
        try:
            node_list_results = connections.run(cmd, hide=hide_output())
            print_debug(f"Calling {cmd} succeeded on {self._succeeded_hosts(node_list_results)}")
        except GroupException as e:
            print_err(f"Calling {cmd} failed on {self._failed_hosts(e.result)}")
            if not e.result.succeeded:  # TODO: Does run immediately fail if one host fails? Do we even get here?
                return e.result
            else:
                node_list_results = e.result

        # Check if output was ^0$ (zero for no nodes running) for all remote machines
        # Create new group result with success if no nodes are running
        no_nodes_already_running_results = GroupResult()
        nodes_already_running: bool
        for connection, result in node_list_results.items():
            if re.match(r"^0$", result.stdout):  # No nodes already running
                nodes_already_running = False
            else:  # Nodes already running
                nodes_already_running = True

            # Create new result and pass most of the data from the original result.
            # Exited is the exit code.
            # We set it to 0 if no nodes are already running, else 1.
            no_nodes_already_running_results[connection] = Result(
                connection=connection,
                exited=int(nodes_already_running),
                command=result.command,
                stdout=result.stdout,
                stderr=result.stderr,
            )

        if no_nodes_already_running_results.succeeded:
            print_debug(
                f"No ROS 2 nodes are already running on {self._succeeded_hosts(no_nodes_already_running_results)}"
            )
        if no_nodes_already_running_results.failed:
            print_err(f"ROS 2 nodes are already running on {self._failed_hosts(no_nodes_already_running_results)}")
        return no_nodes_already_running_results

    def _check_tmux_session_already_running(self, connections: Group) -> GroupResult:
        print_debug(f"Checking if tmux session with name '{self._tmux_session_name}' is already running")

        # We need to check if the default tmux socket is already created,
        # before we can list all tmux sessions.
        # Else, the tmux ls command would fail.
        # The tmux ls command outputs names of all running tmux sessions.
        cmd = "test -S /tmp/tmux-1000/default && tmux ls -F '#S' ; true"

        print_debug(f"Calling: {cmd}")
        try:
            tmux_ls_results = connections.run(cmd, hide=hide_output())
            print_debug(f"Calling {cmd} succeeded on {self._succeeded_hosts(tmux_ls_results)}")
        except GroupException as e:
            print_err(f"Calling {cmd} failed on {self._failed_hosts(e.result)}")
            if not e.result.succeeded:
                return e.result
            else:
                tmux_ls_results = e.result

        # Check if the tmux session name can be found in the output of tmux ls
        # If so, the tmux session is already running
        # Create new group result with success if no tmux session is already running
        tmux_session_already_running_results = GroupResult()
        tmux_session_already_running: bool
        for connection, result in tmux_ls_results.items():
            if self._tmux_session_name in result.stdout:
                tmux_session_already_running = True
            else:
                tmux_session_already_running = False

            # Create new result and pass most of the data from the original result.
            # Exited is the exit code.
            # We set it to 0 if no tmux session is already running, else 1.
            tmux_session_already_running_results[connection] = Result(
                connection=connection,
                exited=int(tmux_session_already_running),
                command=result.command,
                stdout=result.stdout,
                stderr=result.stderr,
            )

        if tmux_session_already_running_results.succeeded:
            print_debug(
                f"No tmux session called {self._tmux_session_name} has been found on hosts {self._succeeded_hosts(tmux_ls_results)}"
            )
        if tmux_session_already_running_results.failed:
            print_err(
                f"Tmux session called {self._tmux_session_name} has been found on hosts {self._failed_hosts(tmux_ls_results)}"
            )
        return tmux_session_already_running_results

    def _launch_teamplayer(self, connections: Group) -> GroupResult:
        print_debug("Launching teamplayer")
        # Create tmux session
        cmd = f"tmux new-session -d -s {self._tmux_session_name} && tmux send-keys -t {self._tmux_session_name} 'ros2 launch bitbots_bringup teamplayer.launch' Enter"

        print_debug(f"Calling {cmd}")
        try:
            results = connections.run(cmd, hide=hide_output())
            # Print commands to connect to teamplayer tmux session
            help_cmds = ""
            for connection in results.succeeded:
                help_cmds += f"{connection.host} : [bold]ssh {connection.user}@{connection.host} -t 'tmux attach-session -t {self._tmux_session_name}'[/bold]\n"
            print_success(
                f"Teamplayer launched successfully on {self._succeeded_hosts(results)}!\nTo attach to the tmux session, run:\n\n{help_cmds}"
            )
        except GroupException as e:
            print_err(
                f"Creating tmux session called {self._tmux_session_name} failed OR launching teamplayer failed on the following hosts: {self._failed_hosts(e.result)}"
            )
            return e.result
        return results
