import re

from fabric import Group, GroupResult

from tasks.abstract_task import AbstractTask
from misc import *


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
        print_debug(f"Checking if ROS 2 nodes are already running")
        cmd = 'ros2 node list -c'
        print_debug(f"Calling {cmd}")
        results = connections.run(cmd, hide=hide_output())

        # Check if output was ^0$ (no nodes running) for all remote machines
        for connection, result in results.items():
            if not re.match(r'^0$', result.stdout):
                pass  # TODO

        if results.failed:
            print_err(f"ROS 2 nodes are already running or check failed on {self._failed_hosts(results)}")
        if results.succeeded:
            print_debug(f"No ROS 2 nodes are already running on {self._succeeded_hosts(results)}")
        return results

    def _check_tmux_session_already_running(self, connections: Group) -> GroupResult:
        print_debug(f"Checking if tmux session with name '{self._tmux_session_name}' is already running")

        # We need to check if the default tmux socket is already created,
        # before we can list all tmux sessions.
        # Else, the tmux ls command would fail.
        # The tmux ls command outputs names of all running tmux sessions.
        cmd = "test -S /tmp/tmux-1000/default && tmux ls -F '#S' ; true"
        print_debug(f"Calling: {cmd}")
        results = connections.run(cmd, hide=hide_output())

        for connection, result in results.items():
            if self._tmux_session_name in result.stdout:
                pass  # TODO
        if results.succeeded:
            print_debug(f"No tmux session called {self._tmux_session_name} has been found on hosts {self._succeeded_hosts(results)}")
        if results.failed:
            print_err(f"Tmux session called {self._tmux_session_name} has been found on hosts {self._failed_hosts(results)}")
        return results

    def _launch_teamplayer(self, connections: Group) -> GroupResult:
        print_debug(f"Launching teamplayer")
        # Create tmux session
        cmd = f"tmux new-session -d -s {self._tmux_session_name} && tmux send-keys -t {self._tmux_session_name} 'ros2 launch bitbots_bringup teamplayer.launch' Enter"
        print_debug(f"Calling {cmd}")
        results = connections.run(cmd, hide=hide_output())

        if results.succeeded:
            # Print commands to connect to teamplayer tmux session
            help_cmds = ""
            for connection in results.succeeded:
                help_cmds += f"{connection.host} : [bold]ssh {connection.host} -t 'tmux attach-session -t {self._tmux_session_name}'[/bold]\n"
            print_success(f"Teamplayer launched successfully on {self._succeeded_hosts(results)}!\nTo attach to the tmux session, run:\n\n{help_cmds}")
        if results.failed:
            print_err(f"Creating tmux session called {self._tmux_session_name} failed OR launching teamplayer failed on the following hosts: {self._failed_hosts(results)}")
        return results
