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
        This is, as we want to make sure, nothing else interferres with
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
        cmd = 'ros2 node list -c | grep -q "^0$"'
        print_debug(f"Calling {cmd}")
        results = connections.run(cmd, hide=hide_output())

        if results.failed:
            print_err(f"ROS 2 nodes are already running or check failed on {self._failed_hosts(results)}")
        if results.succeeded:
            print_debug(f"No ROS 2 nodes are already running on {self._succeded_hosts(results)}")
        return results

    def _check_tmux_session_already_running(self, connections: Group) -> GroupResult:
        print_debug(f"Checking if tmux session with name '{self._tmux_session_name}' is already running")

        # The first part of the command outputs names of all running tmux sessions.
        # The second part scanns for the given session name using grep.
        # -v inverts the results, thus not finding the string succeeds.
        cmd = f"""tmux ls -F '#S' | grep -qv "{self._tmux_session_name}" """
        print_debug(f"Calling: {cmd}")
        results = connections.run(cmd, hide=hide_output())

        if results.succeeded:
            print_debug(f"No tmux session called {self._tmux_session_name} has been found on hosts {self._succeded_hosts(results)}")
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
            cmds = {}
            for connection in results.succeeded:
                cmds[connection.host] = f"ssh {connection.host} -t 'tmux attach-session -t {self._tmux_session_name}'"
            print_success(f"Teamplayer launched successfully on {self._succeded_hosts(results)}!\nTo attach to the tmux session, run:\n\n{cmds}")
        if results.failed:
            print_err(f"Creating tmux session called {self._tmux_session_name} failed OR launching teamplayer failed on the following hosts: {self._failed_hosts(results)}")
        return results
