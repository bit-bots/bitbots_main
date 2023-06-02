

def launch_teamplayer(target):
    """
    Launch the teamplayer on given target.
    This is started in a new tmux session.
    Fails if ROS 2 nodes are already running or a tmux session called "teamplayer" is already running.

    :param target: The Target to launch the teamplayer on
    """
    # Check if ROS 2 nodes are already running
    result_nodes_running = _execute_on_target(target, "ros2 node list -c")
    if not result_nodes_running.ok:
        print_err(f"Abort launching teamplayer: Could not determine if ROS 2 nodes are already running on {target.hostname}")
        sys.exit(result_nodes_running.exited)
    count = int(str(result_nodes_running.stdout).strip())
    if count > 0:
        print_err(f"Abort launching teamplayer: {count} nodes are already running on {target.hostname}")
        sys.exit(1)

    # Check if tmux session is already running
    tmux_session_name = "teamplayer"
    result_tmux_session_exists = _execute_on_target(target, "tmux ls -F '#S' || true")
    if tmux_session_name in str(result_tmux_session_exists.stdout):
        print_err(f"Abort launching teamplayer: tmux session 'teamplayer' is already running on {target.hostname}")
        sys.exit(1)
    
    # Launch teamplayer
    print_info(f"Launching teamplayer on {target.hostname}")
    # Create tmux session
    result_new_tmux_session = _execute_on_target(target, f"tmux new-session -d -s {tmux_session_name}")
    if not result_new_tmux_session.ok:
        print_err(f"Could not create tmux session on {target.hostname}")
        sys.exit(result_new_tmux_session.exited)
    # Start teamplayer in tmux session
    result_launch_teamplayer = _execute_on_target(target, f"tmux send-keys -t {tmux_session_name} 'ros2 launch bitbots_bringup teamplayer.launch' Enter")
    if not result_launch_teamplayer.ok:
        print_err(f"Could not start teamplayer launch command in tmux session on {target.hostname}")
        sys.exit(result_launch_teamplayer.exited)
    print_success(f"Teamplayer on {target.hostname} launched successfully!\nTo attach to the tmux session, run:\n\nssh {target.hostname} -t 'tmux attach-session -t {tmux_session_name}'"
    )
