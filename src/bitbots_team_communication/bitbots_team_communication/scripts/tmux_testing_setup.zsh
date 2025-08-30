#!/usr/bin/env zsh

session="TeamComm"
pkg="bitbots_team_communication"

run_tmux_session() {
  session_running="$(tmux list-sessions | grep $session)"

  if [[ -z "$session_running" ]]; then
    tmux new-session -d -s "$session"

    # window/pane setup
    tmux rename-window -t "$session:0" "Launch"
    tmux new-window -t "$session:1" -n "Test"
    tmux split-window -h

    # run required launch files in order
    tmux send-keys -t "$session:Launch" "rl bitbots_team_communication team_comm_standalone.launch" Enter

    # start test publisher/subscriber
    tmux send-keys -t "$session:Test.left" "rr bitbots_team_communication test_team_comm.py" Enter
    tmux send-keys -t "$session:Test.right" "rr bitbots_team_communication show_team_comm.py" Enter
  fi

  tmux attach-session -t "$session:Test"
}

kill_tmux_session() {
  tmux kill_session "$session"
}

trap kill_session INT

cd "$COLCON_WS"
colcon build --symlink-install --packages-up-to "$pkg"
source install/setup.zsh
run_tmux_session
