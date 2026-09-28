import sys
from pathlib import Path

import pytest

# Ensure scripts directory is in sys.path
SCRIPTS_DIR = Path(__file__).resolve().parents[2]
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from deploy.deploy_game import (
    DeployGame,
    RobotConfig,
    assign_robots_to_hosts,
    generate_container_command,
    generate_host_config_command,
    generate_teamplayer_command,
    generate_tmux_ssh_command,
    load_game_config,
    resolve_robot_targets,
)


def test_load_game_config_valid(tmp_path):
    config_file = tmp_path / "config.yaml"
    config_file.write_text(
        """
game: "4:4"
hosts:
  - "host1"
  - "host2"
simulator_ip: "10.66.0.254"
workspace: "~/my_workspace"
user: "custom_user"
teamplayer_args:
  record: "true"
"""
    )

    loaded = load_game_config(config_file)
    assert loaded["game"] == "4:4"
    assert loaded["hosts"] == ["host1", "host2"]
    assert loaded["simulator_ip"] == "10.66.0.254"
    assert loaded["workspace"] == "~/my_workspace"
    assert loaded["user"] == "custom_user"
    assert loaded["teamplayer_args"] == {"record": "true"}


def test_load_game_config_simulator_host(tmp_path):
    config_file = tmp_path / "config_sim_host.yaml"
    config_file.write_text(
        """
game: "4:4"
hosts:
  - "host1"
  - "host2"
simulator_host: "host1"
simulator_ip: "10.66.0.15"
"""
    )
    loaded = load_game_config(config_file)
    assert loaded["simulator_host"] == "host1"
    assert loaded["simulator_ip"] == "10.66.0.15"


def test_load_game_config_aliases(tmp_path):
    config_file = tmp_path / "config_alias.yaml"
    config_file.write_text(
        """
num_robots: "2:2"
available_hosts:
  - hostname: "nuc1"
  - ip: "192.168.1.100"
"""
    )

    loaded = load_game_config(config_file)
    assert loaded["game"] == "2:2"
    assert loaded["hosts"] == ["nuc1", "192.168.1.100"]


def test_load_game_config_missing_file():
    with pytest.raises(FileNotFoundError):
        load_game_config("/nonexistent/path/config.yaml")


def test_load_game_config_invalid_schema(tmp_path):
    config_file = tmp_path / "invalid.yaml"
    config_file.write_text("game: '4:4'")  # Missing hosts
    with pytest.raises(ValueError, match="must specify 'hosts'"):
        load_game_config(config_file)

    config_file.write_text("hosts: ['host1']")  # Missing game
    with pytest.raises(ValueError, match="must specify 'game'"):
        load_game_config(config_file)


def test_resolve_robot_targets():
    targets = resolve_robot_targets(8)
    assert len(targets) == 8
    # Check first 6 use known virtual targets
    assert targets[0][0] == 11  # Domain ID
    assert targets[0][1] == "10.66.6.1"  # IP
    assert targets[0][2] == "kalliope"  # Hostname

    assert targets[1][0] == 12
    assert targets[1][1] == "10.66.6.2"
    assert targets[1][2] == "mickey"

    # Robot 7 (Domain 17) uses generated fallback
    assert targets[6][0] == 17
    assert targets[6][1] == "10.66.6.7"
    assert targets[6][2] == "robot17"


def test_assign_robots_to_hosts():
    assignments = assign_robots_to_hosts("4:4", ["host1", "host2"])
    assert len(assignments) == 8

    # Team 0 (Blue, team_id 6)
    assert assignments[0].domain_id == 11
    assert assignments[0].bot_id == 1
    assert assignments[0].team_id == 6
    assert assignments[0].team_color == 0
    assert assignments[0].role == "offense"
    assert assignments[0].position_number == 0
    assert assignments[0].assigned_host == "host1"

    assert assignments[1].domain_id == 12
    assert assignments[1].bot_id == 2
    assert assignments[1].team_id == 6
    assert assignments[1].team_color == 0
    assert assignments[1].role == "goalie"
    assert assignments[1].position_number == 0
    assert assignments[1].assigned_host == "host2"

    # Team 1 (Red, team_id 7) starts at index 4
    assert assignments[4].domain_id == 15
    assert assignments[4].bot_id == 1
    assert assignments[4].team_id == 7
    assert assignments[4].team_color == 1
    assert assignments[4].role == "offense"
    assert assignments[4].position_number == 0
    assert assignments[4].assigned_host == "host1"


def test_assign_robots_with_simulator_host():
    # 4:4 match (8 robots total) across 2 hosts where host1 is simulator host (weight 2)
    assignments_44 = assign_robots_to_hosts("4:4", ["host1", "host2"], simulator_host="host1")
    assert len(assignments_44) == 8
    host1_robots = [r for r in assignments_44 if r.assigned_host == "host1"]
    host2_robots = [r for r in assignments_44 if r.assigned_host == "host2"]
    # host1 has 3 robots (+ 2 for sim = 5), host2 has 5 robots (5 total load). Difference: 2 fewer robots on host1.
    assert len(host1_robots) == 3
    assert len(host2_robots) == 5

    # 2:2 match (4 robots total) across 2 hosts
    assignments_22 = assign_robots_to_hosts("2:2", ["host1", "host2"], simulator_host="host1")
    assert len(assignments_22) == 4
    host1_robots_22 = [r for r in assignments_22 if r.assigned_host == "host1"]
    host2_robots_22 = [r for r in assignments_22 if r.assigned_host == "host2"]
    # host1 has 1 robot (+ 2 = 3), host2 has 3 robots (3 total load). Difference: 2 fewer robots on host1.
    assert len(host1_robots_22) == 1
    assert len(host2_robots_22) == 3


def test_generate_host_config_command():
    sim_cmd = generate_host_config_command(is_simulator_host=True, simulator_ip="10.66.0.15")
    assert sim_cmd == "./docker/manage.py run-config sim $(hostname) 10.66.0.15"

    robot_cmd = generate_host_config_command(is_simulator_host=False, simulator_ip="10.66.0.15")
    assert robot_cmd == "./docker/manage.py run-config robot $(hostname) 10.66.0.15"


def test_generate_commands():
    robot = RobotConfig(
        robot_index=0,
        domain_id=11,
        ip="10.66.6.1",
        hostname="kalliope",
        bot_id=1,
        team_id=6,
        team_color=0,
        role="offense",
        position_number=0,
        assigned_host="host1",
    )

    container_cmd = generate_container_command(robot, simulator_ip="10.66.0.254")
    assert container_cmd == "./docker/manage.py run-project kalliope --zenoh-router --simulator-ip 10.66.0.254"

    teamplayer_cmd = generate_teamplayer_command(robot, extra_args={"tts": "false"})
    assert (
        teamplayer_cmd
        == "ROS_DOMAIN_ID=11 pixi run -e default ros2 launch bitbots_bringup teamplayer.launch sim:=true bot_id:=1 team_id:=6 team_color:=0 role:=offense position_number:=0 tts:=false"
    )

    tmux_cmd = generate_tmux_ssh_command(robot, user="bitbots", extra_args={"tts": "false"})
    assert (
        tmux_cmd
        == 'ssh bitbots@10.66.6.1 "tmux new-session -d -s teamplayer_11 \'cd ~/bitbots_main && ROS_DOMAIN_ID=11 pixi run -e default ros2 launch bitbots_bringup teamplayer.launch sim:=true bot_id:=1 team_id:=6 team_color:=0 role:=offense position_number:=0 tts:=false\'"'
    )


def test_deploy_game_dry_run(tmp_path):
    config_file = tmp_path / "game.yaml"
    config_file.write_text(
        """
game: "2:2"
hosts:
  - "hostA"
  - "hostB"
"""
    )

    # Should run successfully with --dry-run without calling deploy_robots.py
    deployer = DeployGame([str(config_file), "--dry-run"])
    assert len(deployer.robots) == 4
    assert deployer.robots[0].assigned_host == "hostA"
    assert deployer.robots[1].assigned_host == "hostB"
    assert deployer.robots[2].assigned_host == "hostA"
    assert deployer.robots[3].assigned_host == "hostB"
