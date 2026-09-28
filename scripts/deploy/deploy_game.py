#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
from deploy.misc import (
    CONSOLE,
    LOGLEVEL,
    get_known_targets,
    print_bit_bot,
    print_debug,
    print_error,
    print_info,
    print_success,
    print_warning,
)
from rich import box
from rich.panel import Panel
from rich.table import Table

# Try importing world utilities from bitbots_mujoco_sim
try:
    from bitbots_mujoco_sim.world import (
        BASE_TEAM_ID,
        TEAM_ROLE_ORDER,
        compute_game_settings,
        parse_num_robots,
    )
except ImportError:
    BASE_TEAM_ID = 6
    TEAM_ROLE_ORDER = ["offense", "goalie", "defense", "defense", "defense", "offense", "offense"]

    def parse_num_robots(num_robots: str | int) -> list[int]:
        teams = [int(part) for part in str(num_robots).split(":")]
        if any(count < 0 for count in teams) or sum(teams) < 1:
            raise ValueError(f"Invalid num_robots specification: {num_robots!r}")
        return teams

    def compute_game_settings(teams: list[int]) -> list[dict[str, Any]]:
        settings: list[dict[str, Any]] = []
        for team_index, count in enumerate(teams):
            if count > len(TEAM_ROLE_ORDER):
                raise ValueError(
                    f"Team {team_index} has {count} robots, but at most {len(TEAM_ROLE_ORDER)} players per team are supported"
                )
            role_position_counters: dict[str, int] = {}
            for robot_in_team in range(count):
                role = TEAM_ROLE_ORDER[robot_in_team]
                position_number = role_position_counters.get(role, 0)
                role_position_counters[role] = position_number + 1
                settings.append(
                    {
                        "bot_id": robot_in_team + 1,
                        "team_id": BASE_TEAM_ID + team_index,
                        "team_color": team_index % 2,
                        "role": role,
                        "position_number": position_number,
                    }
                )
        return settings


@dataclass
class RobotConfig:
    robot_index: int
    domain_id: int
    ip: str
    hostname: str
    bot_id: int
    team_id: int
    team_color: int
    role: str
    position_number: int
    assigned_host: str


def load_game_config(config_path: Path | str) -> dict[str, Any]:
    """Load and validate the game and host configuration from a YAML or JSON file."""
    path = Path(config_path)
    if not path.is_file():
        raise FileNotFoundError(f"Config file not found: {path}")

    with open(path) as f:
        config = yaml.safe_load(f)

    if not isinstance(config, dict):
        raise ValueError("Configuration root must be a dictionary/mapping.")

    # Accept multiple possible keys for game type
    game_spec = config.get("game") or config.get("num_robots") or config.get("kind") or config.get("type")
    if game_spec is None:
        raise ValueError("Configuration must specify 'game' (e.g. '4:4', '2:2', or '1').")

    # Accept multiple possible keys for hosts
    hosts = config.get("hosts") or config.get("available_hosts")
    if hosts is None:
        raise ValueError("Configuration must specify 'hosts' (list of available hosts).")

    if isinstance(hosts, str):
        hosts = [hosts]
    elif not isinstance(hosts, list) or len(hosts) == 0:
        raise ValueError("'hosts' must be a non-empty list of hostnames or IP addresses.")

    normalized_hosts: list[str] = []
    for host in hosts:
        if isinstance(host, dict):
            h_val = host.get("hostname") or host.get("ip") or host.get("name")
            if not h_val:
                raise ValueError(f"Invalid host dictionary in configuration: {host}")
            normalized_hosts.append(str(h_val))
        elif isinstance(host, str):
            normalized_hosts.append(host)
        else:
            raise ValueError(f"Invalid host specification in configuration: {host}")

    sim_ip = config.get("simulator_ip") or config.get("sim_ip")
    sim_host = config.get("simulator_host") or config.get("sim_host")
    if not sim_host and sim_ip and sim_ip in normalized_hosts:
        sim_host = sim_ip

    return {
        "game": str(game_spec),
        "hosts": normalized_hosts,
        "simulator_ip": sim_ip,
        "simulator_host": sim_host,
        "workspace": config.get("workspace", "~/bitbots_main"),
        "user": config.get("user", "bitbots"),
        "teamplayer_args": config.get("teamplayer_args", {}),
    }


def resolve_robot_targets(num_robots: int) -> list[tuple[int, str, str]]:
    """Determine domain ID, virtual IP, and robot hostname for each robot index."""
    known = get_known_targets()
    virtual_ips: dict[int, tuple[str, str]] = {}

    for ip_str, details in known.items():
        if ip_str.startswith("10.66."):
            domain = int(details.get("domain_id") or details.get("ros_domain_id") or 0)
            if domain >= 11:
                hostname = details.get("hostname") or details.get("robot_name") or f"robot{domain}"
                virtual_ips[domain] = (ip_str, str(hostname))

    robot_targets = []
    for idx in range(num_robots):
        domain = 11 + idx
        if domain in virtual_ips:
            ip, hostname = virtual_ips[domain]
        else:
            ip = f"10.66.6.{idx + 1}"
            hostname = f"robot{domain}"
        robot_targets.append((domain, ip, hostname))

    return robot_targets


def assign_robots_to_hosts(
    game_spec: str,
    hosts: list[str],
    simulator_host: str | None = None,
    sim_weight: int = 2,
) -> list[RobotConfig]:
    """Parse game settings and distribute robots across available hosts.

    The simulator host is estimated to be worth 2 extra robots (sim_weight=2) for an equal load distribution.
    """
    teams = parse_num_robots(game_spec)
    settings = compute_game_settings(teams)
    num_robots = len(settings)
    targets = resolve_robot_targets(num_robots)

    # Simulator host starts with initial load equal to sim_weight if present in hosts
    host_load = {host: (sim_weight if (simulator_host and host == simulator_host) else 0) for host in hosts}
    host_robot_count = {host: 0 for host in hosts}

    assignments: list[RobotConfig] = []
    for idx, (setting, (domain, ip, hostname)) in enumerate(zip(settings, targets)):
        best_host = min(hosts, key=lambda h: (host_load[h], host_robot_count[h], hosts.index(h)))
        host_load[best_host] += 1
        host_robot_count[best_host] += 1

        assignments.append(
            RobotConfig(
                robot_index=idx,
                domain_id=domain,
                ip=ip,
                hostname=hostname,
                bot_id=setting["bot_id"],
                team_id=setting["team_id"],
                team_color=setting["team_color"],
                role=setting["role"],
                position_number=setting["position_number"],
                assigned_host=best_host,
            )
        )

    return assignments


def generate_host_config_command(
    is_simulator_host: bool,
    simulator_ip: str | None = None,
) -> str:
    """Generate the ./docker/manage.py command to run the host configuration container."""
    config_type = "sim" if is_simulator_host else "robot"
    cmd = f"./docker/manage.py run-config {config_type} $(hostname)"
    if simulator_ip:
        cmd += f" {simulator_ip}"
    return cmd


def generate_container_command(
    robot: RobotConfig,
    simulator_ip: str | None = None,
) -> str:
    """Generate the ./docker/manage.py command to run the robot container with Zenoh."""
    cmd = f"./docker/manage.py run-project {robot.hostname} --zenoh-router"
    if simulator_ip:
        cmd += f" --simulator-ip {simulator_ip}"
    return cmd


def generate_teamplayer_command(
    robot: RobotConfig,
    extra_args: dict[str, Any] | None = None,
) -> str:
    """Generate the ROS 2 launch command to start teamplayer for this robot."""
    args = [
        "sim:=true",
        f"bot_id:={robot.bot_id}",
        f"team_id:={robot.team_id}",
        f"team_color:={robot.team_color}",
        f"role:={robot.role}",
        f"position_number:={robot.position_number}",
    ]
    if extra_args:
        for k, v in extra_args.items():
            args.append(f"{k}:={v}")

    args_str = " ".join(args)
    return f"ROS_DOMAIN_ID={robot.domain_id} pixi run -e default ros2 launch bitbots_bringup teamplayer.launch {args_str}"


def generate_tmux_ssh_command(
    robot: RobotConfig,
    user: str = "bitbots",
    extra_args: dict[str, Any] | None = None,
    workspace: str = "~/bitbots_main",
) -> str:
    """Generate an SSH + tmux one-liner to start teamplayer in background."""
    teamplayer_cmd = generate_teamplayer_command(robot, extra_args)
    tmux_session = f"teamplayer_{robot.domain_id}"
    full_cmd = f"tmux new-session -d -s {tmux_session} 'cd {workspace} && {teamplayer_cmd}'"
    return f"ssh {user}@{robot.ip} \"{full_cmd}\""


class DeployGame:
    def __init__(self, argv: list[str] | None = None) -> None:
        self.repo_root = Path(__file__).resolve().parents[2]
        os.chdir(self.repo_root)

        self._args = self._parse_arguments(argv)
        LOGLEVEL.CURRENT = LOGLEVEL.CURRENT + self._args.verbose - self._args.quiet

        if self._args.print_bit_bot:
            print_bit_bot()

        self.config = load_game_config(self._args.config)
        sim_host = self._args.simulator_host or self.config.get("simulator_host")
        sim_ip = self._args.simulator_ip or self.config.get("simulator_ip")
        if not sim_host and sim_ip and sim_ip in self.config["hosts"]:
            sim_host = sim_ip

        self.robots = assign_robots_to_hosts(
            self.config["game"],
            self.config["hosts"],
            simulator_host=sim_host,
        )

        self.display_game_overview()

        if not self._args.no_deploy:
            self.run_deploy()

        self.display_host_commands()

    def _parse_arguments(self, argv: list[str] | None = None) -> argparse.Namespace:
        parser = argparse.ArgumentParser(
            description="Deploy and orchestrate a multi-robot simulation game across multiple hosts."
        )
        parser.add_argument(
            "config",
            type=str,
            help="Path to the game configuration YAML file.",
        )
        parser.add_argument(
            "--no-deploy",
            "--dry-run",
            action="store_true",
            dest="no_deploy",
            help="Do not execute the deploy script on hosts; only display the deployment plan and commands.",
        )
        parser.add_argument(
            "--sync",
            action="store_true",
            help="Pass --sync to deploy_robots.py (only synchronize files).",
        )
        parser.add_argument(
            "--build",
            action="store_true",
            help="Pass --build to deploy_robots.py (only build workspace).",
        )
        parser.add_argument(
            "--configure",
            action="store_true",
            help="Pass --configure to deploy_robots.py.",
        )
        parser.add_argument(
            "--launch",
            action="store_true",
            help="Pass --launch to deploy_robots.py.",
        )
        parser.add_argument(
            "--container",
            action="store_true",
            default=True,
            help="Deploy in container mode (default: True).",
        )
        parser.add_argument(
            "-u",
            "--user",
            type=str,
            default=None,
            help="Override SSH user for target hosts.",
        )
        parser.add_argument(
            "--simulator-host",
            "--sim-host",
            type=str,
            default=None,
            dest="simulator_host",
            help="Specify host designated as simulator host (receives 2 fewer robots for load balancing).",
        )
        parser.add_argument(
            "--simulator-ip",
            "--sim-ip",
            type=str,
            default=None,
            dest="simulator_ip",
            help="Override simulator IP in commands.",
        )
        parser.add_argument(
            "-w",
            "--workspace",
            type=str,
            default=None,
            help="Override remote workspace path.",
        )
        parser.add_argument(
            "--deploy-args",
            type=str,
            default="",
            help="Additional raw arguments to pass to deploy_robots.py.",
        )
        parser.add_argument(
            "--skip-local-repo-check",
            action="store_true",
            help="Pass --skip-local-repo-check to deploy_robots.py.",
        )
        parser.add_argument(
            "-v",
            "--verbose",
            action="count",
            default=0,
            help="Increase verbosity.",
        )
        parser.add_argument(
            "-q",
            "--quiet",
            action="count",
            default=0,
            help="Decrease verbosity.",
        )
        parser.add_argument(
            "--print-bit-bot",
            action="store_true",
            default=False,
            help="Print Bit-Bots logo.",
        )

        return parser.parse_args(argv)

    def display_game_overview(self) -> None:
        """Display summary table of the game setup and robot-host allocation."""
        table = Table(title=f"Game Setup ({self.config['game']}) - Host Allocation", box=box.ROUNDED)
        table.add_column("Robot Index", justify="center", style="cyan")
        table.add_column("Domain ID", justify="center", style="magenta")
        table.add_column("Identifier / IP", style="green")
        table.add_column("Team (Color)", justify="center")
        table.add_column("Role (Pos)", justify="center")
        table.add_column("Assigned Host", style="bold yellow")

        for r in self.robots:
            color_name = "Blue" if r.team_color == 0 else "Red"
            table.add_row(
                str(r.robot_index + 1),
                str(r.domain_id),
                f"{r.hostname} ({r.ip})",
                f"Team {r.team_id} ({color_name})",
                f"{r.role} (#{r.position_number})",
                r.assigned_host,
            )

        CONSOLE.print(table)

    def run_deploy(self) -> None:
        """Run deploy_robots.py for all target hosts."""
        deploy_script = self.repo_root / "scripts" / "deploy_robots.py"
        hosts = self.config["hosts"]
        user = self._args.user or self.config["user"]
        workspace = self._args.workspace or self.config["workspace"]

        deploy_cmd = [
            sys.executable,
            str(deploy_script),
            *hosts,
            "--user",
            user,
            "--workspace",
            workspace,
        ]

        if self._args.container:
            deploy_cmd.append("--container")
        if self._args.skip_local_repo_check:
            deploy_cmd.append("--skip-local-repo-check")
        if self._args.sync:
            deploy_cmd.append("--sync")
        if self._args.build:
            deploy_cmd.append("--build")
        if self._args.configure:
            deploy_cmd.append("--configure")
        if self._args.launch:
            deploy_cmd.append("--launch")

        if self._args.deploy_args:
            deploy_cmd.extend(self._args.deploy_args.split())

        print_info(f"Starting host deployment using deploy script for hosts: {', '.join(hosts)}")
        print_debug(f"Calling: {' '.join(deploy_cmd)}")

        try:
            subprocess.run(deploy_cmd, check=True)
            print_success(f"Deployment completed successfully on hosts: {', '.join(hosts)}")
        except subprocess.CalledProcessError as e:
            print_error(f"Deploy script failed with returncode {e.returncode}")
            if not self._args.no_deploy:
                print_warning("Continuing to display required container and teamplayer commands...")

    def display_host_commands(self) -> None:
        """Print required commands for starting containers and launching teamplayer on each host."""
        sim_ip = self._args.simulator_ip or self.config["simulator_ip"]
        sim_host = self._args.simulator_host or self.config.get("simulator_host")
        if not sim_host and sim_ip and sim_ip in self.config["hosts"]:
            sim_host = sim_ip

        extra_args = self.config["teamplayer_args"]
        user = self._args.user or self.config["user"]
        workspace = self._args.workspace or self.config["workspace"]

        # Group robots by host (preserving host order)
        host_robots: dict[str, list[RobotConfig]] = {h: [] for h in self.config["hosts"]}
        for r in self.robots:
            host_robots.setdefault(r.assigned_host, []).append(r)

        CONSOLE.print("\n[bold cyan]═══ Commands for Individual Hosts ═══[/bold cyan]\n")

        for host, robots in host_robots.items():
            is_sim = (host == sim_host)
            content = []
            host_type = "Simulator Host" if is_sim else "Robot Host"
            content.append(f"[bold underline]Host: {host}[/bold underline] ([italic]{host_type}[/italic])\n")

            # 1. Host Config Container (UDP bridge via Zenoh)
            host_config_cmd = generate_host_config_command(is_sim, sim_ip)
            config_file = "config_sim.toml" if is_sim else "config_robot.toml"
            container_name = "bitbots-config-sim" if is_sim else "bitbots-config-robot"
            effective_sim_ip = sim_ip or "10.66.0.15"
            raw_docker_cmd = (
                f"docker run -d --name {container_name} --net=bitbots-net udp_via_zenoh {config_file} $(hostname) {effective_sim_ip}"
            )
            content.append(
                f"  [dim]1. Start UDP Bridge Host Container ({config_file}):[/dim]\n"
                f"     [green]{host_config_cmd}[/green]\n"
                f"     [dim]Raw Docker: {raw_docker_cmd}[/dim]\n"
            )

            # If simulator host, also show simulator launch command if relevant
            if is_sim:
                content.append(
                    "  [dim]▶ Simulator Options (if running simulation on this host):[/dim]\n"
                    "     [green]./docker/manage.py run-simulator -z[/green]\n"
                    "     [white]pixi run -e default ros2 launch bitbots_bringup mujoco_simulation.launch.py[/white]\n"
                )

            # Robot containers & teamplayers
            if robots:
                content.append(f"  [dim]▶ Robots assigned ({len(robots)}):[/dim]")
                for r in robots:
                    color_name = "Blue" if r.team_color == 0 else "Red"
                    content.append(
                        f"    [bold yellow]• Robot {r.hostname}[/bold yellow] (Domain: [magenta]{r.domain_id}[/magenta], Team: {r.team_id} [{color_name}], Role: [cyan]{r.role}[/cyan])"
                    )

                    # Start Robot Container
                    container_cmd = generate_container_command(r, sim_ip)
                    content.append(f"      [dim]a. Start Robot Container with Zenoh:[/dim]\n         [green]{container_cmd}[/green]")

                    # Launch Teamplayer command
                    teamplayer_cmd = generate_teamplayer_command(r, extra_args)
                    content.append(f"      [dim]b. Start Teamplayer (inside container / shell):[/dim]\n         [white]{teamplayer_cmd}[/white]")

                    # SSH / Tmux shortcut
                    tmux_ssh = generate_tmux_ssh_command(r, user=user, extra_args=extra_args, workspace=workspace)
                    content.append(f"      [dim]c. Start Teamplayer in background via SSH:[/dim]\n         [cyan]{tmux_ssh}[/cyan]\n")
            else:
                content.append("  [italic dim](No robot instances assigned to this simulator host)[/italic dim]\n")

            panel = Panel(
                "\n".join(content),
                title=f"[bold]Commands for Host: {host}[/bold]",
                border_style="magenta" if is_sim else "blue",
                box=box.ROUNDED,
            )
            CONSOLE.print(panel)


if __name__ == "__main__":
    try:
        DeployGame()
    except KeyboardInterrupt:
        print_error("Interrupted by user")
        sys.exit(1)
    except Exception as e:
        print_error(str(e))
        sys.exit(1)
