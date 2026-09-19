from __future__ import annotations

import argparse
import os
import sys

from manage.engine import ContainerEngine, get_engine
from manage.misc import (
    DEFAULT_SUBNET,
    DEFAULT_USER,
    DOCKER_DIR,
    IMAGE_NAME_BASE,
    IMAGE_NAME_PROJECT,
    LOGLEVEL,
    NETWORK_NAME,
    REPO_ROOT,
    SSH_PORT_PROJECT,
    find_ssh_key,
    print_bit_bot,
    print_debug,
    print_error,
    print_info,
    print_warning,
    resolve_robot_domain_id,
    resolve_robot_ip,
)


class ContainerManager:
    """Main application class for container management."""

    def __init__(self, argv: list[str] | None = None) -> None:
        self._args = self._parse_arguments(argv)
        LOGLEVEL.CURRENT = LOGLEVEL.CURRENT + self._args.verbose - self._args.quiet

        print_debug(f"ContainerManager initialized with args: {self._args}")

        if self._args.print_bit_bot:
            print_bit_bot()

        try:
            self.engine: ContainerEngine = get_engine(self._args.engine)
        except ValueError as e:
            print_error(str(e))
            sys.exit(1)

        self.execute_command()

    def _parse_arguments(self, argv: list[str] | None = None) -> argparse.Namespace:
        parser = argparse.ArgumentParser(
            prog="manage.py",
            description="Hamburg Bit-Bots Container Management Tool. "
            "Builds, runs, connects to, and manages Docker/Podman containers for the Bit-Bots software stack.",
            formatter_class=argparse.RawDescriptionHelpFormatter,
        )

        default_engine = os.environ.get("CONTAINER_ENGINE", "docker").lower()
        if default_engine not in ["docker", "podman"]:
            default_engine = "docker"

        parser.add_argument(
            "-e",
            "--engine",
            choices=["docker", "podman"],
            default=default_engine,
            help=f"Container engine to use (default: {default_engine}, env: CONTAINER_ENGINE)",
        )
        parser.add_argument("-v", "--verbose", action="count", default=0, help="Increase output verbosity.")
        parser.add_argument("-q", "--quiet", action="count", default=0, help="Decrease output verbosity.")
        parser.add_argument("--print-bit-bot", action="store_true", default=False, help="Print Bit-Bots logo.")
        parser.add_argument("--gpu-args", type=str, default=None, help="Override GPU flags passed to container.")
        parser.add_argument(
            "--subnet",
            type=str,
            default=DEFAULT_SUBNET,
            help=f"Container network subnet (default: {DEFAULT_SUBNET})",
        )

        subparsers = parser.add_subparsers(dest="command", help="Command to execute")

        # Build commands
        subparsers.add_parser("build-base", help="Build the base image")
        subparsers.add_parser("build-project", help="Build the project image")
        subparsers.add_parser("build-all", help="Build all images")

        build_parser = subparsers.add_parser("build", help="Build container images")
        build_parser.add_argument(
            "target_image",
            nargs="?",
            choices=["base", "project", "all"],
            default="all",
            help="Image to build (default: all)",
        )

        # Network commands
        net_parser = subparsers.add_parser("create-network", help="Create container network")
        net_parser.add_argument(
            "net_subnet",
            nargs="?",
            default=None,
            help=f"Subnet for the network (default: {DEFAULT_SUBNET})",
        )

        # Run commands
        run_parent_parser = argparse.ArgumentParser(add_help=False)
        run_parent_parser.add_argument(
            "-z",
            "--zenoh-router",
            "--zenoh",
            action="store_true",
            default=False,
            dest="zenoh_router",
            help="Start Zenoh router in container",
        )
        run_parent_parser.add_argument(
            "--simulator-ip",
            "--sim-ip",
            type=str,
            default=None,
            dest="simulator_ip",
            help="IP address of the simulator for the Zenoh router to connect to (optional)",
        )

        run_proj_parser = subparsers.add_parser(
            "run-project",
            parents=[run_parent_parser],
            help="Run project container",
        )
        run_proj_parser.add_argument("target_id", nargs="?", default=None, help="Target hostname, robot name, or IP")

        run_sim_parser = subparsers.add_parser(
            "run-simulator",
            parents=[run_parent_parser],
            help="Run simulator container",
        )
        run_sim_parser.add_argument("target_id", nargs="?", default=None, help="Target hostname, robot name, or IP")

        sim_parser = subparsers.add_parser(
            "simulator",
            parents=[run_parent_parser],
            help="Alias for run-simulator",
        )
        sim_parser.add_argument("target_id", nargs="?", default=None, help="Target hostname, robot name, or IP")

        run_parser = subparsers.add_parser("run", parents=[run_parent_parser], help="Run container by type")
        run_parser.add_argument(
            "run_type",
            choices=["project", "simulator"],
            help="Type of container to run",
        )
        run_parser.add_argument("target_id", nargs="?", default=None, help="Target hostname, robot name, or IP")

        # Stop command
        subparsers.add_parser("stop-all", help="Stop and remove all Bit-Bots containers")
        subparsers.add_parser("stop", help="Alias for stop-all")

        # SSH command
        ssh_parser = subparsers.add_parser("ssh", help="SSH into a running container")
        ssh_parser.add_argument("target_id", help="Target hostname, robot name, or IP")

        # Net shell command
        subparsers.add_parser("net-shell", help="Enter network namespace shell")
        subparsers.add_parser("netshell", help="Alias for net-shell")

        # Host connect commands (Docker Swarm overlay)
        conn_parser = subparsers.add_parser("connect-host", help="Connect host directly to overlay network (Docker)")
        conn_parser.add_argument(
            "host_ip", nargs="?", default=None, help="Host IP on container network (default: 10.66.0.254)"
        )

        subparsers.add_parser("disconnect-host", help="Disconnect host from overlay network (Docker)")

        # Swarm commands (Docker)
        swarm_init_parser = subparsers.add_parser("swarm-init", help="Initialize Docker Swarm (Docker)")
        swarm_init_parser.add_argument("advertise_addr", nargs="?", default=None, help="Optional advertise address")

        swarm_join_parser = subparsers.add_parser("swarm-join", help="Join Docker Swarm cluster (Docker)")
        swarm_join_parser.add_argument(
            "swarm_args", nargs=argparse.REMAINDER, help="Arguments passed to docker swarm join"
        )

        subparsers.add_parser("swarm-leave", help="Leave Docker Swarm cluster (Docker)")

        args = parser.parse_args(argv)
        if not args.command:
            parser.print_help()
            sys.exit(0)

        return args

    def execute_command(self) -> None:
        cmd = self._args.command

        if cmd == "build-base" or (cmd == "build" and self._args.target_image == "base"):
            self.build_base()
        elif cmd == "build-project" or (cmd == "build" and self._args.target_image == "project"):
            self.build_project()
        elif cmd in ["build-all", "build"] and (
            not hasattr(self._args, "target_image") or self._args.target_image == "all"
        ):
            self.build_all()
        elif cmd == "create-network":
            subnet = self._args.net_subnet or self._args.subnet
            self.create_network(subnet)
        elif cmd == "run-project" or (cmd == "run" and self._args.run_type == "project"):
            self.run_project(
                self._args.target_id,
                zenoh_router=self._args.zenoh_router,
                simulator_ip=getattr(self._args, "simulator_ip", None),
            )
        elif cmd in ["run-simulator", "simulator"] or (cmd == "run" and self._args.run_type == "simulator"):
            self.run_simulator(self._args.target_id, zenoh_router=self._args.zenoh_router)
        elif cmd in ["stop-all", "stop"]:
            self.stop_all()
        elif cmd == "ssh":
            self.ssh(self._args.target_id)
        elif cmd in ["net-shell", "netshell"]:
            self.net_shell()
        elif cmd == "connect-host":
            self.connect_host(self._args.host_ip)
        elif cmd == "disconnect-host":
            self.disconnect_host()
        elif cmd == "swarm-init":
            self.swarm_init(self._args.advertise_addr)
        elif cmd == "swarm-join":
            self.swarm_join(self._args.swarm_args)
        elif cmd == "swarm-leave":
            self.swarm_leave()
        else:
            print_error(f"Unknown command '{cmd}'")
            sys.exit(1)

    def build_base(self) -> None:
        self.engine.build_image(
            IMAGE_NAME_BASE,
            DOCKER_DIR / "Containerfile.base",
            REPO_ROOT,
        )

    def build_project(self) -> None:
        key_path = find_ssh_key()
        build_args = {"BASE_IMAGE": IMAGE_NAME_BASE}
        if key_path and key_path.is_file():
            print_info(f"Using SSH key from {key_path}")
            build_args["ssh_pub_key"] = key_path.read_text().strip()
        else:
            print_warning("No SSH public key found in ~/.ssh/id_ed25519.pub or ~/.ssh/id_rsa.pub")
            print_warning("SSH access will not be possible without manual configuration.")

        self.engine.build_image(
            IMAGE_NAME_PROJECT,
            DOCKER_DIR / "Containerfile.project",
            REPO_ROOT,
            build_args=build_args,
        )

    def build_all(self) -> None:
        self.build_base()
        self.build_project()

    def create_network(self, subnet: str | None = None) -> None:
        sub = subnet or self._args.subnet
        self.engine.create_network(NETWORK_NAME, sub)

    def run_project(
        self,
        target: str | None = None,
        zenoh_router: bool = False,
        simulator_ip: str | None = None,
    ) -> None:
        subnet = self._args.subnet
        domain_id = None
        sim_ip = simulator_ip if simulator_ip is not None else getattr(self._args, "simulator_ip", None)
        if target:
            ip = resolve_robot_ip(target, subnet)
            domain_id = resolve_robot_domain_id(target, subnet)
            if not ip:
                print_error(f"Could not find IP for target: {target}")
                sys.exit(1)
            name = f"bitbots-project-{ip.replace('.', '-')}"
            net_args = ["--network", NETWORK_NAME, "--ip", ip]
            self.create_network(subnet)
            print_info(f"Running project container {name} with IP {ip}...")
            print_info(f"You can connect via: ssh {DEFAULT_USER}@{ip}")
        else:
            name = "bitbots-project-run"
            net_args = ["-p", f"{SSH_PORT_PROJECT}:22"]
            print_info(f"Running project container {name} on port {SSH_PORT_PROJECT}...")
            print_info(f"You can connect via: ssh -p {SSH_PORT_PROJECT} {DEFAULT_USER}@localhost")

        env_args = []
        if domain_id:
            env_args.extend(["-e", f"ROS_DOMAIN_ID={domain_id}"])
            print_info(f"ROS_DOMAIN_ID set to {domain_id}")
        if zenoh_router:
            env_args.extend(["-e", "START_ZENOH_ROUTER=1", "-e", "ZENOH_ROUTER=1"])
            print_info("Zenoh router enabled")
        if sim_ip:
            env_args.extend(["-e", f"SIMULATOR_IP={sim_ip}"])
            print_info(f"Simulator target IP set to {sim_ip}")

        gpu_args = self.engine.get_gpu_args(self._args.gpu_args)
        self.engine.run_container(IMAGE_NAME_PROJECT, name, net_args, env_args, gpu_args, detached=True)

    def run_simulator(self, target: str | None = None, zenoh_router: bool = False) -> None:
        subnet = self._args.subnet
        name = "simulator"
        env_args = ["-e", "SIMULATOR=1", "-e", "ROS_DOMAIN_ID=0"]
        if zenoh_router:
            env_args.extend(["-e", "START_ZENOH_ROUTER=1", "-e", "ZENOH_ROUTER=1", "-e", "ZENOH_MODE=router"])
            print_info("Zenoh router enabled")

        if target:
            ip = resolve_robot_ip(target, subnet)
            if not ip:
                print_error(f"Could not find IP for target: {target}")
                sys.exit(1)
            net_args = ["--network", NETWORK_NAME, "--ip", ip, "-p", "8080:8080"]
            self.create_network(subnet)
            print_info(f"Running simulator container {name} with IP {ip}...")
            print_info(f"You can connect via: ssh {DEFAULT_USER}@{ip}")
            print_info("Port 8080 is exposed on localhost:8080")
        else:
            net_args = ["--network", NETWORK_NAME, "-p", "8080:8080"]
            self.create_network(subnet)
            print_info(f"Running simulator container {name} on network {NETWORK_NAME}...")
            print_info("Port 8080 is exposed on localhost:8080")

        gpu_args = self.engine.get_gpu_args(self._args.gpu_args)
        self.engine.run_container(IMAGE_NAME_PROJECT, name, net_args, env_args, gpu_args, detached=True)

    def stop_all(self) -> None:
        self.engine.stop_and_remove_containers()

    def ssh(self, target: str) -> None:
        self.engine.ssh(target, user=DEFAULT_USER)

    def net_shell(self) -> None:
        self.engine.net_shell()

    def connect_host(self, ip: str | None = None) -> None:
        try:
            self.engine.connect_host(ip)
        except NotImplementedError as e:
            print_error(str(e))
            sys.exit(1)

    def disconnect_host(self) -> None:
        try:
            self.engine.disconnect_host()
        except NotImplementedError as e:
            print_error(str(e))
            sys.exit(1)

    def swarm_init(self, advertise_addr: str | None = None) -> None:
        try:
            self.engine.swarm_init(advertise_addr)
        except NotImplementedError as e:
            print_error(str(e))
            sys.exit(1)

    def swarm_join(self, args: list[str]) -> None:
        try:
            self.engine.swarm_join(args)
        except NotImplementedError as e:
            print_error(str(e))
            sys.exit(1)

    def swarm_leave(self) -> None:
        try:
            self.engine.swarm_leave()
        except NotImplementedError as e:
            print_error(str(e))
            sys.exit(1)
