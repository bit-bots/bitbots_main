from __future__ import annotations

import abc
import glob
import os
import shutil
import subprocess
import sys
from pathlib import Path

from manage.misc import (
    DEFAULT_SUBNET,
    DEFAULT_USER,
    IMAGE_NAME_BASE,
    IMAGE_NAME_PROJECT,
    NETWORK_NAME,
    print_debug,
    print_error,
    print_info,
    print_success,
    print_warning,
    resolve_robot_ip,
)


class ContainerEngine(abc.ABC):
    """Abstract base class for container engines (Docker / Podman)."""

    def __init__(self, binary_name: str) -> None:
        self.name = binary_name
        self.binary = shutil.which(binary_name)

    def check_available(self) -> bool:
        """Checks if the container engine binary is found on PATH."""
        if not self.binary:
            print_error(f"Engine '{self.name}' not found on PATH. Please install it or specify another engine.")
            return False
        return True

    def run_cmd(
        self,
        args: list[str],
        check: bool = True,
        capture_output: bool = False,
        text: bool = True,
        **kwargs,
    ) -> subprocess.CompletedProcess:
        """Runs an engine command."""
        cmd = [self.name] + args
        print_debug(f"Running command: {' '.join(cmd)}")
        return subprocess.run(cmd, check=check, capture_output=capture_output, text=text, **kwargs)

    @abc.abstractmethod
    def get_gpu_args(self, custom_gpu_args: str | None = None) -> list[str]:
        """Returns GPU arguments for container run command."""
        pass

    def build_image(
        self,
        image_name: str,
        containerfile_path: Path | str,
        context_path: Path | str,
        build_args: dict[str, str] | None = None,
    ) -> None:
        """Builds an image from Containerfile."""
        if not self.check_available():
            sys.exit(1)

        args = ["build", "-t", image_name]
        if build_args:
            for k, v in build_args.items():
                args.extend(["--build-arg", f"{k}={v}"])

        args.extend(["-f", str(containerfile_path), str(context_path)])
        print_info(f"Building image '{image_name}' with {self.name}...")
        self.run_cmd(args, check=True)
        print_success(f"Successfully built '{image_name}'")

    @abc.abstractmethod
    def create_network(self, network_name: str = NETWORK_NAME, subnet: str = DEFAULT_SUBNET) -> None:
        """Creates container network."""
        pass

    def run_container(
        self,
        image_name: str,
        container_name: str,
        net_args: list[str],
        env_args: list[str],
        gpu_args: list[str],
        detached: bool = True,
    ) -> None:
        """Runs a container with the given settings."""
        if not self.check_available():
            sys.exit(1)

        args = ["run"]
        if detached:
            args.append("-d")
        args.extend(["--name", container_name])
        args.extend(net_args)
        args.extend(env_args)
        args.extend(gpu_args)
        args.append(image_name)

        print_info(f"Starting container '{container_name}' using image '{image_name}' with {self.name}...")
        self.run_cmd(args, check=True)
        print_success(f"Container '{container_name}' started successfully.")

    def find_containers(self, regex_pattern: str = r"^(bitbots-|simulator$)") -> list[str]:
        """Finds container names matching the regex pattern."""
        if not self.check_available():
            return []

        try:
            res = self.run_cmd(["ps", "-a", "--format", "{{.Names}}"], capture_output=True)
            names = res.stdout.strip().splitlines()
            import re

            pattern = re.compile(regex_pattern)
            return [name.strip() for name in names if pattern.search(name.strip())]
        except subprocess.CalledProcessError as e:
            print_debug(f"Failed to list containers: {e}")
            return []

    def stop_and_remove_containers(self, regex_pattern: str = r"^(bitbots-|simulator$)") -> None:
        """Stops and removes all containers matching the regex pattern."""
        if not self.check_available():
            sys.exit(1)

        containers = self.find_containers(regex_pattern)
        if containers:
            print_info(f"Stopping containers: {', '.join(containers)}")
            self.run_cmd(["stop"] + containers, check=False)
            print_info(f"Removing containers: {', '.join(containers)}")
            self.run_cmd(["rm"] + containers, check=False)
            print_success("Containers stopped and removed.")
        else:
            print_info("No Bit-Bots containers found to stop.")

    def inspect_container_ip(self, container_name_or_id: str) -> str | None:
        """Gets the IP address of a container across networks."""
        try:
            res = self.run_cmd(
                ["inspect", "-f", "{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}", container_name_or_id],
                capture_output=True,
            )
            ip = res.stdout.strip()
            return ip if ip else None
        except Exception:
            return None

    @abc.abstractmethod
    def net_shell(self) -> None:
        """Enters the container network namespace shell."""
        pass

    @abc.abstractmethod
    def ssh(self, target: str, user: str = DEFAULT_USER) -> None:
        """SSHs into a container target."""
        pass

    # Optional Docker Swarm & host routing methods (raise NotImplementedError if not supported)
    def swarm_init(self, advertise_addr: str | None = None) -> None:
        raise NotImplementedError(f"swarm-init is not supported on engine '{self.name}'")

    def swarm_join(self, args: list[str]) -> None:
        raise NotImplementedError(f"swarm-join is not supported on engine '{self.name}'")

    def swarm_leave(self) -> None:
        raise NotImplementedError(f"swarm-leave is not supported on engine '{self.name}'")

    def connect_host(self, ip: str | None = None) -> None:
        raise NotImplementedError(f"connect-host is not supported on engine '{self.name}'")

    def disconnect_host(self) -> None:
        raise NotImplementedError(f"disconnect-host is not supported on engine '{self.name}'")


class DockerEngine(ContainerEngine):
    """Docker container engine implementation."""

    def __init__(self) -> None:
        super().__init__("docker")

    def get_gpu_args(self, custom_gpu_args: str | None = None) -> list[str]:
        if custom_gpu_args is not None:
            return custom_gpu_args.split()

        env_args = os.environ.get("GPU_ARGS")
        if env_args:
            return env_args.split()

        args: list[str] = []
        if Path("/dev/dri").is_dir():
            args.extend(["--device", "/dev/dri"])
        if Path("/dev/kfd").exists():
            args.extend(["--device", "/dev/kfd"])

        if shutil.which("nvidia-smi") or Path("/dev/nvidia0").exists() or Path("/dev/nvidiactl").exists():
            args.extend(["--gpus", "all"])

        return args

    def create_network(self, network_name: str = NETWORK_NAME, subnet: str = DEFAULT_SUBNET) -> None:
        if not self.check_available():
            sys.exit(1)

        swarm_state = "inactive"
        try:
            res = self.run_cmd(["info", "--format", "{{.Swarm.LocalNodeState}}"], capture_output=True)
            swarm_state = res.stdout.strip()
        except Exception as e:
            print_debug(f"Could not check swarm state: {e}")

        is_manager = "false"
        try:
            res = self.run_cmd(["info", "--format", "{{.Swarm.ControlAvailable}}"], capture_output=True)
            is_manager = res.stdout.strip()
        except Exception as e:
            print_debug(f"Could not check if node is manager: {e}")

        if swarm_state != "active":
            print_info("Docker Swarm is not active. Initializing Docker Swarm...")
            try:
                self.run_cmd(["swarm", "init"])
                print_success("Docker Swarm initialized successfully.")
                is_manager = "true"
            except subprocess.CalledProcessError:
                print_error(
                    "Failed to initialize Docker Swarm. If you have multiple network interfaces, run: manage.py swarm-init <advertise_addr>"
                )
                sys.exit(1)

        if is_manager == "false":
            print_info(
                f"Connected to Docker Swarm as worker node. Overlay network '{network_name}' is managed by the Swarm manager."
            )
            return

        # Check existing network
        try:
            inspect_res = self.run_cmd(["network", "inspect", "-f", "{{.Driver}}", network_name], capture_output=True)
            driver = inspect_res.stdout.strip()
            if driver == "overlay":
                print_info(f"Attachable overlay network '{network_name}' already exists.")
                return
            else:
                print_warning(f"Network '{network_name}' exists with driver '{driver}'. Recreating as overlay...")
                self.run_cmd(["network", "rm", network_name])
        except subprocess.CalledProcessError:
            pass  # Network does not exist

        print_info(f"Creating attachable overlay network '{network_name}' with subnet '{subnet}'...")
        self.run_cmd(
            ["network", "create", "--driver", "overlay", "--attachable", "--subnet", subnet, network_name], check=True
        )
        print_success(f"Network '{network_name}' created successfully.")

    def swarm_init(self, advertise_addr: str | None = None) -> None:
        if not self.check_available():
            sys.exit(1)

        args = ["swarm", "init"]
        if advertise_addr:
            args.extend(["--advertise-addr", advertise_addr])

        swarm_state = "inactive"
        try:
            res = self.run_cmd(["info", "--format", "{{.Swarm.LocalNodeState}}"], capture_output=True)
            swarm_state = res.stdout.strip()
        except Exception:
            pass

        if swarm_state == "active":
            print_info("Docker Swarm is already active on this node.")
        else:
            print_info("Initializing Docker Swarm...")
            self.run_cmd(args, check=True)
            print_success("Docker Swarm initialized.")

        print_info("To join other PCs to this swarm as worker nodes, run on those PCs:")
        try:
            self.run_cmd(["swarm", "join-token", "worker"])
        except Exception:
            pass
        print_info("To create the shared multi-host overlay network, run: manage.py create-network")

    def swarm_join(self, args: list[str]) -> None:
        if not self.check_available():
            sys.exit(1)
        if not args:
            print_error("Usage: manage.py swarm-join <token> <manager-ip:port>")
            sys.exit(1)

        join_args = ["swarm", "join"]
        if len(args) == 2 and not args[0].startswith("-"):
            join_args.extend(["--token", args[0], args[1]])
        else:
            join_args.extend(args)

        self.run_cmd(join_args, check=True)
        print_success("Joined Docker Swarm successfully.")

    def swarm_leave(self) -> None:
        if not self.check_available():
            sys.exit(1)
        print_info("Leaving Docker Swarm...")
        self.run_cmd(["swarm", "leave", "--force"], check=True)
        print_success("Left Docker Swarm.")

    def connect_host(self, ip: str | None = None) -> None:
        if not self.check_available():
            sys.exit(1)

        ifname = "veth-bb-host"
        peername = "veth-bb-gw"

        try:
            self.run_cmd(["network", "inspect", NETWORK_NAME], capture_output=True)
        except subprocess.CalledProcessError:
            print_error(f"Network '{NETWORK_NAME}' does not exist. Run 'manage.py create-network' first.")
            sys.exit(1)

        # Determine image
        image = IMAGE_NAME_BASE
        try:
            self.run_cmd(["image", "inspect", image], capture_output=True)
        except subprocess.CalledProcessError:
            found = False
            for cand in [IMAGE_NAME_PROJECT, "ubuntu:24.04"]:
                try:
                    self.run_cmd(["image", "inspect", cand], capture_output=True)
                    image = cand
                    found = True
                    break
                except subprocess.CalledProcessError:
                    continue
            if not found:
                print_info(f"Building base image '{IMAGE_NAME_BASE}' first...")
                from manage.misc import DOCKER_DIR, REPO_ROOT

                self.build_image(IMAGE_NAME_BASE, DOCKER_DIR / "Containerfile.base", REPO_ROOT)
                image = IMAGE_NAME_BASE

        # Cleanup existing bitbots-host-gateway
        self.run_cmd(["rm", "-f", "bitbots-host-gateway"], check=False, capture_output=True)

        req_ip = ip.split("/")[0] if ip else "10.66.0.254"
        print_info(f"Starting host gateway container on '{NETWORK_NAME}' with IP {req_ip}...")

        started = False
        try:
            self.run_cmd(
                [
                    "run",
                    "-d",
                    "--name",
                    "bitbots-host-gateway",
                    "--network",
                    NETWORK_NAME,
                    "--ip",
                    req_ip,
                    "--cap-add=NET_ADMIN",
                    "--sysctl",
                    "net.ipv4.ip_forward=1",
                    "--restart",
                    "unless-stopped",
                    image,
                    "sleep",
                    "infinity",
                ],
                capture_output=True,
            )
            started = True
        except subprocess.CalledProcessError:
            print_warning(f"Could not assign IP {req_ip}. Falling back to dynamic IP allocation...")

        if not started:
            self.run_cmd(
                [
                    "run",
                    "-d",
                    "--name",
                    "bitbots-host-gateway",
                    "--network",
                    NETWORK_NAME,
                    "--cap-add=NET_ADMIN",
                    "--sysctl",
                    "net.ipv4.ip_forward=1",
                    "--restart",
                    "unless-stopped",
                    image,
                    "sleep",
                    "infinity",
                ],
                check=True,
            )

        cpid_res = self.run_cmd(["inspect", "-f", "{{.State.Pid}}", "bitbots-host-gateway"], capture_output=True)
        cpid = cpid_res.stdout.strip()
        assigned_ip = self.inspect_container_ip("bitbots-host-gateway") or req_ip

        if not cpid or cpid == "0":
            print_error("Failed to get process ID for bitbots-host-gateway container.")
            sys.exit(1)

        setup_script = f"""
        if ip link show "{ifname}" >/dev/null 2>&1; then
            ip link delete "{ifname}" 2>/dev/null || true
        fi
        ip link add "{ifname}" type veth peer name "{peername}"
        ip link set "{peername}" netns "{cpid}"
        nsenter -t "{cpid}" -n ip link set lo up 2>/dev/null || true
        nsenter -t "{cpid}" -n ip link set "{peername}" up
        nsenter -t "{cpid}" -n ip addr add 10.66.254.2/30 dev "{peername}"
        nsenter -t "{cpid}" -n sysctl -w net.ipv4.ip_forward=1 >/dev/null 2>&1 || true
        nsenter -t "{cpid}" -n iptables -t nat -C POSTROUTING -o eth0 -j MASQUERADE 2>/dev/null || \
            nsenter -t "{cpid}" -n iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE 2>/dev/null || true
        nsenter -t "{cpid}" -n iptables -C FORWARD -i "{peername}" -o eth0 -j ACCEPT 2>/dev/null || \
            nsenter -t "{cpid}" -n iptables -A FORWARD -i "{peername}" -o eth0 -j ACCEPT 2>/dev/null || true
        nsenter -t "{cpid}" -n iptables -C FORWARD -i eth0 -o "{peername}" -m state --state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null || \
            nsenter -t "{cpid}" -n iptables -A FORWARD -i eth0 -o "{peername}" -m state --state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null || true
        ip link set "{ifname}" up
        ip addr add 10.66.254.1/30 dev "{ifname}"
        ip route replace 10.66.0.0/16 via 10.66.254.2 dev "{ifname}"
        """

        if os.geteuid() != 0:
            print_info("Configuring host routing to Docker overlay network requires root privileges (sudo)...")
            subprocess.run(["sudo", "bash", "-c", setup_script], check=True)
        else:
            subprocess.run(["bash", "-c", setup_script], check=True)

        print_success(f"Successfully connected host to '{NETWORK_NAME}' (gateway IP: {assigned_ip} on {ifname}).")
        print_info("The host PC is now directly part of the multi-PC network.")

    def disconnect_host(self) -> None:
        if not self.check_available():
            sys.exit(1)

        ifname = "veth-bb-host"
        containers = self.find_containers(r"^bitbots-host-gateway$")
        if containers:
            print_info("Stopping host gateway container...")
            self.run_cmd(["stop", "bitbots-host-gateway"], check=False, capture_output=True)
            self.run_cmd(["rm", "bitbots-host-gateway"], check=False, capture_output=True)

        res = subprocess.run(["ip", "link", "show", ifname], capture_output=True)
        if res.returncode == 0:
            print_info(f"Disconnecting host interface {ifname}...")
            cmd = ["ip", "link", "delete", ifname]
            if os.geteuid() != 0:
                subprocess.run(["sudo"] + cmd, check=False)
            else:
                subprocess.run(cmd, check=False)
            print_success("Host disconnected from container network.")
        else:
            print_info(f"Host interface {ifname} is not connected.")

    def stop_and_remove_containers(self, regex_pattern: str = r"^(bitbots-|simulator$)") -> None:
        super().stop_and_remove_containers(regex_pattern)
        ifname = "veth-bb-host"
        res = subprocess.run(["ip", "link", "show", ifname], capture_output=True)
        if res.returncode == 0:
            print_info(f"Removing host network interface {ifname}...")
            cmd = ["ip", "link", "delete", ifname]
            if os.geteuid() != 0:
                subprocess.run(["sudo"] + cmd, check=False)
            else:
                subprocess.run(cmd, check=False)

    def _find_overlay_netns(self) -> str | None:
        try:
            res = self.run_cmd(["network", "inspect", "-f", "{{.Id}}", NETWORK_NAME], capture_output=True)
            net_id = res.stdout.strip()
            if not net_id:
                return None
        except Exception:
            return None

        search_script = """
        net_id="$1"
        dirs=("/var/run/docker/netns" "/run/docker/netns" "/var/run/netns" "/run/netns")
        for dir in "${dirs[@]}"; do
            [ -d "$dir" ] || continue
            for len in 9 10 8 12 16 64; do
                short="${net_id:0:$len}"
                [ -n "$short" ] || continue
                if [ -f "$dir/1-$short" ]; then
                    echo "$dir/1-$short"
                    exit 0
                fi
                if [ -f "$dir/$short" ]; then
                    echo "$dir/$short"
                    exit 0
                fi
            done
            short8="${net_id:0:8}"
            if [ -n "$short8" ]; then
                for f in "$dir"/*"$short8"*; do
                    if [ -f "$f" ] && [[ "$(basename "$f")" != lb_* ]]; then
                        echo "$f"
                        exit 0
                    fi
                done
            fi
        done
        for dir in "${dirs[@]}"; do
            [ -d "$dir" ] || continue
            for f in "$dir"/1-*; do
                if [ -f "$f" ] && nsenter --net="$f" ip link show br0 >/dev/null 2>&1; then
                    echo "$f"
                    exit 0
                fi
            done
        done
        exit 1
        """
        cmd = ["bash", "-c", search_script, "--", net_id]
        if os.geteuid() != 0:
            res = subprocess.run(["sudo"] + cmd, capture_output=True, text=True)
        else:
            res = subprocess.run(cmd, capture_output=True, text=True)

        if res.returncode == 0 and res.stdout.strip():
            return res.stdout.strip()
        return None

    def net_shell(self) -> None:
        if not self.check_available():
            sys.exit(1)

        # 1. Check if bitbots-host-gateway is running
        try:
            res = self.run_cmd(["ps", "--format", "{{.Names}}"], capture_output=True)
            if "bitbots-host-gateway" in res.stdout.splitlines():
                print_info("Entering Bit-Bots network shell via host gateway...")
                subprocess.run([self.name, "exec", "-it", "bitbots-host-gateway", "bash"])
                return
        except Exception:
            pass

        # 2. Check if any container on NETWORK_NAME is running
        try:
            res = self.run_cmd(["ps", "--filter", f"network={NETWORK_NAME}", "-q"], capture_output=True)
            cids = res.stdout.strip().splitlines()
            if cids:
                cid = cids[0]
                name_res = self.run_cmd(["inspect", "-f", "{{.Name}}", cid], capture_output=True)
                cname = name_res.stdout.strip().lstrip("/")
                print_info(f"Entering Bit-Bots network shell via container '{cname}'...")
                subprocess.run([self.name, "exec", "-it", cid, "bash"])
                return
        except Exception:
            pass

        # 3. Check overlay netns
        netns_file = self._find_overlay_netns()
        if netns_file:
            print_info(f"Entering Docker Swarm overlay network namespace ({netns_file})...")
            cmd = ["nsenter", f"--net={netns_file}", "bash"]
            if os.geteuid() != 0:
                subprocess.run(["sudo"] + cmd)
            else:
                subprocess.run(cmd)
            return

        print_info("---------------------------------------------------------")
        print_info("  BIT-BOTS NETWORK SHELL")
        print_info("---------------------------------------------------------")
        print_info(f"No running container found on '{NETWORK_NAME}'.")
        print_info("Start a container or connect the host first:")
        print_info("  - Connect host:  manage.py connect-host")
        print_info("  - Run project:   manage.py run-project <name>")
        print_info("---------------------------------------------------------")
        env = os.environ.copy()
        env["BITBOTS_NET_SHELL"] = "1"
        shell = os.environ.get("SHELL", "bash")
        subprocess.run([shell], env=env)

    def ssh(self, target: str, user: str = DEFAULT_USER) -> None:
        if not target:
            print_error("Usage: manage.py ssh <hostname|robot_name|IP>")
            sys.exit(1)

        ip = resolve_robot_ip(target)
        if not ip:
            ip = self.inspect_container_ip(target)
        if not ip:
            print_error(f"Could not find IP for target: {target}")
            sys.exit(1)

        ssh_opts = ["-o", "StrictHostKeyChecking=no", "-o", "UserKnownHostsFile=/dev/null", "-o", "LogLevel=ERROR"]

        # Check if directly reachable
        ping_ok = subprocess.run(["ping", "-c", "1", "-W", "1", ip], capture_output=True).returncode == 0
        if os.environ.get("BITBOTS_NET_SHELL") == "1" or os.geteuid() == 0 or ping_ok:
            cmd = ["ssh"] + ssh_opts + [f"{user}@{ip}"]
            subprocess.run(cmd)
        else:
            # Proxy through container on network
            ip_slug = ip.replace(".", "-")
            cid = None
            try:
                res = self.run_cmd(["ps", "--format", "{{.ID}} {{.Names}}"], capture_output=True)
                for line in res.stdout.splitlines():
                    if ("bitbots" in line or "simulator" in line) and ip_slug in line:
                        cid = line.split()[0]
                        break
            except Exception:
                pass

            if not cid:
                try:
                    res = self.run_cmd(["ps", "--filter", f"network={NETWORK_NAME}", "-q"], capture_output=True)
                    cids = res.stdout.strip().splitlines()
                    if cids:
                        cid = cids[0]
                except Exception:
                    pass

            if cid:
                proxy_cmd = f"docker exec -i {cid} nc {ip} 22"
                cmd = ["ssh"] + ssh_opts + ["-o", f"ProxyCommand={proxy_cmd}", f"{user}@{ip}"]
                subprocess.run(cmd)
            else:
                print_error(
                    f"Container with IP {ip} not reachable and no local proxy container found on '{NETWORK_NAME}'."
                )
                print_info("Options to enable connectivity:")
                print_info("  1. Run 'manage.py connect-host' with sudo to connect host directly.")
                print_info("  2. Run 'manage.py net-shell' to open a shell inside the overlay network.")
                print_info("  3. Start a container on this machine to enable proxying.")
                sys.exit(1)


class PodmanEngine(ContainerEngine):
    """Podman container engine implementation."""

    def __init__(self) -> None:
        super().__init__("podman")

    def get_gpu_args(self, custom_gpu_args: str | None = None) -> list[str]:
        if custom_gpu_args is not None:
            return custom_gpu_args.split()

        env_args = os.environ.get("GPU_ARGS")
        if env_args:
            return env_args.split()

        args: list[str] = []
        if Path("/dev/dri").is_dir():
            args.extend(["--device", "/dev/dri"])
        if Path("/dev/kfd").exists():
            args.extend(["--device", "/dev/kfd"])

        has_cdi = False
        cdi_dirs = [
            "/etc/cdi",
            "/var/run/cdi",
            "/etc/containers/cdi",
            "/var/run/containers/cdi",
            os.path.expanduser("~/.config/cdi"),
            os.path.expanduser("~/.config/containers/cdi"),
        ]
        for cdi_dir in cdi_dirs:
            if Path(cdi_dir).is_dir():
                for root, _, files in os.walk(cdi_dir):
                    for file in files:
                        filepath = Path(root) / file
                        try:
                            if "nvidia.com/gpu" in filepath.read_text():
                                has_cdi = True
                                break
                        except Exception:
                            pass
                    if has_cdi:
                        break
            if has_cdi:
                break

        if has_cdi:
            args.extend(["--gpus", "all"])
        else:
            for dev in glob.glob("/dev/nvidia*"):
                if Path(dev).exists():
                    args.extend(["--device", dev])
            if Path("/dev/nvidia-caps").is_dir():
                for dev in glob.glob("/dev/nvidia-caps/*"):
                    if Path(dev).exists():
                        args.extend(["--device", dev])

        return args

    def create_network(self, network_name: str = NETWORK_NAME, subnet: str = DEFAULT_SUBNET) -> None:
        if not self.check_available():
            sys.exit(1)

        try:
            self.run_cmd(["network", "inspect", network_name], capture_output=True)
            print_info(f"Network '{network_name}' already exists.")
        except subprocess.CalledProcessError:
            print_info(f"Creating network '{network_name}' with subnet '{subnet}'...")
            self.run_cmd(["network", "create", "--subnet", subnet, network_name], check=True)
            print_success(f"Network '{network_name}' created successfully.")

    def net_shell(self) -> None:
        if not self.check_available():
            sys.exit(1)

        print_info("Entering rootless network namespace...")
        print_info("Hiding problematic system SSH config files to avoid permission errors...")

        netns_script = """
        if [ -d /etc/ssh/ssh_config.d ]; then
            mount -t tmpfs tmpfs /etc/ssh/ssh_config.d
        fi
        if [ -f /etc/ssh/ssh_config ] && [ "$(stat -c %u /etc/ssh/ssh_config)" != "0" ]; then
            mount --bind /dev/null /etc/ssh/ssh_config
        fi
        mount -t tmpfs tmpfs /root
        if [ -d "$HOME/.ssh" ]; then
            mkdir -p /root/.ssh
            mount --bind "$HOME/.ssh" /root/.ssh
        fi
        echo '---------------------------------------------------------'
        echo '  BIT-BOTS NETWORK SHELL'
        echo '---------------------------------------------------------'
        echo 'You are now in the container network namespace.'
        echo 'Container IPs are directly reachable.'
        echo 'Type "exit" to return to your normal host shell.'
        echo '---------------------------------------------------------'
        export BITBOTS_NET_SHELL=1
        exec ${SHELL:-bash}
        """
        subprocess.run(["podman", "unshare", "--rootless-netns", "bash", "-c", netns_script])

    def ssh(self, target: str, user: str = DEFAULT_USER) -> None:
        if not target:
            print_error("Usage: manage.py ssh <hostname|robot_name|IP>")
            sys.exit(1)

        ip = resolve_robot_ip(target)
        if not ip:
            ip = self.inspect_container_ip(target)
        if not ip:
            print_error(f"Could not find IP for target: {target}")
            sys.exit(1)

        ssh_opts = ["-o", "StrictHostKeyChecking=no", "-o", "UserKnownHostsFile=/dev/null", "-o", "LogLevel=ERROR"]

        if os.environ.get("BITBOTS_NET_SHELL") == "1" or os.geteuid() == 0:
            cmd = ["ssh"] + ssh_opts + [f"{user}@{ip}"]
            subprocess.run(cmd)
        else:
            ip_slug = ip.replace(".", "-")
            cid = None
            try:
                res = self.run_cmd(["ps", "--format", "{{.ID}} {{.Names}}"], capture_output=True)
                for line in res.stdout.splitlines():
                    if ("bitbots" in line or "simulator" in line) and ip_slug in line:
                        cid = line.split()[0]
                        break
            except Exception:
                pass

            if not cid:
                try:
                    res = self.run_cmd(
                        ["ps", "--filter", f"network={NETWORK_NAME}", "--filter", f"name={target}", "-q"],
                        capture_output=True,
                    )
                    cids = res.stdout.strip().splitlines()
                    if cids:
                        cid = cids[0]
                except Exception:
                    pass

            if cid:
                proxy_cmd = f"podman exec -i {cid} nc localhost 22"
                cmd = ["ssh"] + ssh_opts + ["-o", f"ProxyCommand={proxy_cmd}", f"{user}@{ip}"]
                subprocess.run(cmd)
            else:
                print_error(f"Container with IP {ip} not found or not running.")
                print_info(f"Ensure the container is started and attached to '{NETWORK_NAME}'.")
                sys.exit(1)


def get_engine(engine_name: str = "docker") -> ContainerEngine:
    """Factory function to get the ContainerEngine instance."""
    engine_name = engine_name.lower().strip()
    if engine_name == "docker":
        return DockerEngine()
    elif engine_name == "podman":
        return PodmanEngine()
    else:
        raise ValueError(f"Unknown container engine: '{engine_name}'. Supported engines are 'docker' and 'podman'.")
