#!/usr/bin/env python3

from typing import Dict, List, Optional

import argparse
import ipaddress
import os
import subprocess
import sys
import yaml

from collections import defaultdict

from fabric import Connection, Result
from rich import box
from rich.console import Console
from rich.panel import Panel
from rich.prompt import Prompt

BITBOTS_META = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SYNC_INCLUDES_FILE = os.path.join(BITBOTS_META, f"sync_includes_wolfgang_nuc.yaml")

CONSOLE = Console()
CURRENT_CONNECTION: Optional[Connection] = None


class LOGLEVEL:
    ERR_SUCCESS = 0
    WARN = 1
    INFO = 2
    DEBUG = 3
    CURRENT = 2


class Target:
    # Map robot names to hostnames
    _robotnames: Dict[str, str] = {
        "amy": "nuc1",
        "rory": "nuc2",
        "jack": "nuc3",
        "donna": "nuc4",
        "melody": "nuc5",
        "rose": "nuc6",
    }

    # Map hostnames to workspaces
    _workspaces = defaultdict(lambda: "~/colcon_ws")
    _workspaces["nuc1"] = "~/colcon_ws"
    _workspaces["nuc2"] = "~/colcon_ws"
    _workspaces["nuc3"] = "~/colcon_ws"
    _workspaces["nuc4"] = "~/colcon_ws"
    _workspaces["nuc5"] = "~/colcon_ws"
    _workspaces["nuc6"] = "~/colcon_ws"

    _IP_prefix = "172.20.1."
    # Map hostnames to IPs
    _IPs = {
        "nuc1": ipaddress.ip_address(_IP_prefix + "11"),
        "nuc2": ipaddress.ip_address(_IP_prefix + "12"),
        "nuc3": ipaddress.ip_address(_IP_prefix + "13"),
        "nuc4": ipaddress.ip_address(_IP_prefix + "14"),
        "nuc5": ipaddress.ip_address(_IP_prefix + "15"),
        "nuc6": ipaddress.ip_address(_IP_prefix + "16"),
    }

    def __init__(self, identifier: str) -> None:
        """
        Target represents a robot to deploy to.
        It can be initialized with a hostname, IP or robot name.
        """
        self.hostname: Optional[str] = None
        self.ip: Optional[ipaddress.IPv4Address | ipaddress.IPv6Address] = None

        # Is identifier an IP?
        try:
            self.ip = ipaddress.ip_address(identifier)
            # Infer hostname from IP
            for name, known_ip in self._IPs.items():
                if known_ip == self.ip:
                    self.hostname = name
        except ValueError:
            self.ip = None

        # Is identifier a hostname?
        if identifier in self._IPs.keys():
            self.hostname = identifier

        # Is identifier a robot name?
        if identifier in self._robotnames.keys():
            self.hostname = self._robotnames[identifier]

        if self.hostname is not None and self.hostname in self._IPs.keys():
            self.ip = self._IPs[self.hostname]
        else:
            raise ValueError("Could not determine hostname or IP from input: '{identifier}'")
        
        self.workspace = self._workspaces[self.hostname]

    def __str__(self) -> str:
        """Returns the target's hostname if available or IP-address."""
        return self.hostname if self.hostname is not None else str(self.ip)


def _should_run_quietly() -> bool:
    """
    Returns whether the task should run quietly or not.
    
    :return: True if the current loglevel is below INFO, False otherwise.
    """
    return LOGLEVEL.CURRENT <= LOGLEVEL.INFO


def print_err(msg) -> None:
    """Prints an error message in a red box to the console."""
    if LOGLEVEL.CURRENT >= LOGLEVEL.ERR_SUCCESS:
        CONSOLE.print(Panel(msg, title="Error", style="bold red", box=box.HEAVY))


def print_warn(msg) -> None:
    """Prints a warning message in a yellow box to the console."""
    if LOGLEVEL.CURRENT >= LOGLEVEL.WARN:
        CONSOLE.print(Panel(msg, title="Warning", style="yellow", box=box.SQUARE))


def print_success(msg) -> None:
    """Prints a success message in a green box to the console."""
    if LOGLEVEL.CURRENT >= LOGLEVEL.ERR_SUCCESS:
        CONSOLE.print(Panel(msg, title="Success", style="green", box=box.SQUARE))


def print_info(msg) -> None:
    """Prints an info message to the console."""
    if LOGLEVEL.CURRENT >= LOGLEVEL.INFO:
        CONSOLE.log(msg, style="")


def print_debug(msg) -> None:
    """Prints a debug message to the console."""
    if LOGLEVEL.CURRENT >= LOGLEVEL.DEBUG:
        CONSOLE.log(msg, style="dim")


def print_bit_bot() -> None:
    """Prints the Bit-Bots logo to the console."""
    print("""\033[1m
                `/shNMoyymmmmmmmmmmys+NmNs/`
              `+mmdddmmmddddddddddddmmmdddmm/
              ymdddddddmmmmmmmmmmmmmmdddddddms
            .dmdddddddddddmddddddmmddddddddddmy`
           .mmdddddddddddmyoooooosddddddddddddms`
       .ssshmdddddddddddddmhsooshddddddddddddddmhsss.
    -+shysshddddmddddmdddddddddddddddmdhhhhdmdddhssyhs+-
   :ddyyyyydddy+/////+shddddddddddddy+//////+sdddyyyyydd:
  `mmh/+hdddh///////////sdddddddddd+///++++////hdddh+/hmm`
  omdddddmmy///sdNNNmy///omddddddd+//odNMMNmo///hdmdddddmo
 `dddddddmm+//yMMMMMMMh///hdddmddh//oNMMMMMMNo//smmddddddd`
 .Nddddddmm+//hMMMMMMMd///hdhNNddh//+NMMMMMMNo//ymmddddddN.
 .mNNNNNNNdh///yNMMMNd+//sdddddddds//+hmNNmho//oddmmNNNNNm.
-ydddmmmmNmdh+///+++////yddddddddddy//////////sddmNmmmmdddy-
:sddddddymNdddyo+////oyddddddddddddddys++++oyddddddydddddds:
/hddddddh::Nddddmdddmdddddddddddddddddddmmdddddmm.:hdddddhh/
:dhhhhyhh. :dmddddddddddddddddddddddddddddddddmy. .hhyhhhhd:
/mhhhhyo/   `omdddddddddddddddddddddddddddddmmo`   /oyhhhhm/
/hhhhd`       `ommddddddddddddddddddddddddmmo`       `dhhhh/
/hhhhd+         `+ymdddddddddddddddddddddy:`         +dhhhh/
mhhhhds          +:mhhdhmmmmmddmmmmNmhhhs/:          sdhhhhm
mhhhhm.         .mNhsshmN.-//:///-hNdyoydMm          .mhhhhm
mhhhhs          oNNy//ymN`        yNho:omNd           shhhhm
dhhss`          +NMNsyNNN`        yNNm+dNNd           `sshhd
dhdh/.          +NNdyyNNm`        yNNdyhNNd           ./hdhd
mhssy+          +dddddddm`        ydddddddd           +ysshm
hy-             +dddddddm`        ydddddddd              -yh
               `hmmmmmmmm+       -dmmmmmmmm:
               `Nddddddddy       :mdddddddm+
               `Nddddddddy       :mdddddddm+
               `Ndddddddmy       :mdddddddm+
               `mddddddddy       :mdddddddd+
               `mddddddddy       :mdddddddd+
               `mmNmohNNdy       :mmNmohmNm+
               `NNmdhdmNmh       /NmmdhddNms
              `/mms+-/ymm:        ymho-:odmh/.
              yo+hsoooshs         -hsooosys+y:
              h+++++++oo+         `ho+++++++s:
              dysssssssy+         `dsssssssyh:
\033[0m""")


def _parse_arguments() -> argparse.Namespace:
    """
    Parses the command line arguments.
    
    :return: The parsed arguments.
    """
    parser = argparse.ArgumentParser(
        description="Deploy, configure, and launch the Bit-Bots software on a robot."
        )

    # Positional arguments
    parser.add_argument(
        "target",
        type=str,
        help="The target robot or computer you want to compile for. Multiple targets can be specified seperated by commas. 'ALL' can be used to target all known robots."
        )

    # Optional mode
    mode = parser.add_mutually_exclusive_group(required=False)
    mode.add_argument("-s", "--sync-only", action="store_true", help="Only synchronize (copy) files from you to the target machine")
    mode.add_argument("-c", "--configure-only", action="store_true", help="Only configure the target machine")
    mode.add_argument("-b", "--build-only", action="store_true", help="Only build on the target machine")

    # Optional arguments
    parser.add_argument("-p", "--package", default='', help="Synchronize and build only the given ROS package")
    parser.add_argument("--clean-src", action="store_true", help="Clean source directory before syncing")
    parser.add_argument("--no-rosdeps", action="store_false", default=True, dest="install_rosdeps", help="Don't install ROS dependencies on the target.")
    parser.add_argument("--clean-build", action="store_true", help="Clean workspace before building. If --package is given, clean only that package")
    parser.add_argument("-l", "--launch-teamplayer", action="store_true", help="Launch teamplayer software on the target")
    parser.add_argument("-B", "--print-bit-bot", action="store_true", default=False, help="Print our logo at script start")
    parser.add_argument("-v", "--verbose", action="count", default=0, help="More output")
    parser.add_argument("-q", "--quiet", action="count", default=0, help="Less output")
    return parser.parse_args()


def _get_targets(input_targets: str) -> List[Target]:
    """
    Parse target argument into usable Targets.
    Targets are comma seperated and can be either hostnames, robot names or IPs
    'ALL' is a valid target and will be expanded to all known targets

    :param input_targets: Comma seperated list of targets
    :return: List of Targets
    """
    targets: List[Target] = []

    if input_targets == "ALL":
        for hostname in Target._IPs.keys():
            targets.append(Target(hostname))
        return targets

    for input_target in input_targets.split(","):
        try:
            target = Target(input_target)
        except ValueError:
            print_err(f"Could not determine hostname or IP from input: '{input_target}'")
            exit(1)
        targets.append(target)
    return targets


def _get_includes_from_file(file_path: str, package: str = '') -> List[str]:
    """
    Retrieve a list of file to sync from and includes-file

    :param file_path: Path of the includes-file
    :param package: Limit to file from this package, if empty, all files are included
    :returns: List of files to sync
    """
    includes = list()
    with open(file_path) as file:
        data = yaml.safe_load(file)
        # Exclude files
        for entry in data['exclude']:
            includes.append(f'--include=- {entry}')
        # Include files
        for entry in data['include']:
            if isinstance(entry, dict):
                for folder, subfolders in entry.items():
                    if package == '':
                        includes.append(f'--include=+ {folder}')
                        for subfolder in subfolders:
                            includes.append(f'--include=+ {folder}/{subfolder}')
                            includes.append(f'--include=+ {folder}/{subfolder}/**')
                    elif package in subfolders:
                        includes.append(f'--include=+ {folder}')
                        includes.append(f'--include=+ {folder}/{package}')
                        includes.append(f'--include=+ {folder}/{package}/**')
            elif isinstance(entry, str):
                if package == '' or package == entry:
                    includes.append(f'--include=+ {entry}')
                    includes.append(f'--include=+ {entry}/**')
    includes.append('--include=- *')
    return includes


def _get_connection(target: Target) -> Connection:
    """
    Get a connection to the given Target using the 'bitbots' username.
    
    :param target: The target to connect to
    :return: The connection
    """
    return Connection(host=str(target), user="bitbots")


def _execute_on_target(target: Target, command: str, hide: Optional[str | bool] = None) -> Result:
    """
    Execute a command on the given Target over the current connection.
    
    :param target: The Target to execute the command on
    :param command: The command to execute
    :param hide: Whether to hide the output of the command
    :return: The result of the command
    """
    global CURRENT_CONNECTION
    if CURRENT_CONNECTION is None:
        print_err(f"No connection available to {target.hostname}.")
        sys.exit(1)

    if hide is None:
        hide = 'stdout' if _should_run_quietly() else None
    return CURRENT_CONNECTION.run(command, hide=hide)


def _internet_available_on_target(target: Target) -> bool:
    """
    Check if the Target has an internet connection by pinging apt repos.
    
    :param target: The Target to check for an internet connection
    :return: Whether the Target has an internet connection
    """
    print_info(f"Checking internet connection on {target.hostname}")

    apt_mirror = "de.archive.ubuntu.com"
    return _execute_on_target(target, f"timeout --foreground 0.5 curl -sSLI {apt_mirror}").ok


def sync(target: Target, package: str = '', pre_clean=False) -> None:
    """
    Synchronize (copy) the local source directory to the given Target using the rsync tool.

    :param target: The Target to sync to
    :param package: Limit to this package, if empty, all packages are synced
    :param pre_clean: Whether to clean the source directory before syncing
    """
    if pre_clean and package:
        print_warn("Cleaning selected packages is not supported. Will clean all packages instead.")

    if pre_clean:
        print_info(f"Cleaning source directory on {target.hostname}")
        clean_result = _execute_on_target(target, f"rm -rf {target.workspace}/src && mkdir -p {target.workspace}/src")
        if not clean_result.ok:
            print_warn(f"Cleaning of source directory on {target.hostname} failed. Continuing anyways")

    cmd = [
        "rsync",
        "--checksum",
        "--archive",
        "--delete",
    ]

    if not _should_run_quietly():
        cmd.append("--verbose")

    cmd.extend(_get_includes_from_file(SYNC_INCLUDES_FILE, package))
    cmd.extend([BITBOTS_META + "/", f"bitbots@{target}:{target.workspace}/src/"])

    print_debug(f"Calling {' '.join(cmd)}")
    sync_result = subprocess.run(cmd)
    if sync_result.returncode != 0:
        print_err(f"Synchronizing {target.hostname} failed with error code {sync_result.returncode}")
        sys.exit(sync_result.returncode)


def install_rosdeps(target: Target) -> None:
    """
    Install ROS dependencies using the rosdep tool on the given Target.

    :param target: The Target to install the ROS dependencies on
    """
    if _internet_available_on_target(target):
        print_info(f"Installing rosdeps on {target.hostname}")
        target_src_path = os.path.join(target.workspace, "src")

        cmd = f"rosdep install -y --ignore-src --from-paths {target_src_path}"

        rosdep_result = _execute_on_target(target, cmd)
        if not rosdep_result.ok:
            print_warn(f"Rosdep install on {target.hostname} exited with code {rosdep_result.exited}. Check its output for more info")
    else: 
        print_info(f"Skipping rosdep install on {target.hostname} as we do not have internet")


def configure_game_settings(target: Target) -> None:
    """
    Configure the game settings on the given Target with user input.
    This tries to run the game_settings.py script on the Target.
    
    :param target: The Target to configure the game settings on
    """
    result_game_settings = _execute_on_target(target, f"python3 {target.workspace}/src/bitbots_misc/bitbots_utils/bitbots_utils/game_settings.py", hide=False)
    if not result_game_settings.ok:
        print_err(f"Game settings on {target.hostname} failed")
        sys.exit(result_game_settings.exited)
    print_info(f"Configured game settings on {target.hostname}")


def configure_wifi(target: Target) -> None:
    """
    Configure the wifi on the given Target with user input.
    The user is shown a list of available wifi networks and
    can choose one by answering with the according UUID.
    All other connections are disabled and depriorized.

    :param target: The Target to configure the wifi on
    """
    _execute_on_target(target, "nmcli connection show", hide=False)
    connection_id = Prompt.ask("UUID or name of connection which should be enabled [leave unchanged]", console=CONSOLE)

    if connection_id != "":
        # disable all other connections
        connection_ids = str(_execute_on_target(target, "nmcli --fields UUID,TYPE connection show | grep wifi | awk '{print $1}'").stdout).strip().split("\n")
        for i in connection_ids:
            if not _execute_on_target(target, f'sudo nmcli connection modify {i} connection.autoconnect FALSE').ok:
                print_warn(f"Could not disable connection {i} on {target.hostname}")
            if not _execute_on_target(target, f'sudo nmcli connection modify {i} connection.autoconnect-priority 0').ok:
                print_warn(f"Could not set priority of connection {i} to 0 on {target.hostname}")

        result_enable_connection = _execute_on_target(target, f"sudo nmcli connection up {connection_id}")
        if not result_enable_connection.ok:
            print_err(f"Could not enable connection {connection_id} on {target.hostname}")
            sys.exit(result_enable_connection.exited)
        
        result_set_autoconnect = _execute_on_target(target, f"sudo nmcli connection modify {connection_id} connection.autoconnect TRUE")
        if not result_set_autoconnect.ok:
            print_err(f"Could not enable connection {connection_id} on {target.hostname}")
            sys.exit(result_set_autoconnect.exited)
        result_set_priority = _execute_on_target(target, f"sudo nmcli connection modify {connection_id} connection.autoconnect-priority 100")
        if not result_set_priority.ok:
            print_err(f"Could not set priority of connection {connection_id} to 100 on {target.hostname}")
            sys.exit(result_set_priority.exited)
    print_info(f"Configured wifi on {target.hostname}")


def configure(target: Target) -> None:
    """
    Configure the given Target with user input.
    This includes configuring the game settings and the wifi.

    :param target: The Target to configure
    """
    configure_game_settings(target)
    configure_wifi(target)


def build(target: Target, package: str = '', pre_clean: bool = False) -> None:
    """
    Build the source code using colcon on the given Target.
    If no package is given, all packages are built.

    :param target: The Target to build the package on
    :param package: The package to build, if empty all packages are built
    :param pre_clean: Whether to clean the build directory before building
    """
    if package and pre_clean:
        cmd_clean = f"colcon clean packages -y --packages-select {package}"
    elif pre_clean:
        cmd_clean = 'rm -rf build install log;'
    else:
        cmd_clean = ' '

    package_option = f"--packages-up-to {package}" if package else ''
    cmd = (
        "sync;"
        f"cd {target.workspace};"
        "source /opt/ros/rolling/setup.zsh;"
        "source install/setup.zsh;"
        f"{cmd_clean};"
        "ISOLATED_CPUS=\"$(grep -oP 'isolcpus=\K([\d,]+)' /proc/cmdline)\";"
        f"chrt -r 1 taskset -c ${{ISOLATED_CPUS:-0-15}} colcon build --symlink-install {package_option} --continue-on-error || exit 1;"
        "sync;"
    )

    build_result = _execute_on_target(target, cmd)
    if not build_result.ok:
        print_err(f"Build on {target.hostname} failed")
        sys.exit(build_result.exited)


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


def main() -> None:
    """
    Main entry point of the script.
    Parses the arguments and runs the tasks on given Targets.
    """
    args = _parse_arguments()

    LOGLEVEL.CURRENT = LOGLEVEL.CURRENT + args.verbose - args.quiet

    if args.print_bit_bot:
        print_bit_bot()

    # Determine which tasks to run
    do_sync = args.sync_only or not (args.configure_only or args.build_only)
    do_install_rosdep = args.install_rosdep and not (args.sync_only or args.configure_only or args.build_only)
    do_configure = args.configure_only or not (args.sync_only or args.build_only)
    do_build = args.build_only or not (args.sync_only or args.configure_only)
    do_launch_teamplayer = args.launch_teamplayer and not (args.sync_only or args.configure_only or args.build_only)

    num_tasks = sum([do_sync, do_install_rosdep, do_configure, do_build, do_launch_teamplayer]) + 1  # +1 for connection

    # Run tasks for each target
    targets = _get_targets(args.target)
    for target in targets:
        current_task = 1

        # Get connection
        with CONSOLE.status(f"[bold blue][TASK {current_task}/{num_tasks}] Connecting to {target.hostname} via SSH", spinner="point"):
            global CURRENT_CONNECTION
            CURRENT_CONNECTION = _get_connection(target)
            # TODO: Check if connection is successful
        print_success(f"[TASK {current_task}/{num_tasks}] Connected to {target.hostname}")
        current_task += 1

        if do_sync:
            with CONSOLE.status(f"[bold blue][TASK {current_task}/{num_tasks}] Syncing to {target.hostname}", spinner="point"):
                sync(target, args.package, pre_clean=args.clean_src)
            print_success(f"[TASK {current_task}/{num_tasks}] Synchronization of {target.hostname} successful")
            current_task += 1

        if do_install_rosdep:
            with CONSOLE.status(f"[bold blue][TASK {current_task}/{num_tasks}] Installing ROS dependencies on {target.hostname}", spinner="point"):
                install_rosdeps(target)
            print_success(f"[TASK {current_task}/{num_tasks}] Installation of ROS dependencies on {target.hostname} successful")
            current_task += 1

        if do_configure:
            # DO NOT run this in a status, as it screws up the user input
            configure(target)
            print_success(f"[TASK {current_task}/{num_tasks}] Configuration of {target.hostname} successful")
            current_task += 1


        if do_build:
            with CONSOLE.status(f"[bold blue][TASK {current_task}/{num_tasks}] Compiling on {target.hostname}", spinner="point"):
                build(target, args.package, pre_clean=args.clean_build)
            print_success(f"[TASK {current_task}/{num_tasks}] Compilation on {target.hostname} successful")
            current_task += 1

        if do_launch_teamplayer:
            with CONSOLE.status(f"[bold blue][TASK {current_task}/{num_tasks}] Launching teamplayer on {target.hostname}", spinner="point"):
                launch_teamplayer(target)
            print_success(f"[TASK {current_task}/{num_tasks}] Launching teamplayer on {target.hostname} successful")
        
        # close connection
        CURRENT_CONNECTION.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print_err("Interrupted by user")
        sys.exit(1)
