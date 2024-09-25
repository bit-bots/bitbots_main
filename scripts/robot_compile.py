#!/usr/bin/env python3

import argparse
import ipaddress
import os
import subprocess
import sys

import yaml

print(
    DeprecationWarning(
        "WARNING: This script is deprecated. Use 'deploy_robot.py' instead. Please remove this script in the future."
    )
)


class LOGLEVEL:
    current = 2
    DEBUG = 3
    INFO = 2
    WARN = 1
    ERR_SUCCESS = 0


BITBOTS_MAIN = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def print_err(msg):
    if LOGLEVEL.current >= LOGLEVEL.ERR_SUCCESS:
        print("\033[91m\033[1m##" + "".join(["#"] * len(str(msg))) + "##\033[0m")
        print("\033[91m\033[1m# " + "".join([" "] * len(str(msg))) + " #\033[0m")
        print("\033[91m\033[1m# " + str(msg) + " #\033[0m")
        print("\033[91m\033[1m# " + "".join([" "] * len(str(msg))) + " #\033[0m")
        print("\033[91m\033[1m##" + "".join(["#"] * len(str(msg))) + "##\033[0m")


def print_warn(msg):
    if LOGLEVEL.current >= LOGLEVEL.WARN:
        print("\033[93m\033[1m# " + "".join([" "] * len(str(msg))) + " #\033[0m")
        print("\033[93m\033[1m# " + str(msg) + " #\033[0m")
        print("\033[93m\033[1m# " + "".join([" "] * len(str(msg))) + " #\033[0m")


def print_success(msg):
    if LOGLEVEL.current >= LOGLEVEL.ERR_SUCCESS:
        print("\033[92m\033[1m# " + "".join([" "] * len(str(msg))) + " #\033[0m")
        print("\033[92m\033[1m# " + str(msg) + " #\033[0m")
        print("\033[92m\033[1m# " + "".join([" "] * len(str(msg))) + " #\033[0m")


def print_info(msg):
    if LOGLEVEL.current >= LOGLEVEL.INFO:
        print("\033[96m" + str(msg) + "\033[0m")


def print_debug(msg):
    if LOGLEVEL.current >= LOGLEVEL.DEBUG:
        print(msg)


def print_bit_bot():
    print(
        """\033[1m
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
\033[0m"""
    )


class Target:
    class Workspaces:
        amy = "colcon_ws"
        rory = "colcon_ws"
        jack = "colcon_ws"
        donna = "colcon_ws"
        melody = "colcon_ws"
        rose = "colcon_ws"

    class RobotComputers:
        amy = ["nuc1"]
        rory = ["nuc2"]
        jack = ["nuc3"]
        donna = ["nuc4"]
        melody = ["nuc5"]
        rose = ["nuc6"]

    class IPs:
        __prefix__ = "172.20.1."
        nuc1 = __prefix__ + "11"
        nuc2 = __prefix__ + "12"
        nuc3 = __prefix__ + "13"
        nuc4 = __prefix__ + "14"
        nuc5 = __prefix__ + "15"
        nuc6 = __prefix__ + "16"

    def __init__(self, ip, ssh_target, hostname=None, robot_name=None):
        """
        :type ip: str
        :type ssh_target: str
        """
        self.ip = ip  # type: str
        self.ssh_target = ssh_target  # type: str

        # figure out hostname
        if hostname:
            self.hostname = hostname
        else:
            for name, iip in self.IPs.__dict__.items():
                if isinstance(ip, str) and iip == ip:
                    self.hostname = name

        # figure out robot_name
        if robot_name:
            self.robot_name = robot_name
        else:
            for name, computers in self.RobotComputers.__dict__.items():
                if isinstance(computers, list) and self.hostname in computers:
                    self.robot_name = name

        self.workspace = getattr(self.Workspaces, self.robot_name)  # type: str

        self.sync_includes_file = os.path.join(BITBOTS_MAIN, f"sync_includes_wolfgang_{self.hostname[:-1]}.yaml")


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Compile and configure software for the Wolfgang humanoid robot " "platform"
    )
    parser.add_argument(
        "target",
        type=str,
        help="The target robot or computer you want to compile for. Multiple "
        "targets can be specified seperated by ,",
    )

    mode = parser.add_mutually_exclusive_group(required=False)
    mode.add_argument("-s", "--sync-only", action="store_true", help="Only sync file from you to the target")
    mode.add_argument("-c", "--compile-only", action="store_true", help="Only build on the target")
    mode.add_argument("-k", "--configure", action="store_true", help="Configure the target as well as everything else")
    mode.add_argument("-K", "--configure-only", action="store_true", help="Only configure the target")

    parser.add_argument("-p", "--package", default="", help="Sync/Compile only the given ROS package")
    parser.add_argument("-y", "--yes-to-all", action="store_true", help="Answer yes to all questions")
    parser.add_argument(
        "--clean-build",
        action="store_true",
        help="Clean workspace before building. If --package is given, clean only that package",
    )
    parser.add_argument("--clean-src", action="store_true", help="Clean source directory before syncing")
    parser.add_argument(
        "--no-rosdeps",
        action="store_false",
        default=True,
        dest="install_rosdeps",
        help="Don't install rosdeps on the target." "Might be useful when no internet connection is available.",
    )
    parser.add_argument("--print-bit-bot", action="store_true", default=False, help="Print our logo at script start")
    parser.add_argument("-v", "--verbose", action="count", default=0, help="More output")
    parser.add_argument("-q", "--quiet", action="count", default=0, help="Less output")
    return parser.parse_args()


def parse_targets(targets):
    """
    Parse target argument into usable Targets

    :type targets: str
    :rtype: list
    """
    res = []

    for target in targets.split(","):
        # this means, a whole robot is meant
        if hasattr(Target.RobotComputers, target):
            for computer in getattr(Target.RobotComputers, target):
                res.append(Target(getattr(Target.IPs, computer), getattr(Target.IPs, computer)))
                print_info(
                    f"Using robot={res[-1].robot_name} hostname={res[-1].hostname} ip={res[-1].ip} for target {target}"
                )

        # this mean, a hostname was specified
        elif hasattr(Target.IPs, target):
            res.append(Target(getattr(Target.IPs, target), target))
            print_info(
                f"Using robot={res[-1].robot_name} hostname={res[-1].hostname} ip={res[-1].ip} for target {target}"
            )

        # this means a known IP address was specified
        elif target in Target.IPs.__dict__.values():
            res.append(Target(target, target))
            print_info(
                f"Using robot={res[-1].robot_name} hostname={res[-1].hostname} ip={res[-1].ip} for target {target}"
            )

        # this means an arbitrary target (likely an IP) was specified
        else:
            try:
                ipaddress.ip_address(target)
            except ValueError:
                print_err(f"{target} is neither a known target nor a valid ip address")
                sys.exit(1)

            cmd = ["ssh", f"bitbots@{target}", "cat /etc/hostname"]
            host_inspect_result = subprocess.run(cmd, stdout=subprocess.PIPE, encoding="utf-8")

            if host_inspect_result.returncode != 0:
                print_err(f"Unable to connect to {target}")
                sys.exit(1)

            hostname = host_inspect_result.stdout.strip()
            robot_name = None
            for robot, computers in Target.RobotComputers.__dict__.items():
                if isinstance(computers, list) and hostname in computers:
                    robot_name = robot
                    break

            if robot_name is None:
                print_err(f"{target} does not seem to be part of a known robot")
                sys.exit(1)

            res.append(Target(target, target, hostname=hostname, robot_name=robot_name))
            print_info(f"Using robot={robot_name}, hostname={hostname} ip={target} for target {target}")

    return res


def get_includes_from_file(file_path, package=""):
    """
    Retrieve a list of file to sync from and includes-file

    :param file_path: Path of the includes-file
    :type file_path: str
    :param package: Limit to file from this package
    :type package: str
    :returns: (a list of included files, a list of excluded files)
    :rtype: tuple[list, list]
    """
    includes = list()
    with open(file_path) as file:
        data = yaml.safe_load(file)
        for entry in data["exclude"]:
            # --include is right here. No worries.
            includes.append(f"--include=- {entry}")
        for entry in data["include"]:
            if isinstance(entry, dict):
                for folder, subfolders in entry.items():
                    if package == "":
                        includes.append(f"--include=+ {folder}")
                        for subfolder in subfolders:
                            includes.append(f"--include=+ {folder}/{subfolder}")
                            includes.append(f"--include=+ {folder}/{subfolder}/**")
                    elif package in subfolders:
                        includes.append(f"--include=+ {folder}")
                        includes.append(f"--include=+ {folder}/{package}")
                        includes.append(f"--include=+ {folder}/{package}/**")
            elif isinstance(entry, str):
                if package == "" or package == entry:
                    includes.append(f"--include=+ {entry}")
                    includes.append(f"--include=+ {entry}/**")
    includes.append("--include=- *")
    return includes


def _execute_on_target(target, command, catch_output=False):
    """
    Execute a command on the given target over ssh

    :type target: Target
    :type command: str
    :type catch_output: bool
    :rtype: subprocess.CompletedProcess
    """
    real_cmd = ["ssh", "-t", f"bitbots@{target.ssh_target}", command]
    print_debug("Calling {}".format(" ".join(real_cmd)))

    if not catch_output:
        return subprocess.run(real_cmd)
    else:
        return subprocess.run(real_cmd, text=True, capture_output=True)


def _should_run_quietly():
    return LOGLEVEL.current < LOGLEVEL.INFO


def sync(target, package="", pre_clean=False):
    """
    :type target: Target
    :type package: str
    :type pre_clean: bool
    """
    if pre_clean:
        print_info(f"Pre-cleaning on {target.hostname}")
        clean_result = _execute_on_target(target, f"rm -rf {target.workspace}/src/*")
        if clean_result.returncode != 0:
            print_warn(f"Cleaning of source directory on {target.hostname} failed. Continuing anyways")

    print_info(f"Synchronizing {target.hostname}")
    cmd = [
        "rsync",
        "--checksum",
        "--archive",
        "--delete",
    ]

    if not _should_run_quietly():
        cmd.append("--verbose")

    cmd.extend(get_includes_from_file(target.sync_includes_file, package))
    cmd.extend([BITBOTS_MAIN + "/", f"bitbots@{target.ssh_target}:{target.workspace}/src/"])

    print_debug("Calling {}".format(" ".join(cmd)))
    sync_result = subprocess.run(cmd)
    if sync_result.returncode != 0:
        print_err(f"Synchronizing {target.hostname} failed with error code {sync_result.returncode}")
        sys.exit(sync_result.returncode)

    print_success(f"Synchronization of {target.hostname} successful")


def build(target, package="", pre_clean=False):
    """
    :type target: Target
    :type package: str
    :type pre_clean: bool
    """
    print_info(f"Building on {target.hostname}")

    if package and pre_clean:
        print_err("Cleaning a specific package is not supported! Not cleaning.")
    elif pre_clean:
        cmd_clean = "rm -rf build install log;"
    else:
        cmd_clean = ""

    cmd = (
        "sync;"
        "cd {workspace};"
        "source /opt/ros/iron/setup.zsh;"
        "source install/setup.zsh;"
        "{cmd_clean}"
        "ISOLATED_CPUS=\"$(grep -oP 'isolcpus=\\K([\\d-]+)' /proc/cmdline)\";"
        "chrt -r 1 taskset -c ${{ISOLATED_CPUS:-0-15}} colcon build --symlink-install {package} --continue-on-error {quiet_option} || exit 1;"
        "sync;"
    ).format(
        **{
            "workspace": target.workspace,
            "cmd_clean": cmd_clean,
            "quiet_option": "> /dev/null" if LOGLEVEL.current < LOGLEVEL.INFO else "",
            "package": "--packages-up-to " + package if package else "",
        }
    )

    build_result = _execute_on_target(target, cmd)
    if build_result.returncode != 0:
        print_err(f"Build on {target.hostname} failed")
        sys.exit(build_result.returncode)

    print_success(f"Build on {target.hostname} succeeded")


def install_rosdeps(target):
    """
    Install dependencies on a target with rosdep

    :type target: Target
    """
    if internet_available(target):
        print_info(f"Installing rosdeps on {target.hostname}")
        target_src_path = os.path.join(target.workspace, "src")
        extra_flags = "-q" if _should_run_quietly() else ""

        cmd = f"rosdep install -y {extra_flags} --rosdistro iron --ignore-src --from-paths {target_src_path}"

        rosdep_result = _execute_on_target(target, cmd)
        if rosdep_result.returncode == 0:
            print_success(f"Rosdeps on {target.hostname} installed successfully")
        else:
            print_warn(f"Rosdep install on {target.hostname} had non-zero exit code. Check its output for more info")
    else:
        print_info(f"Skipping rosdep install on {target.hostname} as we do not have internet")


def configure_game_settings(target):
    print_info("Configuring game settings")
    _execute_on_target(
        target,
        "python3 ~/colcon_ws/src/bitbots_misc/bitbots_parameter_blackboard/bitbots_parameter_blackboard/game_settings.py",
    )
    print_success(f"Game settings on {target.hostname} configured")


def configure_wifi(target):
    """
    Configure default wifi network on given target.

    :type target: Target
    """
    print_info("Configuring Wifi")

    _execute_on_target(target, "nmcli connection show").check_returncode()
    connection_id = input("UUID or name of connection which should be enabled [leave unchanged]: ")

    if connection_id != "":
        # disable all other connections
        connection_ids = (
            str(
                _execute_on_target(
                    target, "nmcli --fields UUID,TYPE connection show | grep wifi | awk '{print $1}'", catch_output=True
                ).stdout
            )
            .strip()
            .split("\n")
        )
        for i in connection_ids:
            _execute_on_target(
                target, f"sudo nmcli connection modify {i} connection.autoconnect FALSE"
            ).check_returncode()
            _execute_on_target(
                target, f"sudo nmcli connection modify {i} connection.autoconnect-priority 0"
            ).check_returncode()

        _execute_on_target(target, f"sudo nmcli connection up {connection_id}").check_returncode()
        _execute_on_target(
            target, f"sudo nmcli connection modify {connection_id} connection.autoconnect TRUE"
        ).check_returncode()
        _execute_on_target(
            target, f"sudo nmcli connection modify {connection_id} connection.autoconnect-priority 100"
        ).check_returncode()


def internet_available(target):
    """
    Check if target has an internet connection by pinging apt repos.

    :type target: Target
    :rtype: bool
    """
    print_info(f"Checking internet connection on {target.hostname}")

    apt_mirror = "de.archive.ubuntu.com"
    redirect_output = "> /dev/null" if _should_run_quietly() else ""
    return (
        _execute_on_target(target, f"timeout --foreground 0.5 curl -sSLI {apt_mirror} {redirect_output}").returncode
        == 0
    )


def main():
    args = parse_arguments()

    LOGLEVEL.current = LOGLEVEL.current + args.verbose - args.quiet

    if args.print_bit_bot:
        print_bit_bot()

    targets = parse_targets(args.target)

    for target in targets:
        # sync
        if args.compile_only:
            print_info(f"Not syncing to {target.hostname} due to compile-only mode")
        elif args.configure_only:
            print_info(f"Not syncing to {target.hostname} due to configure-only mode")
        else:
            sync(target, args.package, pre_clean=args.clean_src)

        # configure
        if args.sync_only:
            print_info(f"Not configuring {target.hostname} due to sync-only mode")
        elif args.compile_only:
            print_info(f"Not configuring {target.hostname} due to compile-only mode")
        elif not (args.configure or args.configure_only):
            print_info(f"Not configuring {target.hostname} due to missing --configure")
        else:
            print_info(f"Running game-settings script for {target.hostname}")
            configure_game_settings(target)
            configure_wifi(target)

        # build
        if args.sync_only:
            print_info(f"Not compiling on {target.hostname} due to sync-only mode")
        elif args.configure_only:
            print_info(f"Not compiling on {target.hostname} due to configure-only mode")
        else:
            if args.install_rosdeps:
                install_rosdeps(target)
            build(target, args.package, pre_clean=args.clean_build)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print_err("Interrupted by user")
        sys.exit(1)
