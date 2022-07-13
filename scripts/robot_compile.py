#!/usr/bin/env python3

import argparse
import sys
import os
import yaml
import subprocess
import ipaddress


class LOGLEVEL:
    current = 2
    DEBUG = 3
    INFO = 2
    WARN = 1
    ERR_SUCCESS = 0


BITBOTS_META = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def print_err(msg):
    if LOGLEVEL.current >= LOGLEVEL.ERR_SUCCESS:
        print("\033[91m\033[1m##" + "".join(["#"] * len(str(msg))) + "##\033[0m")
        print("\033[91m\033[1m# " + "".join([" "] * len(str(msg))) + " #\033[0m")
        print('\033[91m\033[1m# ' + str(msg) + ' #\033[0m')
        print("\033[91m\033[1m# " + "".join([" "] * len(str(msg))) + " #\033[0m")
        print("\033[91m\033[1m##" + "".join(["#"] * len(str(msg))) + "##\033[0m")


def print_warn(msg):
    if LOGLEVEL.current >= LOGLEVEL.WARN:
        print("\033[93m\033[1m# " + "".join([" "] * len(str(msg))) + " #\033[0m")
        print('\033[93m\033[1m# ' + str(msg) + ' #\033[0m')
        print("\033[93m\033[1m# " + "".join([" "] * len(str(msg))) + " #\033[0m")


def print_success(msg):
    if LOGLEVEL.current >= LOGLEVEL.ERR_SUCCESS:
        print("\033[92m\033[1m# " + "".join([" "] * len(str(msg))) + " #\033[0m")
        print('\033[92m\033[1m# ' + str(msg) + ' #\033[0m')
        print("\033[92m\033[1m# " + "".join([" "] * len(str(msg))) + " #\033[0m")


def print_info(msg):
    if LOGLEVEL.current >= LOGLEVEL.INFO:
        print('\033[96m' + str(msg) + '\033[0m')


def print_debug(msg):
    if LOGLEVEL.current >= LOGLEVEL.DEBUG:
        print(msg)


def print_bit_bot():
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


class Target:

    class Workspaces:
        amy = "colcon_ws"
        rory = "colcon_ws"
        jack = "colcon_ws"
        donna = "colcon_ws"
        melody = "colcon_ws"
        rose = "colcon_ws"
        davros = "davros_ws"

    class RobotComputers:
        amy = ["nuc1"]
        rory = ["nuc2"]
        jack = ["nuc3"]
        donna = ["nuc4"]
        melody = ["nuc5"]
        rose = ["nuc6"]
        davros = ["davros"]

    class IPs:
        __prefix__ = "192.168.1."
        nuc1 = __prefix__ + "11"
        nuc2 = __prefix__ + "12"
        nuc3 = __prefix__ + "13"
        nuc4 = __prefix__ + "14"
        nuc5 = __prefix__ + "15"
        nuc6 = __prefix__ + "16"
        davros = __prefix__ + "25"

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

        # figure out sync_includes
        if self.hostname == "davros":
            self.sync_includes_file = os.path.join(BITBOTS_META, "sync_includes_davros.yaml")
        else:
            self.sync_includes_file = os.path.join(BITBOTS_META,
                                                   "sync_includes_wolfgang_{}.yaml".format(self.hostname[:-1]))


def parse_arguments():
    parser = argparse.ArgumentParser(description="Compile and configure software for the Wolfgang humanoid robot "
                                     "platform")
    parser.add_argument("target",
                        type=str,
                        help="The target robot or computer you want to compile for. Multiple "
                        "targets can be specified seperated by ,")

    mode = parser.add_mutually_exclusive_group(required=False)
    mode.add_argument("-s", "--sync-only", action="store_true", help="Only sync file from you to the target")
    mode.add_argument("-c", "--compile-only", action="store_true", help="Only build on the target")
    mode.add_argument("-k", "--configure", action="store_true", help="Configure the target as well as everything else")
    mode.add_argument("-K", "--configure-only", action="store_true", help="Only configure the target")

    parser.add_argument("-p", "--package", default='', help="Sync/Compile only the given ROS package")
    parser.add_argument("-y", "--yes-to-all", action="store_true", help="Answer yes to all questions")
    parser.add_argument("--clean-build",
                        action="store_true",
                        help="Clean workspace before building. If --package is given, clean only that package")
    parser.add_argument("--clean-src", action="store_true", help="Clean source directory before syncing")
    parser.add_argument("--no-rosdeps",
                        action="store_false",
                        default=False,
                        dest="check_rosdeps",
                        help="Don't check installed rosdeps on the target."
                        "Might be useful when no internet connection is available.")
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
                print_info("Using robot={} hostname={} ip={} for target {}".format(res[-1].robot_name, res[-1].hostname,
                                                                                   res[-1].ip, target))

        # this mean, a hostname was specified
        elif hasattr(Target.IPs, target):
            res.append(Target(getattr(Target.IPs, target), target))
            print_info("Using robot={} hostname={} ip={} for target {}".format(res[-1].robot_name, res[-1].hostname,
                                                                               res[-1].ip, target))

        # this means a known IP address was specified
        elif target in Target.IPs.__dict__.values():
            res.append(Target(target, target))
            print_info("Using robot={} hostname={} ip={} for target {}".format(res[-1].robot_name, res[-1].hostname,
                                                                               res[-1].ip, target))

        # this means an arbitrary target (likely an IP) was specified
        else:
            try:
                ipaddress.ip_address(target)
            except ValueError:
                print_err("{} is neither a known target nor a valid ip address".format(target))
                sys.exit(1)

            cmd = ["ssh", "bitbots@{}".format(target), "cat /etc/hostname"]
            host_inspect_result = subprocess.run(cmd, stdout=subprocess.PIPE, encoding="utf-8")

            if host_inspect_result.returncode != 0:
                print_err("Unable to connect to {}".format(target))
                sys.exit(1)

            hostname = host_inspect_result.stdout.strip()
            robot_name = None
            for robot, computers in Target.RobotComputers.__dict__.items():
                if isinstance(computers, list) and hostname in computers:
                    robot_name = robot
                    break

            if robot_name is None:
                print_err("{} does not seem to be part of a known robot".format(target))
                sys.exit(1)

            res.append(Target(target, target, hostname=hostname, robot_name=robot_name))
            print_info("Using robot={}, hostname={} ip={} for target {}".format(robot_name, hostname, target, target))

    return res


def get_includes_from_file(file_path, package=''):
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
        for entry in data['exclude']:
            # --include is right here. No worries.
            includes.append('--include=- {}'.format(entry))
        for entry in data['include']:
            if isinstance(entry, dict):
                for folder, subfolders in entry.items():
                    if package == '':
                        includes.append('--include=+ {}'.format(folder))
                        for subfolder in subfolders:
                            includes.append('--include=+ {}/{}'.format(folder, subfolder))
                            includes.append('--include=+ {}/{}/**'.format(folder, subfolder))
                    elif package in subfolders:
                        includes.append('--include=+ {}'.format(folder))
                        includes.append('--include=+ {}/{}'.format(folder, package))
                        includes.append('--include=+ {}/{}/**'.format(folder, package))
            elif isinstance(entry, str):
                if package == '' or package == entry:
                    includes.append('--include=+ {}'.format(entry))
                    includes.append('--include=+ {}/**'.format(entry))
    includes.append('--include=- *')
    return includes


def _execute_on_target(target, command, catch_output=False):
    """
    Execute a command on the given target over ssh

    :type target: Target
    :type command: str
    :type catch_output: bool
    :rtype: subprocess.CompletedProcess
    """
    real_cmd = ["ssh", "bitbots@{}".format(target.ssh_target), command]
    print_debug("Calling {}".format(" ".join(real_cmd)))

    if not catch_output:
        return subprocess.run(real_cmd)
    else:
        return subprocess.run(real_cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def sync(target, package='', pre_clean=False):
    """
    :type target: Target
    :type package: str
    :type pre_clean: bool
    """
    if pre_clean:
        print_info("Pre-cleaning on {}".format(target.hostname))
        clean_result = _execute_on_target(target, "rm -rf {}/src/*".format(target.workspace))
        if clean_result.returncode != 0:
            print_warn("Cleaning of source directory on {} failed. Continuing anyways".format(target.hostname))

    print_info("Synchronizing {}".format(target.hostname))
    cmd = [
        "rsync",
        "--checksum",
        "--archive",
        "-v" if LOGLEVEL.current >= LOGLEVEL.INFO else "",
        "--delete",
    ]
    cmd.extend(get_includes_from_file(target.sync_includes_file, package))
    cmd.extend([BITBOTS_META + "/", "bitbots@{}:{}/src/".format(target.ssh_target, target.workspace)])

    print_debug("Calling {}".format(" ".join(cmd)))
    sync_result = subprocess.run(cmd)
    if sync_result.returncode != 0:
        print_err("Synchronizing {} failed with error code {}".format(target.hostname, sync_result.returncode))
        sys.exit(sync_result.returncode)

    print_success("Synchronization of {} successful".format(target.hostname))


def build(target, package='', pre_clean=False):
    """
    :type target: Target
    :type package: str
    :type pre_clean: bool
    """
    print_info("Building on {}".format(target.hostname))

    if package and pre_clean:
        print_err("Cleaning a specific package is not supported! Not cleaning.")
    elif pre_clean:
        cmd_clean = 'rm -rf build install;'
    else:
        cmd_clean = ''

    cmd = ("sync;"
           "cd {workspace};"
           "source /opt/ros/rolling/setup.zsh;"
           "source install/setup.zsh;"
           "{cmd_clean}"
           "ISOLATED_CPUS=\"$(grep -oP 'isolcpus=\K([\d,]+)' /proc/cmdline)\";"
           "chrt -r 1 taskset -c $ISOLATED_CPUS colcon build --symlink-install {package} --continue-on-error {quiet_option} || exit 1;"
           "sync;").format(
               **{
                   "workspace": target.workspace,
                   "cmd_clean": cmd_clean,
                   "quiet_option": "> /dev/null" if LOGLEVEL.current < LOGLEVEL.INFO else "",
                   "package": '--packages-up-to ' + package if package else '',
               })

    build_result = _execute_on_target(target, cmd)
    if build_result.returncode != 0:
        print_err("Build on {} failed".format(target.hostname))
        sys.exit(build_result.returncode)

    print_success("Build on {} succeeded".format(target.hostname))


def check_rosdeps(target):
    """
    Check installed dependencies on a target with rosdep

    :type target: Target
    """
    print_info("Checking installed rosdeps on {}".format(target.hostname))

    cmd = "rosdep check {} --ignore-src --from-paths {}".format("" if LOGLEVEL.current >= LOGLEVEL.INFO else "-q",
                                                                os.path.join(target.workspace, "src"))

    rosdep_result = _execute_on_target(target, cmd)
    if rosdep_result.returncode != 0:
        print_warn("rosdep check on {} had non-zero exit code. Check its output for more info".format(target.hostname))

    print_success("Rosdeps on {} installed successfully".format(target.hostname))


def configure_game_settings(target):
    print_info("Configuring game settings")
    _execute_on_target(target, "python3 ~/colcon_ws/src/bitbots_misc/bitbots_utils/bitbots_utils/game_settings.py")
    print_success("Game settings on {} configured".format(target.hostname))


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
        connection_ids = str(_execute_on_target(target, "nmcli --fields UUID,TYPE connection show | grep wifi | awk '{print $1}'", catch_output=True).stdout) \
                             .strip().split("\n")
        for i in connection_ids:
            _execute_on_target(target,
                               'sudo nmcli connection modify {} connection.autoconnect FALSE'.format(i)).check_returncode()
            _execute_on_target(
                target, 'sudo nmcli connection modify {} connection.autoconnect-priority 0'.format(i)).check_returncode()

        _execute_on_target(target, "sudo nmcli connection up {}".format(connection_id)).check_returncode()
        _execute_on_target(
            target, "sudo nmcli connection modify {} connection.autoconnect TRUE".format(connection_id)).check_returncode()
        _execute_on_target(
            target,
            "sudo nmcli connection modify {} connection.autoconnect-priority 100".format(connection_id)).check_returncode()


def main():
    args = parse_arguments()

    LOGLEVEL.current = LOGLEVEL.current + args.verbose - args.quiet

    if args.print_bit_bot:
        print_bit_bot()

    targets = parse_targets(args.target)

    for target in targets:
        # sync
        if args.compile_only:
            print_info("Not syncing to {} due to compile-only mode".format(target.hostname))
        elif args.configure_only:
            print_info("Not syncing to {} due to configure-only mode".format(target.hostname))
        else:
            sync(target, args.package, pre_clean=args.clean_src)

        # configure
        if args.sync_only:
            print_info("Not configuring {} due to sync-only mode".format(target.hostname))
        elif args.compile_only:
            print_info("Not configuring {} due to compile-only mode".format(target.hostname))
        elif not (args.configure or args.configure_only):
            print_info("Not configuring {} due to missing --configure".format(target.hostname))
        else:
            print_info("Running game-settings script for {}".format(target.hostname))
            configure_game_settings(target)
            configure_wifi(target)

        # build
        if args.sync_only:
            print_info("Not compiling on {} due to sync-only mode".format(target.hostname))
        elif args.configure_only:
            print_info("Not compiling on {} due to configure-only mode".format(target.hostname))
        else:
            if args.check_rosdeps:
                check_rosdeps(target)
            build(target, args.package, pre_clean=args.clean_build)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print_err("Interrupted by user")
        sys.exit(1)
