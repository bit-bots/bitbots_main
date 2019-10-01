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


try:
    from bitbots_bringup import game_settings
except ImportError:
    bringup_dir = os.path.join(BITBOTS_META, "bitbots_misc", "bitbots_bringup", "src")
    print_info("Manually adding {} to PATH to import bitbots_bringup. If this fails please source ros".format(bringup_dir))
    sys.path.append(bringup_dir)
    from bitbots_bringup import game_settings


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
        amy = "wolfgang_ws"
        rory = "wolfgang_ws"
        jack = "wolfgang_ws"
        donna = "wolfgang_ws"
        davros = "davros_ws"

    class RobotComputers:
        amy = [c + "1" for c in ["nuc", "odroid", "jetson"]]
        rory = [c + "2" for c in ["nuc", "odroid", "jetson"]]
        jack = [c + "3" for c in ["nuc", "odroid", "jetson"]]
        donna = [c + "4" for c in ["nuc", "odroid", "jetson"]]
        davros = ["davros"]

    class IPs:
        __prefix__ = "192.168.1."
        nuc1 = __prefix__ + "11"
        odroid1 = __prefix__ + "21"
        jetson1 = __prefix__ + "31"
        nuc2 = __prefix__ + "12"
        odroid2 = __prefix__ + "22"
        jetson2 = __prefix__ + "32"
        nuc3 = __prefix__ + "13"
        odroid3 = __prefix__ + "23"
        jetson3 = __prefix__ + "33"
        nuc4 = __prefix__ + "14"
        odroid4 = __prefix__ + "24"
        jetson4 = __prefix__ + "34"
        davros = __prefix__ + "25"

    def __init__(self, ip, ssh_target):
        """
        :type ip: str
        :type ssh_target: str
        """
        self.ip = ip  # type: str
        self.ssh_target = ssh_target  # type: str

        # figure out hostname
        for name, iip in self.IPs.__dict__.items():
            if isinstance(ip, str) and iip == ip:
                self.hostname = name

        # figure out robot_name
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
    parser.add_argument("target", type=str, help="The target robot or computer you want to compile for. Multiple "
                                                 "targets can be specified seperated by ,")

    mode = parser.add_mutually_exclusive_group(required=False)
    mode.add_argument("-s", "--sync-only", action="store_true", help="Only sync file from you to the target")
    mode.add_argument("-c", "--compile-only", action="store_true", help="Only build on the target")
    mode.add_argument("-k", "--configure", action="store_true", help="Configure the target as well as everything else")

    parser.add_argument("-p", "--package", default='', help="Sync/Compile only the given ROS package")
    parser.add_argument("-y", "--yes-to-all", action="store_true", help="Answer yes to all questions")
    parser.add_argument("--clean-build", action="store_true",
                        help="Clean workspace before building. If --package is given, clean only that package")
    parser.add_argument("--clean-src", action="store_true", help="Clean source directory before syncing")
    parser.add_argument("--no-rosdeps", action="store_false", default=True, dest="check_rosdeps",
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
                print_info("Using robot={} hostname={} ip={} for target {}".format(
                    res[-1].robot_name, res[-1].hostname, res[-1].ip, target))

        # this mean, a hostname was specified
        elif hasattr(Target.IPs, target):
            res.append(Target(getattr(Target.IPs, target), target))
            print_info("Using robot={} hostname={} ip={} for target {}".format(
                res[-1].robot_name, res[-1].hostname, res[-1].ip, target))

        # this means a known IP address was specified
        elif target in Target.IPs.__dict__.values():
            res.append(Target(target, target))
            print_info("Using robot={} hostname={} ip={} for target {}".format(
                res[-1].robot_name, res[-1].hostname, res[-1].ip, target))

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

            print_info("Reparsing {}'s hostname ({})".format(target, host_inspect_result.stdout.replace("\n", "")))
            res.extend(parse_targets(host_inspect_result.stdout.replace("\n", "")))

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


def sync(target, package='', pre_clean=False):
    """
    :type target: Target
    :type package: str
    :type pre_clean: bool
    """
    if pre_clean:
        print_info("Pre-cleaning on {}".format(target.hostname))
        cmd = [
            "ssh",
            "bitbots@{}".format(target.hostname),
            "rm -rf {}/src/*".format(target.workspace)
        ]
        print_debug("Calling {}".format(" ".join(cmd)))
        clean_result = subprocess.run(cmd)
        if clean_result.returncode != 0:
            print_warn("Cleaning of source directory on {} failed. Continuing anyways".format(target.hostname))

    print_info("Synchronizing {}".format(target.hostname))
    cmd = [
        "rsync",
        "--checksum",
        "--archive",
        "-v" if LOGLEVEL.current >= LOGLEVEL.DEBUG else "",
        "--delete",
    ]
    cmd.extend(get_includes_from_file(target.sync_includes_file, package))
    cmd.extend([
        BITBOTS_META + "/",
        "bitbots@{}:{}/src/".format(target.ssh_target, target.workspace)
    ])

    print_debug("Calling {}".format(" ".join(cmd)))
    sync_result = subprocess.run(cmd)
    if sync_result.returncode != 0:
        print_err("Synchronizing {} failed with error code {}".format(target.hostname, sync_result.returncode))
        sys.exit(sync_result.returncode)

    print_success("Synchronization of {} successful".format(target.hostname))


def sync_gamesettings(target):
    """
    :type target: Target
    """
    print_info("Synchronizing gamesettings to {}".format(target.hostname))
    cmd = [
        "rsync",
        "--checksum",
        "--archive",
        "-v" if LOGLEVEL.current >= LOGLEVEL.DEBUG else "",
        os.path.join(BITBOTS_META, "bitbots_misc", "bitbots_bringup", "config", "game_settings.yaml"),
        "bitbots@{}:{}/src/bitbots_misc/bitbots_bringup/config/game_settings.yaml"
            .format(target.ssh_target, target.workspace)
    ]

    print_debug("Calling {}".format(" ".join(cmd)))
    sync_result = subprocess.run(cmd)
    if sync_result.returncode != 0:
        print_err("Synchronizing game settings with {} failed with error code {}"
                  .format(target.hostname, sync_result.returncode))
        sys.exit(sync_result.returncode)

    print_success("Synchronizing game settings with {} succeeded".format(target.hostname))


def build(target, package='', pre_clean=False):
    """
    :type target: Target
    :type package: str
    :type pre_clean: bool
    """
    print_info("Building on {}".format(target.hostname))

    cmd = [
        "ssh",
        "bitbots@{}".format(target.ssh_target),
        ("sync;"
         "cd {workspace};"
         "source devel/setup.zsh;"
         "{cmd_clean}"
         "catkin build --force-color {package} {quiet_option} --continue-on-failure --summary || exit 1;"
         "./src/scripts/repair.sh {quiet_option};"
         "sync;"
         ).format(**{
            "workspace": target.workspace,
            "cmd_clean": "cakin clean -y {};".format(package) if pre_clean else "",
            "quiet_option": "> /dev/null" if LOGLEVEL.current < LOGLEVEL.INFO else "",
            "package": package
        })
    ]

    print_debug("Calling {}".format(" ".join(cmd)))
    build_result = subprocess.run(cmd)
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

    cmd = [
        "ssh",
        "bitbots@{}".format(target.ssh_target),
        "rosdep check {} --ignore-src --from-paths {}".format(
            "" if LOGLEVEL.current >= LOGLEVEL.INFO else "-q",
            os.path.join(target.workspace, "src")
        ),
    ]

    print_debug("Calling {}".format(" ".join(cmd)))

    rosdep_result = subprocess.run(cmd)
    if rosdep_result.returncode != 0:
        print_warn("rosdep check on {} had non-zero exit code. Check its output for more info"
                   .format(target.hostname))

    print_success("Rosdeps on {} installed successfully".format(target.hostname))


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
        else:
            sync(target, args.package, pre_clean=args.clean_src)

        # build
        if args.sync_only:
            print_info("Not compiling on {} due to sync-only mode".format(target.hostname))
        else:
            if args.check_rosdeps:
                check_rosdeps(target)
            build(target, args.package, pre_clean=args.clean_build)

        # configure
        if args.sync_only:
            print_info("Not configuring {} due to sync-only mode".format(target.hostname))
        elif args.compile_only:
            print_info("Not configuring {} due to compile-only mode".format(target.hostname))
        elif not args.configure:
            print_info("Not configuring {} due to mossing --configure".format(target.hostname))
        else:
            print_info("Running game-settings script for {}".format(target.hostname))
            game_settings.main()
            sync_gamesettings(target)


if __name__ == "__main__":
    main()
