#!/usr/bin/env python3

import argparse
import sys
import os
import yaml
import subprocess


BITBOTS_META = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def print_err(msg):
    print("\033[91m\033[1m##" + "".join(["#"] * len(str(msg))) + "##\033[0m")
    print("\033[91m\033[1m# " + "".join([" "] * len(str(msg))) + " #\033[0m")
    print('\033[91m\033[1m# ' + str(msg) + ' #\033[0m')
    print("\033[91m\033[1m# " + "".join([" "] * len(str(msg))) + " #\033[0m")
    print("\033[91m\033[1m##" + "".join(["#"] * len(str(msg))) + "##\033[0m")


def print_warn(msg):
    print("\033[93m\033[1m# " + "".join([" "] * len(str(msg))) + " #\033[0m")
    print('\033[93m\033[1m# ' + str(msg) + ' #\033[0m')
    print("\033[93m\033[1m# " + "".join([" "] * len(str(msg))) + " #\033[0m")


def print_success(msg):
    print("\033[92m\033[1m# " + "".join([" "] * len(str(msg))) + " #\033[0m")
    print('\033[92m\033[1m# ' + str(msg) + ' #\033[0m')
    print("\033[92m\033[1m# " + "".join([" "] * len(str(msg))) + " #\033[0m")


def print_info(msg):
    print('\033[96m' + str(msg) + '\033[0m')


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
        davros = ["odroid5"]

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
        nuc4 = __prefix__ + "13"
        odroid4 = __prefix__ + "23"
        jetson4 = __prefix__ + "33"
        davros = __prefix__ + "25"

    def __init__(self, ip):
        """
        :type ip: str
        :type hostname: str
        """
        self.ip = ip  # type: str

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
            self.sync_includes_file = os.path.join(BITBOTS_META, "sync_includes_wolfgang_{}.yaml".format(self.hostname[:-1]))


def parse_arguments():
    parser = argparse.ArgumentParser(description="Compile and comfingure software for the Wolfgang humanoid robot "
                                                 "platform")
    parser.add_argument("target", type=str, help="The target robot or computer you want to compile for. Multiple "
                                                 "targets can be specified seperated by ,")

    mode = parser.add_mutually_exclusive_group(required=False)
    mode.add_argument("-s", "--sync-only", action="store_true", help="Only sync file from you to the target")
    mode.add_argument("-c", "--compile-only", action="store_true", help="Only build on the target")
    mode.add_argument("-k", "--configure-only", action="store_true", help="Only configure the target")

    parser.add_argument("-p", "--package", help="Sync/Compile only the given ROS package")
    parser.add_argument("-y", "--yes-to-all", action="store_true", help="Answer yes to all questions")
    parser.add_argument("--clean-build", action="store_true",
                        help="Clean workspace before building. --package is given, clean only that package")
    parser.add_argument("--clean-src", action="store_true", help="Clean source directory before syncing")
    parser.add_argument("--clean-all", action="store_true",
                        help="Clean workspace and source directory before building/syncing")
    parser.add_argument("-q", "--quiet", action="store_true", help="Less output")
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
                res.append(Target(getattr(Target.IPs, computer)))
                print_info("Using robot={} hostname={} ip={} for target {}".format(
                    res[-1].robot_name, res[-1].hostname, res[-1].ip, target))

        # this mean, a hostname was specified
        elif hasattr(Target.IPs, target):
            res.append(Target(getattr(Target.IPs, target)))
            print_info("Using robot={} hostname={} ip={} for target {}".format(
                res[-1].robot_name, res[-1].hostname, res[-1].ip, target))

        # this means an IP address was specified
        elif target in Target.IPs.__dict__.values():
            res.append(Target(target))
            print_info("Using robot={} hostname={} ip={} for target {}".format(
                res[-1].robot_name, res[-1].hostname, res[-1].ip, target))

        else:
            print_err("Could not find a robot, hostname or IP address matching {}".format(target))
            sys.exit(1)

    return res


def get_includes_from_file(file_path, package):
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
                    if package is None:
                        includes.append('--include=+ {}'.format(folder))
                        for subfolder in subfolders:
                            includes.append('--include=+ {}/{}'.format(folder, subfolder))
                            includes.append('--include=+ {}/{}/**'.format(folder, subfolder))
                    elif package in subfolders:
                        includes.append('--include=+ {}'.format(folder))
                        includes.append('--include=+ {}/{}'.format(folder, package))
                        includes.append('--include=+ {}/{}/**'.format(folder, package))
            elif isinstance(entry, str):
                if package is None or entry == package:
                    includes.append('--include=+ {}'.format(entry))
                    includes.append('--include=+ {}/**'.format(entry))
    includes.append('--include=- *')
    return includes


def sync(target, package, verbose=True):
    """
    :type target: Target
    :type package: str
    """
    cmd = [
        "rsync",
        "--checksum",
        "--archive",
        "--delete"
        "-v" if verbose else "",
    ]
    cmd.extend(get_includes_from_file(target.sync_includes_file, package))
    cmd.extend([
        BITBOTS_META,
        "bitbots@{}:{}/src/".format(target.ip, target.workspace)
    ])

    sync_result = subprocess.run(cmd)
    if sync_result.returncode != 0:
        print_err("Synchronizing {} failed with error code {}".format(target.hostname, sync_result.returncode))
        sys.exit(sync_result.returncode)

    print_success("Synchronized {}".format(target.hostname))


def build(target):
    """.:type target: Target"""
    pass


def main():
    args = parse_arguments()
    targets = parse_targets(args.target)

    for target in targets:
        # sync
        if args.compile_only:
            print_info("Not syncing to {} due to compile-only mode".format(target.hostname))
        elif args.configure_only:
            print_info("Not syncing to {} due to configure-only mode".format(target.hostname))
        else:
            sync(target, args.package)

        # build
        if args.sync_only:
            print_info("Not compiling on {} due to sync-only mode".format(target.hostname))
        elif args.configure_only:
            print_info("Not compiling on {} due to configure-only mode".format(target.hostname))
        else:
            build(target)

        # configure
        if args.sync_only:
            print_info("Not configuring {} due to sync-only mode".format(target.hostname))
        elif args.compile_only:
            print_info("Not configuring {} due to compile-only mode".format(target.hostname))
        else:
            build(target)


if __name__ == "__main__":
    main()
