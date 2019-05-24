#!/usr/bin/env python3
import sys
import os
import argparse
import subprocess
from rospkg import RosPack


class Colors:
    BLACK = "\033[0;30m"
    RED = "\033[0;31m"
    GREEN = "\033[0;32m"
    ORANGE = "\033[0;33m"
    BLUE = "\033[0;34m"
    PURPLE = "\033[0;35m"
    CYAN = "\033[0;36m"
    LIGHT_GRAY = "\033[0;37m"
    DARK_GRAY = "\033[1;30m"
    LIGHT_RED = "\033[1;31m"
    LIGHT_GREEN = "\033[1;32m"
    YELLOW = "\033[1;33m"
    LIGHT_BLUE = "\033[1;34m"
    LIGHT_PURPLE = "\033[1;35m"
    LIGHT_CYAN = "\033[1;36m"
    WHITE = "\033[1;37m"
    NO_COLOR = "\033[0m"
    BOLD = "$(tput bold)"
    NORMAL = "$(tput sgr0)"


def log_error(message):
    print("{}[!]{} {}".format(Colors.RED, Colors.NO_COLOR, message.replace("\n", "\n    ")),
          file=sys.stderr)


def log_info(message):
    print("{}[i]{} {}".format(Colors.BLUE, Colors.NO_COLOR, message.replace("\n", "\n    ")))


def parse_args():
    parser = argparse.ArgumentParser(
        description='A utility script to build and merge bitbots documentation'
    )

    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('-m', '--meta',
                      action='store_const', const=True, default=False,
                      help='Build manual documentation from bitbots_meta only'
                      )
    mode.add_argument('-p', '--package',
                      help='Build documentation for single package only')

    parser.add_argument('-v',
                        action='count', dest='verbosity', default=0,
                        help='Be more verbose. Can be given multiple times')

    return parser.parse_args()


def filter_packages_for_bitbots(rospack):
    return [pkg_name
            for pkg_name in rospack.list()
            if "bitbots" in rospack.get_path(pkg_name)
            ]


def build_package_doc(rospack, pkg_name, args):
    if not pkg_name in rospack.list():
        log_error("Package {} is not in $ROS_PACKAGE_PATH".format(pkg_name))
        return

    log_info("Building documentation for package {}".format(pkg_name))
    p = subprocess.run([
        "rosdoc_lite",
        rospack.get_path(pkg_name)
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, stdin=subprocess.DEVNULL,
        cwd=rospack.get_path(pkg_name),
        encoding='ASCII')

    # error handling
    if p.returncode != 0:
        log_error("Error calling rosdoc_list".format(pkg_name))
        print(p.stderr)
        if args.verbosity >= 1:
            print(p.stdout)

    # verbosity handling
    else:
        if args.verbosity >= 1:
            print(p.stderr)
            if args.verbosity >= 2:
                print(p.stdout)

            print()


def build_meta_doc():
    log_info("Building bitbots_meta documentation")
    doc_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "doc")

    p = subprocess.run(["sphinx-build", doc_dir, os.path.join(doc_dir, "_build")],
                       stdout=subprocess.PIPE)

    # error handling
    if p.returncode != 0:
        log_error("Error calling sphinx-build")
        print(p.stderr)
        if args.verbosity >= 1:
            print(p.stdout)

    # verbosity handling
    else:
        if args.verbosity >= 1:
            print(p.stderr)
            if args.verbosity >= 2:
                print(p.stdout)

            print()


if __name__ == '__main__':
    args = parse_args()
    build_all = not args.meta and not args.package

    rospack = RosPack(os.getenv("ROS_PACKAGE_PATH").split(":"))

    if build_all:
        for pkg_name in filter_packages_for_bitbots(rospack):
            build_package_doc(rospack, pkg_name, args)

    if args.package:
        build_package_doc(rospack, args.package, args)

    if build_all or args.meta:
        build_meta_doc()

    log_error("Cannot yet merge package and meta documentation")
