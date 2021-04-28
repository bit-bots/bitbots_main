#!/usr/bin/env python3
import argparse
import sys
import os
import yaml
from pathlib import Path


if __name__ == "__main__":
    parser = argparse.ArgumentParser(sys.argv[0],
                                     description="Generate .rdmanifest files for a package. These files are "
                                                 "necessary to describe how rosdep can install a package via "
                                                 "the 'source' package manager")
    parser.add_argument("--github-org", default="bit-bots",
                        help="The Github organization which owns the repository containing the target package"
                             " (default: bit-bots)")
    parser.add_argument("--github-repo", required=True,
                        help="The Github repository name containing the target package")
    parser.add_argument("--default-branch", default="master",
                        help="The default branch of the repository containing the target package"
                             " (default: master)")
    parser.add_argument("--package-name", required=True,
                        help="The name of the package for which a .rdmanifest file should be generated")
    parser.add_argument("--target-dir", default=Path(os.getcwd()), type=Path,
                        help="The directory into which a .rdmanifest file should be generated"
                             " (default: current directory)")
    parser.add_argument("--relative-package-dir", default=None,
                        help="Relative directory inside the repository which contains the package."
                             "If not specified the package is not located in a subdirectory but rather the "
                             "repository root")
    parser.add_argument("--dependency", "-d", action="append", dest="dependencies",
                        help="Rosdep key which the target package depends on. "
                             "Can be specified multiple times")
    parser.add_argument("--force", "-f", action="store_true", default=False,
                        help="Force overwriting an existing .rdmanifest file")
    args = parser.parse_args()

    target_file = args.target_dir.absolute() / ".rdmanifest"  # type: Path

    data = {
        "uri": f"https://github.com/{args.github_org}/{args.github_repo}/archive/refs/heads/{args.default_branch}.tar.gz",
        "depends": args.dependencies,
        "exec-path": f"{args.github_repo}-{args.default_branch}/{args.relative_package_dir}"
        if args.relative_package_dir
        else f"{args.github_repo}-{args.default_branch}",
        "check-presence-script": f"#!/bin/bash\ntest -d $BITBOTS_CATKIN_WORKSPACE/src/{args.package_name}",
        "install-script": f"#!/bin/bash\ncp -r . $BITBOTS_CATKIN_WORKSPACE/src/{args.package_name}"
    }

    if target_file.is_file() and not args.force:
        print(
            f"File {target_file.relative_to(os.getcwd())} already exists. Will not overwrite unless --force is given",
            file=sys.stderr)
        sys.exit(1)

    print(f"Writing {target_file.relative_to(os.getcwd())}")
    with open(args.target_dir / ".rdmanifest", "w") as f:
        f.write("---\n")
        f.write("# See http://doku.bit-bots.de/meta/manual/software/ci.html#make-package-resolvable-in-ci\n")
        yaml.safe_dump(data, f)
