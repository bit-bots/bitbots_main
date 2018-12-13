#!/usr/bin/env python3

"""
This module bundles some usefull functions which are needed if you want to generate BitBots documentation.
It is intended to be used by other scripts as a sort of library

:author Finn-Thorben Sell <7sell@informatik.uni-hamburg.de>
"""

import os
import subprocess
import shutil
#import rospkg
from rospkg import RosPack


PKG_DOC_DIR = "doc"


def discover_ros_pkgs(root):
    return RosPack([root])


def clean_pkg(rospack, pkg_name):
    """Cleanup a modules documentation and rosdoc config"""
    pkg_path = rospack.get_path(pkg_name)

    if os.path.exists(os.path.join(pkg_path, "rosdoc.yaml")):
        os.remove(os.path.join(pkg_path, "rosdoc.yaml"))

    if os.path.exists(os.path.join(pkg_path, PKG_DOC_DIR)):
        shutil.rmtree(os.path.join(pkg_path, PKG_DOC_DIR))

def copy_pkg_doc(rospack, pkg_name, dst_dir):
    """Copy a packages documentation (without html) to a specified destination"""
    src_dir = os.path.join(rospack.get_path(pkg_name), PKG_DOC_DIR)

    if not os.path.exists(src_dir):
        raise FileNotFoundError("Documentation at %s does not exist. It needs to be generated before you can copy it" % src_dir)

    if os.path.exists(dst_dir):
        shutil.rmtree(dst_dir)

    shutil.copytree(src_dir, dst_dir, ignore=shutil.ignore_patterns("html"))


def is_doc_enabled(rospack, pkg_name):
    """Is documentation enabled thorugh package.xml for this package?"""
    rosdoc_export = rospack.get_manifest(pkg_name).get_export("rosdoc", "config")
    return len(rosdoc_export) > 0


def __prepare_pkg(pkg_path, rosdoc_conf, sphinx_conf, pkg_index_rst):
    """Prepare a module by copying rosdoc and sphinx config"""
    shutil.copy(rosdoc_conf, os.path.join(pkg_path, "rosdoc.yaml"))

    if not os.path.exists(os.path.join(pkg_path, PKG_DOC_DIR)):
        os.makedirs(os.path.join(pkg_path, PKG_DOC_DIR))
    shutil.copy(sphinx_conf, os.path.join(pkg_path, PKG_DOC_DIR, "conf.py"))
    shutil.copy(pkg_index_rst, os.path.join(pkg_path, PKG_DOC_DIR, "index.rst"))

def __index_pkg_api(pkg_path, pkg_src_dir):
    cmd = "sphinx-apidoc -fe -o %s %s" % (os.path.join(pkg_path, PKG_DOC_DIR), os.path.join(pkg_path, pkg_src_dir))
    print("SPHINX-APIDOC:")
    print(cmd)
    subprocess.call(cmd.split())

def __build_actual_pkg_doc(pkg_path):
    cmd = "python2 /opt/ros/kinetic/bin/rosdoc_lite -o %s %s" % (os.path.join(pkg_path, PKG_DOC_DIR), pkg_path)
    print("ROSDOC:")
    print(cmd)
    subprocess.call(cmd.split())


def build_single_package_doc(rospack, pkg_name, rosdoc_conf, sphinx_conf, pkg_index_rst, pkg_src_dir="src"):
    """
    Build the documentation for a packages API

    :arg rosdoc_conf The location of rosdoc.yaml template which needs to be copied to the package
    :arg sphinx_conf The location of conf.py which needs to be copied to the packages doc directory in order for sphix to work
    :arg pkg_index_rst The location of an index.rst file which serves as the root of the new documentation
    :arg pkg_src_dir Relative path inside the package which holds the actual source-code. Used for API auto-discovery
    """

    pkg_path = rospack.get_path(pkg_name)
    print("Building documentation for " + pkg_name + " from " + pkg_path)

    clean_pkg(rospack, pkg_name)
    __prepare_pkg(pkg_path, rosdoc_conf, sphinx_conf, pkg_index_rst)
    __index_pkg_api(pkg_path, pkg_src_dir)
    __build_actual_pkg_doc(pkg_path)

def build_meta_doc(doc_dir):
    """
    Build the final bitbots_meta documentation
    Usually called after :build_single_package_doc and :copy_pkg_doc were called for each package

    :param doc_dir: Where the meta documentation is located
    :return:
    """
    cmd = "sphinx-build -b html %s %s" % (doc_dir, os.path.join(doc_dir, "html"))
    print("FINAL SPHINX:")
    print(cmd)
    subprocess.call(cmd.split())
