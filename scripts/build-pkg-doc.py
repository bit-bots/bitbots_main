#!/usr/bin/env python3

import os
import sys

import doc_utils


BITBOTS_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

DOC_DIR = os.path.join(BITBOTS_ROOT, "doc")
PKG_TEMPLATE_DIR = os.path.join(DOC_DIR, "pkg-templates")

ROSDOC_CONF = os.path.join(PKG_TEMPLATE_DIR, "rosdoc.yaml")
SPHINX_CONF = os.path.join(PKG_TEMPLATE_DIR, "conf.py")
PKG_INDEX_RST = os.path.join(PKG_TEMPLATE_DIR, "index.rst")


if __name__ == "__main__":
    if len(sys.argv) == 1:
        print("synopsis: build-pkg-doc.py pkg-name")
        print("")
        print("This script is able to generate a single packages documentation")
        exit(1)

    pkg_name = sys.argv[1]
    rospack = doc_utils.discover_ros_pkgs(BITBOTS_ROOT)

    if not doc_utils.is_doc_enabled(rospack, pkg_name):
        print("Documentation for this package is not enabled.")
        print("Please see bitbots main documentation on how to enable automatic documentation for your package.")
        exit(1)

        doc_utils.build_single_package_doc(rospack, pkg_name, ROSDOC_CONF, SPHINX_CONF, PKG_INDEX_RST)
        doc_utils.copy_pkg_doc(rospack, pkg_name, DOC_DIR)
        doc_utils.build_meta_doc(DOC_DIR)
