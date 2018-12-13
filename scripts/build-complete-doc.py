#!/usr/bin/env python3

import os
import shutil
import subprocess
import doc_utils


BITBOTS_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

DOC_DIR = os.path.join(BITBOTS_ROOT, "doc")
TEMPLATE_DIR = os.path.join(DOC_DIR, "templates")
PKGS_DST_DIR = os.path.join(DOC_DIR, "packages")

ROSDOC_CONF = os.path.join(TEMPLATE_DIR, "rosdoc.yaml")
SPHINX_CONF = os.path.join(TEMPLATE_DIR, "conf.py")
PKG_INDEX_RST = os.path.join(TEMPLATE_DIR, "index.rst")

MODULES_AUTOGEN_TMPL = os.path.join(TEMPLATE_DIR, "modules-autogen.rst")


def clean_doc_dir():
    if os.path.exists(os.path.join(DOC_DIR, "html")):
        shutil.rmtree(os.path.join(DOC_DIR, "html"))

    if os.path.exists(PKGS_DST_DIR):
        shutil.rmtree(PKGS_DST_DIR)


if __name__ == "__main__":
    shutil.copy(MODULES_AUTOGEN_TMPL, os.path.join(DOC_DIR, "modules.rst"))

    clean_doc_dir()

    rospack = doc_utils.discover_ros_pkgs(BITBOTS_ROOT)
    for pkg_name in rospack.list():
        if doc_utils.is_doc_enabled(rospack, pkg_name):

            doc_utils.build_single_package_doc(rospack, pkg_name, ROSDOC_CONF, SPHINX_CONF, PKG_INDEX_RST)

            doc_dst_dir = os.path.join(PKGS_DST_DIR, pkg_name)
            doc_utils.copy_pkg_doc(rospack, pkg_name, doc_dst_dir)

            doc_utils.clean_pkg(rospack, pkg_name)

            with open(os.path.join(DOC_DIR, "modules.rst"), "a") as f:
                f.write("   %s/%s/modules\n" % (PKGS_DST_DIR, pkg_name))

            print("Documentation of %s copied to %s" % (pkg_name, doc_dst_dir))

    doc_utils.build_meta_doc(DOC_DIR)
