#!/usr/bin/env python3

import os
import doc_utils


BITBOTS_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

DOC_DIR = os.path.join(BITBOTS_ROOT, "doc")


if __name__ == "__main__":
    doc_utils.build_meta_doc(DOC_DIR)