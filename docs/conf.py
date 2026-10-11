#
# Full list of options at http://www.sphinx-doc.org/en/master/config

# -- Path setup --------------------------------------------------------------

import os

docs_dir = os.path.dirname(os.path.abspath(__file__))

# -- Project information -----------------------------------------------------

project = "bitbots_main"
copyright = "2026, Hamburg Bit-Bots"
author = "Hamburg Bit-Bots"

# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom ones.
extensions = [
    "sphinx.ext.doctest",
    "sphinx.ext.intersphinx",
    "sphinx.ext.todo",
    "sphinx.ext.imgmath",
    "sphinx.ext.viewcode",
    "sphinx_rtd_theme",
]

source_suffix = ".rst"

# The master toctree document.
master_doc = "index"

language = "en"

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ["_build", "_out", "Thumbs.db", ".DS_Store"]

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = None

# -- Options for HTML output -------------------------------------------------

html_theme = "sphinx_rtd_theme"

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ["_static"]

html_logo = os.path.join("_static", "logo.png")
html_favicon = os.path.join("_static", "logo.png")

# -- Options for intersphinx extension ---------------------------------------

intersphinx_mapping = {"python": ("https://docs.python.org/3", None)}

# -- Options for todo extension ----------------------------------------------

# If true, `todo` and `todoList` produce output, else they produce nothing.
todo_include_todos = True
