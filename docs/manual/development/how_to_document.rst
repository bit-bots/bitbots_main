===============
How to document
===============

Our documentation is published under `docs.bit-bots.de <https://docs.bit-bots.de>`_ and is generated with `Sphinx <https://www.sphinx-doc.org/>`_.
All documentation for this repository lives in the top-level ``docs`` directory.

Dependencies
============

The Pixi ``docs`` environment provides Sphinx and the theme used for the build, so no additional installation is necessary.

.. _build_documentation:

How to build the documentation
==============================

Build the documentation from the workspace root:

  .. code-block:: bash

        pixi run -e docs docs

The result is written to ``docs/_build/index.html``, which can be opened in a browser.

How to write documentation
==========================

Create your ``.rst`` documents somewhere under ``docs/manual``, then reference them in the ``docs/index.rst`` toctree of the matching section::

    .. toctree::
        :maxdepth: 1
        :caption: Development:

        manual/development/development_guidelines

Use the same heading underline style as the surrounding pages.
Build the documentation after adding a page to make sure it is referenced and renders correctly.
