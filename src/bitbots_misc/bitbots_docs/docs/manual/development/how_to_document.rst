===============
How to document
===============

Our documentation is published under `docs.bit-bots.de <https://docs.bit-bots.de>`_ and is automatically regenerated from the packages in the `bitbots_main repository <https://github.com/bit-bots/bitbots_main>`_.

Dependencies
============

The documentation is generated with `Sphinx <https://www.sphinx-doc.org/>`_ and the extensions `breathe` and `exhale`.
The Pixi development environment provides all of them, so no additional installation is necessary.

.. _build_documentation:

How to build the documentation
==============================

1. Change to the package with the documentation to build.
   The general documentation (tutorials etc. including this one) is in ``src/bitbots_misc/bitbots_docs``.

2. Build the Sphinx docs from the package root:

  .. code-block:: bash

        pixi run -e default sphinx-build docs docs/_out -b html

3. Open ``docs/_out/index.html`` in a browser.

To build the documentation of another package, go to that package (e.g. ``bitbots_vision``) and run steps 2. and 3. there.

How to write documentation for a package
========================================

In every package (with :ref:`activated<activate_docs_for_package>` documentation) you can find a directory, called ``docs/`` including the configuration file (``docs/conf.py``) and the root-document (``docs/index.rst``).

Preferably create your ``.rst`` documents in the directory ``docs/manual``, then reference them in the ``docs/index.rst`` as follows::

    .. toctree::
        :maxdepth: 1
        :glob:
        :caption: Manuals:

        manual/*


.. _activate_docs_for_package:

Activate documentation for a package
====================================

To be able to actually succeed in building documentation for a package as
:ref:`described above <build_documentation>` that package must have documentation enabled.
This can be done with the following steps and automatically creates the files ``docs/conf.py`` and
``docs/index.rst``

#) Initialize the directory structure:
    To initialize the documentation directory structure in the given package copy the docs folder from another package (e.g. ``bitbots_docs``). Remember to remove the files in ``docs/manual`` as well as their references in the ``docs/index.rst`` file, so you can start with a clean docs setup.

#) ``.gitignore``:
    These additions are not strictly necessary but since we use git for all our packages you should do it
    anyways. It is only required once per repository and not per package.

    .. code-block:: text

        # auto-generated documentation
        **/docs/_build
        **/docs/_out
        **/docs/cppapi
        **/docs/pyapi
