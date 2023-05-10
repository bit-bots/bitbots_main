===============
How to document
===============

Our documentation is published under `doku.bit-bots.de <https://doku.bit-bots.de>`_ and `docs.bit-bots.de <https://docs.bit-bots.de>`_ and will automatically be regenerated from the package `bitbots_meta/bitbots_tools/bitbots_docs <https://github.com/bit-bots/bitbots_tools/tree/master/bitbots_docs>`_.

Installation of dependencies
============================

We are using `Sphinx <https://www.sphinx-doc.org/>`_ and the following extentions to generate our documentation: `breathe`, `exhale`.
The correct version of the extentions must be installed.

.. code-block:: bash

        sudo apt install python3-sphinx python3-sphinx-rtd-theme python3-breathe
        pip3 install exhale --user


How to build the documentation
==============================

1. Go to the package with the general documentation (tutorials etc. including this one). Said package is called  ``bitbots_docs``.

  .. code-block:: bash
            
        cd bitbots_meta/bitbots_tools/bitbots_docs
        
2. Build the sphinx docs for the given package.
        
  .. code-block:: bash
   
        sphinx-build docs docs/_out -b html
        
3. Open the docs with firefox.
        
  .. code-block:: bash
   
        firefox docs/_out/index.html

To build and view documentation for another package go to the ROS package (e.g. ``bitbots_vision``) and run steps 2. and 3. there.

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
