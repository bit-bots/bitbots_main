===============
How to document
===============

Our documentation is published under  `doku.bit-bots.de <http://doku.bit-bots.de>`_ and will automatically be regenerated from the package `bitbots_meta/bitbots_tools/bitbots_docs <https://github.com/bit-bots/bitbots_tools/tree/master/bitbots_docs>`_ and deployed by our `Jenkins CI <http://ci.bit-bots.de>`_.

Installation of system dependencies
===================================

We are using `Sphinx <https://www.sphinx-doc.org/>`_ and the following extentions to generate our documentation: `breathe`, `exhale`.
The correct version of the extentions must be installed.

.. code-block:: bash

        sudo apt remove python-sphinx
        sudo apt install python3-sphinx python3-sphinx-rtd-theme python3-breathe
        pip3 install exhale --user


.. _build_documentation:

How to build the documentation
==============================

* To build the general documentation (tutorials etc. including this one), run the following command:

  .. code-block:: bash

          catkin build bitbots_docs --no-deps --make-args Documentation

* To build documentation for another package (with :ref:`activated<activate_docs_for_package>` documentation), run the following command by replacing ``<package>`` by your package name:

  .. code-block:: bash

          catkin build <package> --no-deps --make-args Documentation

* If you happen to build documentation a lot and don't want to always type the whole command, you can `define a catkin alias <https://catkin-tools.readthedocs.io/en/latest/advanced/verb_customization.html>`_
  by creating the following file:

  .. code-block:: yaml

          # ~/.config/catkin/verb_aliases/01-doc.yaml
          ---
          doc: build --make-args Documentation --


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

#) ``package.xml``:
    The ``package.xml`` describes the package to the build system.
    You need to add a dependency to ``bitbots_docs``.

    .. code-block:: xml

        <package>
            <depend>bitbots_docs</depend>
        </package>

#) ``CMakeLists.txt``:
    This file describes how exactly a package is built.
    First we need to add the ``bitbots_docs`` dependency here as well in order to make additional cmake
    commands available.

    .. code-block:: cmake

        find_package(catkin COMPONENT bitbots_docs)

    This registered the command ``enable_bitbots_docs()`` which is used to register the `Documentation`
    target for this package and thus enables building documentation.

    .. code-block:: cmake

        enable_bitbots_docs()

#) ``.gitignore``:
    These additions are not strictly necessary but since we use git for all our packages you should do it
    anyways. It is only required once per repository and not per package.

    .. code-block:: text

        # auto-generated documentation
        **/docs/_build
        **/docs/_out
        **/docs/cppapi
        **/docs/pyapi

.. note:: See :doc:`../software/ci` for information about building the documentation automatically via Jenkins
