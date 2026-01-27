Coding Style
============

To maintain a consistent coding style throughout the codebase, we use automatic formatting tools.
For this, we use `pre-commit <https://pre-commit.com/>`_ hooks that automatically format the code when a commit is made.
Our configuration can be found in the ``.pre-commit-config.yaml`` file in the root of the repository.
Continuous Integration (CI) also checks if the code is formatted correctly.

Setting up pre-commit
---------------------

.. code-block:: bash

  pre-commit install

Running pre-commit manually
---------------------------

If you want to run pre-commit manually on all files, you can use the following command:

.. code-block:: bash

  pixi run format

Git Commit conventions
======================

We also have some conventions about how we want to use git. They are as follows:

* Commits are written like ``when this commit is applied, it will <commit message>``.
* Commits consist of a heading, newline and optional description.
    * the heading is a short summary of the changes.
    * the heading should start with a prefix like ``ci:``, ``walking:``, etc to show which part of the repository is affected.
    * the description should not elaborate further on what was done but rather explain why something was done.
* Any change should be done in a branch and then PRed to master.
    * branch names should be prefixed with ``feature/``, ``fix/``, ``refactor/`` or none if not applicable.
    * merged branches can be deleted to keep the repo clean of any clutter.
