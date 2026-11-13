Development Guidelines
======================

.. todo::
   Rework of the public documentation (see issue #1037): flesh out the sections
   below into full prose and link to the relevant tooling.

How We Develop
--------------

- Create a branch named ``feature/*``, ``fix/*``, or ``docs/*``.
- Always create a pull request for anything that may be merged. If it is not
  finished yet, create a draft pull request.
- Once it is ready, ask for review from the relevant authors and respond to and
  fix those comments.
- CI needs to pass.
- If you do not have the capacity to make a pull request right now, open an
  issue instead.
- Add pull requests and issues to the project board.

Coding Style
------------

To maintain a consistent coding style throughout the codebase, we use automatic formatting tools.
For this, we use `pre-commit <https://pre-commit.com/>`_ hooks that automatically format the code when a commit is made.
Our configuration can be found in the ``.pre-commit-config.yaml`` file in the root of the repository.
Continuous Integration (CI) also checks if the code is formatted correctly.

Setting up pre-commit
~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

  pre-commit install

Running pre-commit manually
~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you want to run pre-commit manually on all files, you can use the following command:

.. code-block:: bash

  pixi run format

Git Commit conventions
----------------------

We also have some conventions about how we want to use git. They are as follows:

* Commits are written like ``when this commit is applied, it will <commit message>``.
* Commits consist of a heading, newline and optional description.
    * the heading follows `Conventional Commits <https://www.conventionalcommits.org>`_, such as ``fix(motion): ...``.
    * the description should not elaborate further on what was done but rather explain why something was done.
* Branch naming, pull requests, labels, and project triage are described in
  `CONTRIBUTING.md <https://github.com/bit-bots/bitbots_main/blob/main/CONTRIBUTING.md>`_.
