Software installation guide
===========================

In this tutorial, we will learn how to install all dependencies and build our software stack.

**TLDR**: single command setup
------------------------------

**Prerequisites**

- You have an existing GitHub account and added a SSH key to your account.

If you have not previously set up any of our software stack, you can use the following command to install and setup everything in one go:

.. code-block:: bash

  mkdir -p ~/git/bitbots \
    && cd ~/git/bitbots \
    && curl -fsSL https://raw.githubusercontent.com/bit-bots/bitbots_main/main/scripts/setup.sh > /tmp/setup.sh \
    && bash /tmp/setup.sh

Manual steps with in depth explanation
--------------------------------------

**0. Use any modern Linux distribution**

We mainly develop and test our software on Ubuntu so we recommend using Ubuntu for development as well.
Due to the use of pixi other distributions as well as Mac OS might work as well, but might require some tweaks.

Alternatively you can use a devcontainer :doc:`vscode-dev-container`, with a pre-configured environment and follow those instructions, as these docs do not apply to the devcontainer.

**1. Install Pixi**

We manage our development environment with `pixi <https://pixi.sh>`_, which makes setting up and using our software stack much easier.
Run the following command to install pixi for your user:

.. code-block:: bash

  curl -sSL https://pixi.sh/install.sh | bash

**2. Download our software**

- Create a GitHub account, if not already done (see `here <https://doku.bit-bots.de/private/manual/dienste_accounts.html>`_ for further information)
- Add your SSH key to GitHub to access and sync our repositories
    - If you don't know what I am talking about or you don't yet have a SSH key, follow `this guide <https://docs.github.com/en/authentication/connecting-to-github-with-ssh/checking-for-existing-ssh-keys>`_.
    - Go to your account settings and add your SSH key (the ``.pub`` file) to `GitHub <https://github.com/settings/keys>`_
- Now, you can clone (download) our main code repository (repo) called `bitbots_main <https://github.com/bit-bots/bitbots_main>`_:
    - Open a terminal and go to the directory where you want to download our code, e.g. ``~/git/bitbots/``
    - Clone the code repository with: ``git clone git@github.com:bit-bots/bitbots_main.git``
      Confirm the host key by typing ``yes``, if asked.
    - Move into the newly created directory with: ``cd bitbots_main``

**3. Setup the workspace and build the software**

Now that you have downloaded the code, you need to build it.
A number of dependencies will be installed automatically during the build process.
Make sure you have about *10 GB of free disk space* available.
Run the following command in the ``bitbots_main`` directory to build the software:

.. code-block:: bash

  pixi run build --packages-skip bitbots_basler_camera

The compilation of the basler camera driver is skipped, as it requires the Pylon SDK to be installed manually.
If you need the basler camera driver, install the Pylon SDK manually or run `bash scripts/make_basler.sh` if you are using Ubuntu 24.04 and have root access.


Legacy install methods
----------------------

Custom docker setup
  Before utilizing a devcontainer, we used a custom docker setup for ROS 2 development.
  If you want (or need) to utilize a custom setup like this, have a look at https://github.com/timonegk/rosdocked.

Virtual Machine setup
  We recommend against using a virtual machine for ROS 2 development, both for compile speed and setup complexity reasons.
