==========================
VSCode Dev Container Setup
==========================

If you don't use the correct Ubuntu version this setup is probably the way to go.
It should work with most other Linux distributions.
Windows users might need to delete the home directory mount from `.devcontainer/devcontainer.json` and are not officially supported.
Also make sure docker is installed and setup.

Install Docker
--------------

To install docker visit https://docs.docker.com/engine/install/ and follow the instructions for your OS.

Install VSCode
--------------

To install VSCode visit https://code.visualstudio.com/ and follow the instructions for your OS.

Setup VSCode Dev Container
--------------------------

1. Clone the repository: `git clone git@github.com:bit-bots/bitbots_main.git` or `git clone https://github.com/bit-bots/bitbots_main.git && git remote set-url origin git@github.com:bit-bots/bitbots_main.git` if you don't have an SSH key setup yet.
2. Open the repository in VSCode
3. Install the "Dev - Containers" extension
4. Click on the green icon in the bottom left corner of the window and select "Reopen in Container"
5. Wait for the container to build and start
6. Open a terminal in VSCode, you should see a number of instructions on how to setup the container. Follow them.
7. Install recommended extensions for the repository

You should now have a fully working development environment (IntelliSense, Build, ...) for the repository.
You still need to setup your SSH keys in the container to be able to push.
Instructions for that are shown in the terminal when the container starts.
To activate the pixi environment in a terminal run `pixi shell` or prefix commands with `pixi run ...`.


Known issues
------------

- Rebuilding the container results in all modifications to the container being lost. This does not include the repository, which itself is persisted in the container.
- GUI applications do not start. Run `xhost local:root` on the **host** machine to fix this.
- I can not find my files in the home directory. The home directory is mounted at `/srv/host_home` in the container. You can find your files there.
