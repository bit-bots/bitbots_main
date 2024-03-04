==========================
VSCode Dev Container Setup
==========================

If you don't use the correct Ubuntu version this setup is probably the way to go.
It should work with most other Linux distributions.
Windows users might need to delete the home directory mount from `.devcontainer/devcontainer.json` and are not officially supported.
Also make sure docker is installed and setup.

Install Dokcer
--------------

To install docker visit https://docs.docker.com/engine/install/ and follow the instructions for your OS.

Install VSCode
--------------

To install VSCode visit https://code.visualstudio.com/ and follow the instructions for your OS.

Setup VSCode Dev Container
--------------------------

1. Clone the repository: `git clone git@github.com:bit-bots/bitbots_main.git` or `git clone https://github.com/bit-bots/bitbots_main.git && git remote set-url origin git@github.com:bit-bots/bitbots_main.git` if you don't have an SSH key setup yet.
2. Open the repository in VSCode
3. Install the "Remote - Containers" extension
4. Click on the green icon in the bottom left corner of the window and select "Reopen in Container"
5. Wait for the container to build and start
6. Open a terminal in VSCode, you should see a number of instructions on how to setup the container. Follow them.
7. Install recommended extensions for the repository

You should now have a fully working development environment (IntelliSense, Build, ...) for the repository. You can source the workspace by running `sa`. Now all the commands should be available to you.


Known issues
------------

- Rebuilding the container results in all modifications to the container being lost. This does not include the repository, which itself is persisted in the container.
- Sometimes `make install` results in an `mktemp: failed to create file via template ‘/tmp/tmp.XXXXXXXXXX’: Permission denied`. I spend some time trying to fix this but couldn't find a solution. The workaround is to run `make install` again. This time it should work.
- I did everything as stated, but my python IntelliSense does not pick up bit-bots related packages. To solve this open the command palette (Ctrl+Shift+P) and run `ROS: Update Python Path`. This should fix the issue.
- GUI applications do not start. Run `xhost local:root` on the **host** machine to fix this.
- I can not find my files in the home directory. The home directory is mounted at `/srv/host_home` in the container. You can find your files there.
