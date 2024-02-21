# Bit-Bots Development Container

This container is used to develop the bit-bots software. It provides a pre-configured environment with all necessary tools. This repository is mounted into the container and can be used to develop the software. You still need to install some repo specific dependencies, but the container provides a good starting point.

## Usage

### Build

To use the container, you need to have Docker installed. Then you can run the following command to build the base image of the container:

```shell
bash build.bash
```

in the root of this repository (bitbots_main) you can use the make command to build the container:

```shell
make dev-container-build
```

### Run

Once the container is built, you can run it with the following command:

```shell
bash connect.bash
```

in the root of this repository (bitbots_main) you can use the make command to run the container:

```shell
make dev-container-run
```

This will start the container and mount the current repository into the container. You can now use the container to develop the software. You still need to install some repo specific dependencies tho.

To install everything you need to run the following command in the container:

```shell
make -C ~/colcon_ws/src/bitbots_main install
```

### Stop

You can explicitly stop the container with the following command:

```shell
bash stop.bash
```

in the root of this repository (bitbots_main) you can use the make command to stop the container:

```shell
make dev-container-stop
```

### Clean

You can clean up the container with the following command:

```shell
bash clean.bash
```

This will stop and remove the container. You can also use the make command to clean the container:

```shell
make dev-container-clean
```

This is useful if you want to start from scratch or if you have build a new image and want a new container to utilize the new image.

Due to the mounting of this repository the code still persists on your local machine and is not removed when the container is removed.
All changes you make in the container (e.g. installing dependencies) are lost when the container is removed.

## IDE Integration

You can connect VS Code to the container. This allows for proper syntax highlighting and code completion. To do this, you need to install the "Dev Container" extension in VS Code. You can then click into the bottom left corner of the window and select "Attach to running container" (you need to run the container first in a different terminal). This will open a new window with the container's environment. After opening the correct folder, inside the container, you can use the full power of VS Code to develop the software.
Please also install all recommended extensions when prompted in the bottom right corner of the window after opening the folder.

## Known Issues

- I have no internet connection in the container: This is happens when you start the container and then change your network. The DNS server is not updated in the container. You can fix this by manually stopping and running the container again. This will update the DNS server in the container. All changes are preserved, so you don't lose anything (you don't need to clean the container!).
- I have no syntax highlighting for ROS related files: Make sure to open the folder inside the container. Also make sure to install all recommended extensions when prompted in the bottom right corner of the window after opening the folder.
- I only have syntax highlighting for general ros files but not for bit-bots specific imports: You might need to update the python paths with the ROS extension. To do this open the VS Code command palette (Ctrl+Shift+P) and search for "Update Python Path". This will update the python paths and should fix the issue.
