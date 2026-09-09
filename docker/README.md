# Hamburg Bit-Bots Podman Images

This directory contains `Containerfile`s and helper scripts to create and run Podman containers for the Bit-Bots software stack.

## Image Overview

1.  **bitbots-common**: A common base image based on Ubuntu 24.04. It includes basic utilities, an SSH server, rsync, and the Pixi package manager. It is configured to match the robot's Ansible setup, including Zsh configuration (ZimFW) and environment variables.
2.  **bitbots-project**: Built on top of `bitbots-common`, this image contains the full source code of the project and has the ROS 2 workspace pre-built using `pixi run build`.
3.  **bitbots-target**: Built on top of `bitbots-common`, this is a clean environment intended to be used as a target for the Bit-Bots deploy tool.

## Ansible Alignment

The containers are designed to closely match the environment of real robots managed by Ansible. This includes:
- **Zsh Configuration**: Custom prompt, aliases, and history search via ZimFW.
- **Pixi Availability**: The `pixi` command is available in all shell sessions, including non-interactive SSH (e.g., `ssh bitbots@IP pixi ...`).
- **Dotfiles**: Pre-configured `vimrc`, `tmux.conf`, `screenrc`, and `htoprc` matching the robot setup.

## Prerequisites

- **Podman**: Ensure Podman is installed on your host system.
- **SSH Key**: By default, the build script looks for your public SSH key in `~/.ssh/id_ed25519.pub` or `~/.ssh/id_rsa.pub` to authorize it for the `bitbots` user in the container.

## Usage

A helper script `manage.sh` is provided to simplify common tasks.

### Building Images

Build all images (common, project, and target):
```bash
./docker/manage.sh build-all
```

Or build them individually:
```bash
./docker/manage.sh build-common
./docker/manage.sh build-project
./docker/manage.sh build-target
```

### Running Containers

Run the project image (mapped to port 2222):
```bash
./docker/manage.sh run-project
```

Run the target image (mapped to port 2223):
```bash
./docker/manage.sh run-target
```

### Advanced: Multiple Containers & Static IPs

To run containers with their own IP addresses (avoiding port mapping), first create a Podman network:
```bash
./docker/manage.sh create-network 10.66.0.0/16
```

Then run containers with a specific IP:
```bash
./docker/manage.sh run-target 10.66.6.1
```

Or launch by robot name (resolves IP from `scripts/deploy/known_targets.yaml`):
```bash
./docker/manage.sh run-target mickey
```

### Connecting via SSH

**Using Port Mapping:**
```bash
ssh -p 2222 bitbots@localhost
```

**Using Static IP (Rootful/Sudo):**
Running Podman as root allows it to create a real bridge interface on your host, making container IPs directly routable.

1. **Build the images as root** (Note: images built as user are not visible to sudo):
   ```bash
   sudo SSH_PUB_KEY_PATH=$HOME/.ssh/id_ed25519.pub ./docker/manage.sh build-all
   ```
2. **Create the network**:
   ```bash
   sudo ./docker/manage.sh create-network
   ```
3. **Run a container**:
   ```bash
   sudo ./docker/manage.sh run-target mickey
   ```
4. **Connect directly**:
   ```bash
   ssh bitbots@10.66.6.2
   ```

**Using Static IP (Rootless Network Shell - Recommended):**
In rootless mode (running without `sudo`), container IPs are not directly reachable from your normal host shell. You can use the `net-shell` command to enter the container network namespace:

```bash
./docker/manage.sh net-shell
```

Once inside this special shell:
- All containers are directly reachable via their IPs (e.g., Mickey at `10.66.6.2`).
- The `deploy` script and `ssh` commands work exactly like they do on real robots.
- Common SSH permission issues caused by the namespace mapping are automatically handled.

**Convenience SSH Command:**
Alternatively, you can use the management script to SSH into a container from any shell. This command automatically handles networking (tunneling or direct) and suppresses host key warnings for the virtual network:
```bash
./docker/manage.sh ssh mickey
```

**Using Static IP (Rootless + Proxy):**
If you don't want to use `net-shell` or the convenience `ssh` command, you can "tunnel" SSH through Podman by adding this to your `~/.ssh/config`.

### Recommended SSH Configuration
To make "normal" SSH work with virtual containers (especially in rootless mode) and to avoid "REMOTE HOST IDENTIFICATION HAS CHANGED" warnings, add the following to your `~/.ssh/config`:

```ssh
# Configuration for Bit-Bots Virtual Containers
Host 10.66.*
    User bitbots
    StrictHostKeyChecking no
    UserKnownHostsFile /dev/null
    LogLevel ERROR
    # Tunnel via podman exec if not directly reachable (rootless mode)
    ProxyCommand bash -c 'if ! ping -c 1 -W 1 %h >/dev/null 2>&1; then podman exec -i $(podman ps --format "{{.ID}} {{.Names}}" | grep bitbots | grep "${1//./-}" | head -n 1 | cut -d" " -f1) nc localhost 22; else nc %h 22; fi' -- %h
```
*Note: This configuration allows `ssh 10.66.6.x` and the `deploy` script to work seamlessly across both rootless (via proxy) and rootful/net-shell (direct) environments.*

### Network Connectivity (Zenoh, etc.)

- **Rootful Mode (`sudo`):** Provides full network transparency. The host will have a bridge interface (e.g., `podman1`) and can communicate with containers via their IPs for all protocols (TCP/UDP/Zenoh). Recommended for complex network testing.
- **Rootless Mode:** Containers can talk to each other on the `bitbots-net` network, but the host cannot reach them by IP (except via the SSH proxy above). For other services like Zenoh, you should use port mapping (`-p`) or run Zenoh routers inside the container network.

### GPU Access and Acceleration
Containers automatically pass available GPU devices (DRI nodes via `--device /dev/dri`, AMD `/dev/kfd`, and NVIDIA GPUs via `--gpus all` / NVIDIA Container Toolkit) into the container when started via `manage.sh` or `manage_docker.sh`.
You can customize or override the GPU flags by setting the `GPU_ARGS` environment variable:
```bash
GPU_ARGS="--gpus all --device /dev/dri" ./docker/manage.sh run-target mickey
```

### X11 Forwarding
If you want to run GUI applications (like RViz or MuJoCo viewer) from within the container, use the `-X` or `-Y` flag with SSH:
```bash
ssh -X -p 2222 bitbots@localhost
```
Note: This requires an X server running on your host machine.

### Using with the Deploy Tool
With static IPs, the deploy tool can interact with containers just like real robots:
1. Start a container: `./docker/manage.sh run-target mickey`
2. Deploy: `pixi run deploy mickey` (it will resolve the IP `10.66.6.2` from `known_targets.yaml`)

Alternatively, if using port mapping:
```bash
pixi run deploy bitbots@localhost:2223 --workspace /home/bitbots/bitbots_main
```

### Stopping Containers

Stop and remove the running Bit-Bots containers:
```bash
./docker/manage.sh stop-all
```

## Manual Commands

If you prefer to run Podman commands manually, ensure you are in the repository root:

**Build Common:**
```bash
podman build -t bitbots-common --build-arg ssh_pub_key="$(cat ~/.ssh/id_ed25519.pub)" -f docker/Containerfile.common .
```

**Build Project:**
```bash
podman build -t bitbots-project -f docker/Containerfile.project .
```

**Run:**
```bash
podman run -d --name bitbots-project-run -p 2222:22 bitbots-project
```
