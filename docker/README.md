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

Run the simulator container (named `simulator`, starts Zenoh router and SSH server, and exposes port 8080 for web visualizer/tools):
```bash
./docker/manage.sh run-simulator
# Or with a specific IP / robot identifier:
./docker/manage.sh run-simulator 10.66.6.10
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
To make "normal" SSH work with virtual containers across direct, bridged, and rootless environments (and to avoid "REMOTE HOST IDENTIFICATION HAS CHANGED" warnings), add the following to your `~/.ssh/config`:

```ssh
# Configuration for Bit-Bots Virtual Containers
Host 10.66.*
    User bitbots
    StrictHostKeyChecking no
    UserKnownHostsFile /dev/null
    LogLevel ERROR
    # Tunnel via docker/podman proxy when container IP is not directly reachable
    ProxyCommand bash -c 'if ! ping -c 1 -W 1 %h >/dev/null 2>&1; then docker exec -i $(docker ps --filter "network=bitbots-net" -q 2>/dev/null | head -n 1) nc %h 22 2>/dev/null || podman exec -i $(podman ps --format "{{.ID}} {{.Names}}" 2>/dev/null | grep bitbots | grep "${1//./-}" | head -n 1 | cut -d" " -f1) nc localhost 22 2>/dev/null; else nc %h 22; fi' -- %h
```
*Note: This configuration allows `ssh 10.66.6.x` and the `deploy` script to work seamlessly across direct host routing (`connect-host`), rootless proxying, and `net-shell` environments.*

### Network Connectivity (Zenoh, ROS Domain IDs, etc.)

- **Zenoh Routers:** All containers automatically start a Zenoh router (`rmw_zenohd`) via the entrypoint.
  - The `simulator` container runs the Zenoh router in **router** mode on `tcp/simulator:7447` (and exposes port 8080 for web visualizer/tools).
  - All non-simulator robot/project/target containers run the Zenoh router in **peer** mode targeting the simulator router (`tcp/simulator:7447`).
- **ROS Domain IDs:** Target and robot containers have their `ROS_DOMAIN_ID` automatically configured based on `scripts/deploy/known_targets.yaml` (domain IDs 11 to 16 for Kalliope, Mickey, Pink, Romeo, Carrie, and Peter). The simulator container defaults to `ROS_DOMAIN_ID=0`.
- **Direct Host Routing (`connect-host` / `sudo`):** Provides full network transparency. The host has a virtual interface on `10.66.0.0/16` (e.g., `veth-bb-host` with IP `10.66.0.254` in Docker, or `podman1` in Podman) and can communicate directly with all containers via their IPs for all protocols (SSH, TCP, UDP, Zenoh, ROS 2). Recommended for testing and deployment.
- **Rootless / Namespace Mode:** Containers can talk to each other on the `bitbots-net` network. To access them from the host, you can use `./docker/manage_docker.sh connect-host` (or `./docker/manage.sh net-shell`), the SSH proxy above, or port mapping (`-p`).

### Multi-PC Networking with Docker Swarm Overlays

When running multiple robot containers across different physical PCs, Docker Swarm attachable overlay networks connect all containers into the same virtual subnet (`10.66.0.0/16`).

Use `docker/manage_docker.sh` on the participating machines:

1. **Initialize Docker Swarm on the Manager PC:**
   ```bash
   ./docker/manage_docker.sh swarm-init
   ```
   This will initialize Swarm and print the join command for other PCs.

2. **Create the Attachable Overlay Network on the Manager PC:**
   ```bash
   ./docker/manage_docker.sh create-network 10.66.0.0/16
   ```

3. **Join Other PCs to the Swarm:**
   Run the `swarm-join` command on each worker PC using the token and manager IP:
   ```bash
   ./docker/manage_docker.sh swarm-join <worker-token> <manager-ip>:2377
   ```

4. **Launch Containers on Any Connected PC:**
   - On PC 1: `./docker/manage_docker.sh run-target mickey` (assigns `10.66.6.2`)
   - On PC 2: `./docker/manage_docker.sh run-target minnie` (assigns `10.66.6.1`)

5. **Connect the Host PC Directly to the Overlay Network:**
   To make the host PC itself part of the `10.66.0.0/16` network (allowing direct `ssh bitbots@10.66.6.x` and `pixi run deploy <robot>` from your host shell to local and remote containers), run on each PC:
   ```bash
   ./docker/manage_docker.sh connect-host
   ```
   This launches a lightweight gateway container on `bitbots-net` and configures local point-to-point host routing. Because the gateway container is a native Docker Swarm overlay endpoint, Docker Swarm's SDN control plane synchronizes routing tables across all connected PCs, enabling bidirectional communication with containers on other PCs as well as the local PC.

Containers and host machines across different PCs can communicate directly over their `10.66.0.0/16` IP addresses and container names across the overlay network.

*Note on Firewall / Network Ports:* Ensure the following ports are open between the PCs:
- `TCP 2377`: Docker Swarm cluster management
- `TCP/UDP 7946`: Node communication and control plane
- `UDP 4789`: VXLAN overlay data traffic

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
