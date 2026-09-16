# Hamburg Bit-Bots Container Images

This directory contains `Containerfile`s and helper scripts to create and run Docker/Podman containers for the Bit-Bots software stack.

## Image Overview

1.  **bitbots-base**: A base image based on Ubuntu 24.04. It includes basic utilities, an SSH server, rsync, the Pixi package manager, and a pre-built workspace of the project. It is configured to match the robot's Ansible setup, including Zsh configuration (ZimFW) and environment variables.
2.  **bitbots-project**: Built on top of `bitbots-base`, this image configures SSH access and entrypoint scripts, and copies and incrementally rebuilds the ROS 2 workspace.

## Ansible Alignment

The containers are designed to closely match the environment of real robots managed by Ansible. This includes:
- **Zsh Configuration**: Custom prompt, aliases, and history search via ZimFW.
- **Pixi Availability**: The `pixi` command is available in all shell sessions, including non-interactive SSH (e.g., `ssh bitbots@IP pixi ...`).
- **Dotfiles**: Pre-configured `vimrc`, `tmux.conf`, `screenrc`, and `htoprc` matching the robot setup.

## Prerequisites

- **Docker or Podman**: Ensure Docker (default) or Podman is installed on your host system.
- **SSH Key**: By default, the build script looks for your public SSH key in `~/.ssh/id_ed25519.pub` or `~/.ssh/id_rsa.pub` to authorize it for the `bitbots` user in the container.

## Usage

A helper script `manage.py` is provided to simplify common tasks. It supports both Docker (default) and Podman via the `-e/--engine` flag or the `CONTAINER_ENGINE` environment variable.

### Building Images

Build all images (base and project):
```bash
./docker/manage.py build-all
# Or with Podman:
./docker/manage.py -e podman build-all
```

Or build them individually:
```bash
./docker/manage.py build-base
./docker/manage.py build-project
```

### Running Containers

Run the project image (mapped to port 2222):
```bash
./docker/manage.py run-project
```

Run the simulator container (named `simulator`, exposes port 8080 for web visualizer/tools; optionally pass `-z`/`--zenoh-router` to start the Zenoh router):
```bash
./docker/manage.py run-simulator
# Or start with Zenoh router enabled:
./docker/manage.py run-simulator -z
# Or with a specific IP / robot identifier:
./docker/manage.py run-simulator 10.66.6.10
```

### Advanced: Multiple Containers & Static IPs

To run containers with their own IP addresses (avoiding port mapping), first create the network:
```bash
./docker/manage.py create-network 10.66.0.0/16
```

Then run containers with a specific IP:
```bash
./docker/manage.py run-project 10.66.6.1
```

Or launch by robot name (resolves IP from `scripts/deploy/known_targets.yaml`):
```bash
./docker/manage.py run-project mickey
```

### Connecting via SSH

**Using Port Mapping:**
```bash
ssh -p 2222 bitbots@localhost
```

**Using Static IP (Rootful/Sudo):**
Running as root allows creation of a bridge interface on your host, making container IPs directly routable.

1. **Build the images as root** (Note: images built as user are not visible to sudo):
   ```bash
   sudo SSH_PUB_KEY_PATH=$HOME/.ssh/id_ed25519.pub ./docker/manage.py build-all
   ```
2. **Create the network**:
   ```bash
   sudo ./docker/manage.py create-network
   ```
3. **Run a container**:
   ```bash
   sudo ./docker/manage.py run-project mickey
   ```
4. **Connect directly**:
   ```bash
   ssh bitbots@10.66.6.2
   ```

**Using Static IP (Rootless Network Shell - Recommended):**
In rootless mode (running without `sudo`), container IPs are not directly reachable from your normal host shell. You can use the `net-shell` command to enter the container network namespace:

```bash
./docker/manage.py net-shell
```

Once inside this special shell:
- All containers are directly reachable via their IPs (e.g., Mickey at `10.66.6.2`).
- The `deploy` script and `ssh` commands work exactly like they do on real robots.
- Common SSH permission issues caused by the namespace mapping are automatically handled.

**Convenience SSH Command:**
Alternatively, you can use the management script to SSH into a container from any shell. This command automatically handles networking (tunneling or direct) and suppresses host key warnings for the virtual network:
```bash
./docker/manage.py ssh mickey
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

- **Zenoh Routers:** Containers only start a Zenoh router (`rmw_zenohd`) when explicitly requested (via `-z`/`--zenoh-router` in `manage.py` or by setting `START_ZENOH_ROUTER=1`).
  - When enabled in the `simulator` container, it runs the Zenoh router in **router** mode on `tcp/simulator:7447` (and exposes port 8080 for web visualizer/tools).
  - Other containers run without the Zenoh router by default unless explicitly started.
- **ROS Domain IDs:** Target and robot containers have their `ROS_DOMAIN_ID` automatically configured based on `scripts/deploy/known_targets.yaml` (domain IDs 11 to 16 for Kalliope, Mickey, Pink, Romeo, Carrie, and Peter). The simulator container defaults to `ROS_DOMAIN_ID=0`.
- **Direct Host Routing (`connect-host` / `sudo`):** Provides full network transparency. The host has a virtual interface on `10.66.0.0/16` (e.g., `veth-bb-host` with IP `10.66.0.254` in Docker, or `podman1` in Podman) and can communicate directly with all containers via their IPs for all protocols (SSH, TCP, UDP, Zenoh, ROS 2). Recommended for testing and deployment.
- **Rootless / Namespace Mode:** Containers can talk to each other on the `bitbots-net` network. To access them from the host, you can use `./docker/manage.py connect-host` (or `./docker/manage.py net-shell`), the SSH proxy above, or port mapping (`-p`).

### Multi-PC Networking with Docker Swarm Overlays

When running multiple robot containers across different physical PCs, Docker Swarm attachable overlay networks connect all containers into the same virtual subnet (`10.66.0.0/16`).

Use `docker/manage.py` on the participating machines:

1. **Initialize Docker Swarm on the Manager PC:**
   ```bash
   ./docker/manage.py swarm-init
   ```
   This will initialize Swarm and print the join command for other PCs.

2. **Create the Attachable Overlay Network on the Manager PC:**
   ```bash
   ./docker/manage.py create-network 10.66.0.0/16
   ```

3. **Join Other PCs to the Swarm:**
   Run the `swarm-join` command on each worker PC using the token and manager IP:
   ```bash
   ./docker/manage.py swarm-join <worker-token> <manager-ip>:2377
   ```

4. **Launch Containers on Any Connected PC:**
   - On PC 1: `./docker/manage.py run-project mickey` (assigns `10.66.6.2`)
   - On PC 2: `./docker/manage.py run-project minnie` (assigns `10.66.6.1`)

5. **Connect the Host PC Directly to the Overlay Network:**
   To make the host PC itself part of the `10.66.0.0/16` network (allowing direct `ssh bitbots@10.66.6.x` and `pixi run deploy <robot>` from your host shell to local and remote containers), run on each PC:
   ```bash
   ./docker/manage.py connect-host
   ```
   This launches a lightweight gateway container on `bitbots-net` and configures local point-to-point host routing. Because the gateway container is a native Docker Swarm overlay endpoint, Docker Swarm's SDN control plane synchronizes routing tables across all connected PCs, enabling bidirectional communication with containers on other PCs as well as the local PC.

Containers and host machines across different PCs can communicate directly over their `10.66.0.0/16` IP addresses and container names across the overlay network.

*Note on Firewall / Network Ports:* Ensure the following ports are open between the PCs:
- `TCP 2377`: Docker Swarm cluster management
- `TCP/UDP 7946`: Node communication and control plane
- `UDP 4789`: VXLAN overlay data traffic

### GPU Access and Acceleration
Containers automatically pass available GPU devices (DRI nodes via `--device /dev/dri`, AMD `/dev/kfd`, and NVIDIA GPUs via `--gpus all` / NVIDIA Container Toolkit) into the container when started via `manage.py`.
You can customize or override the GPU flags by setting the `GPU_ARGS` environment variable or the `--gpu-args` flag:
```bash
./docker/manage.py --gpu-args "--gpus all --device /dev/dri" run-project mickey
```

### X11 Forwarding
If you want to run GUI applications (like RViz or MuJoCo viewer) from within the container, use the `-X` or `-Y` flag with SSH:
```bash
ssh -X -p 2222 bitbots@localhost
```
Note: This requires an X server running on your host machine.

### Using with the Deploy Tool
With static IPs, the deploy tool can interact with containers just like real robots:
1. Start a container: `./docker/manage.py run-project mickey`
2. Deploy: `pixi run deploy mickey` (it will resolve the IP `10.66.6.2` from `known_targets.yaml`)

Alternatively, if using port mapping:
```bash
pixi run deploy bitbots@localhost:2222 --workspace /home/bitbots/bitbots_main
```

### Stopping Containers

Stop and remove the running Bit-Bots containers:
```bash
./docker/manage.py stop-all
```

## Manual Commands

If you prefer to run container engine commands manually, ensure you are in the repository root:

**Build Base:**
```bash
docker build -t bitbots-base -f docker/Containerfile.base .
```

**Build Project:**
```bash
docker build -t bitbots-project --build-arg ssh_pub_key="$(cat ~/.ssh/id_ed25519.pub)" -f docker/Containerfile.project .
```

**Run:**
```bash
docker run -d --name bitbots-project-run -p 2222:22 bitbots-project
```
