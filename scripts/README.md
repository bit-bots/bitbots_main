# Deployment Supervisor

The deployment supervisor discovers configured robots, maintains one shared
OpenSSH connection per robot, and reconciles every selected robot independently.
It supports both an interactive TUI and batch operation.

## Offline environment

There is one canonical robot environment: the normal Pixi `robot` environment
at `.pixi/envs/robot`. Match operation, automated deployment, and interactive
development all use this same environment.

Robot networks do not have Internet access. While the deployment laptop can
reach the package channels, prepare a package cache for the exact locked
`linux-aarch64` robot environment:

```shell
pixi run deploy cache prepare
```

`pixi-pack` creates a local Conda channel and collects the locked PyPI wheels in
the user's XDG cache. Cache preparation limits downloads to four concurrent
requests and solving to one worker so it remains usable on development
machines.

During deployment, the laptop temporarily serves this cache over the wired LAN.
The supervisor gives `pixi install -e robot --frozen` a session-only mirror
configuration, so Pixi installs or verifies the normal `.pixi/envs/robot`
environment without contacting the Internet. Package hashes remain those from
`pixi.lock`. The mirror configuration is passed only to the provisioning
command and does not alter the robot's normal Pixi configuration.

After deployment, automated commands use `pixi run -e robot --as-is` to avoid
implicit package operations. A developer connected by SSH can use Pixi normally
without an activation wrapper:

```shell
pixi run -e robot build --packages-select bitbots_bringup
pixi run -e robot test --packages-select bitbots_path_planning
pixi shell -e robot
```

Normal commands for the already-locked environment work without Internet.
Adding or updating dependencies still requires Internet access or preparation
and deployment of a new matching cache.

## Interactive operation

```shell
pixi run deploy
```

The TUI continuously probes `scripts/deploy/known_targets.yaml`. Select robots,
stage match, ball, component, source, and build changes, then apply them. Mouse
and keyboard controls are equivalent. **Attach** suspends the deploy TUI and
opens the selected robot's `teamplayer` tmux session.

Enter an arbitrary hostname or `user@host` in the host field to add it manually.

## Batch operation

```shell
pixi run deploy apply --match german_open_2026 nuc1 nuc2
pixi run deploy apply --no-build --no-launch nuc1
pixi run deploy status ALL
pixi run deploy status --json nuc1
```

Connection loss is retried indefinitely at a fixed two-second interval by
default. Run `pixi run deploy --help` and the subcommand help for all options.

## Operational assumptions

- Pixi is installed on every robot and available to non-interactive SSH
  commands. The deploy tool does not install or update Pixi itself.
- The deployment laptop is connected to the wired robot LAN and remains
  reachable while Pixi provisions missing packages.
- Source synchronization copies `pixi.toml` and `pixi.lock` before environment
  provisioning. It deliberately preserves `.pixi/`, build output, and `.deploy/`
  on the robot.
- `--no-sync` assumes the robot already has a manifest and robot lock matching
  the prepared cache. A mismatch fails instead of attempting Internet access.
- The prepared cache covers exactly the lockfile's `robot` environment for
  `linux-aarch64`. It is not a general package mirror.
- All locked PyPI dependencies must be wheels. This is a `pixi-pack`
  requirement and is checked during cache preparation.
- Robots provide SSH, rsync, tmux, curl, NetworkManager, and passwordless
  `sudo -n` for the approved NetworkManager activation command.
- Interrupted package transfers and connection loss are retried indefinitely.
  Invalid manifests, lockfiles, or unavailable packages are deterministic
  failures and wait for an operator retry or configuration change.

## Profiles

`scripts/deploy/profiles.yaml` contains fleet-wide match and ball profiles.
Match profiles reference existing NetworkManager connection names; the tool
never creates or distributes WiFi credentials. An empty profile leaves WiFi
unchanged. Robot-specific settings are rendered to
`.deploy/config/game_settings.yaml` in the remote workspace.
