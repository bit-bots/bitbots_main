# Deployment Supervisor

The deployment supervisor discovers configured robots, maintains one shared
OpenSSH connection per robot, and reconciles every selected robot independently.
It supports both an interactive TUI and batch operation.

## Offline environment

Robot networks do not have Internet access. Prepare the exact `linux-aarch64`
environment bundle while the deployment laptop can reach the package channels:

```shell
pixi run deploy cache prepare
```

The bundle is stored in the user's XDG cache and keyed by the locked `robot`
environment. During deployment, the laptop serves it over the wired LAN and the
robot verifies its SHA-256 digest before activation.

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

## Profiles

`scripts/deploy/profiles.yaml` contains fleet-wide match and ball profiles.
Match profiles reference existing NetworkManager connection names; the tool
never creates or distributes WiFi credentials. An empty profile leaves WiFi
unchanged. Robot-specific settings are rendered to
`.deploy/config/game_settings.yaml` in the remote workspace.
