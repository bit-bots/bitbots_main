# Review Todos — `feature/teamplayer_tui` & bitbots_cargo (PR #944)

Results of the detailed review of `feature/teamplayer_tui` (40 commits) and a
re-check of build-system findings against the head of PR #944
(`feature/bitbots_cargo`, head `44ca588`).

## Must-fix before merge

- [x] **#1 TUI restart race for multi-cmd components.** `App::drain_messages`
      acts on the first `ProcessExited` by restarting `world_model`
      (ball_filter + robot_filter); the second child's exit message then flips
      the *new* instance to Stopped/Crashed and clears its fresh pids, leaving
      new processes untracked. Needs per-process exit accounting or a pending
      child counter.
- [x] **#2 Partial spawn failure leaks a process.** In `start_component`, if
      the second command of a component fails to spawn, the function returns
      early; the already-spawned child keeps running but its pid is lost on
      the next restart → orphan that survives the final SIGTERM sweep.

## Should-fix

- [x] **#13 `cargo test` without `--profile colcon`.** All three
      `bitbots_cargo` helpers register tests as plain `cargo test`, forcing a
      full second compile in the dev profile. Add `--profile colcon` (plus
      `pyo3/extension-module` where applicable). Confirmed still present on
      PR #944 head.
- [ ] **#14 Two sources of truth for the robot stack.** The TUI hardcodes
      component commands/defaults while `teamplayer.launch` still exists and
      is what the deploy scripts use via tmux; defaults already diverge
      (e.g. `monitoring`). Either migrate the deploy path to the TUI or scope
      it explicitly as an operator/dev tool.
      Decision: migrate. Done: process management split into
      `bitbots_orchestrate` (backend with HTTP-over-Unix-socket API) and the
      `teamplayer` TUI frontend, both in `src/bitbots_bringup`. The stack is
      declared in `bitbots_bringup/config/teamplayer.yaml` (parameters,
      profiles, components with `if`/`requires`/`fallback_for`/`locked`),
      which now also covers the player state aggregator, the workspace
      status publisher and the static map→odom fallback. Open: switch deploy
      to `bitbots_orchestrate --autostart --profile game --disable zenoh`;
      cover the multi-robot sim launch (per-instance environment such as
      `ROS_DOMAIN_ID`); then remove `teamplayer.launch`.

## Nits / polish

- [x] **#4 Zenoh state lives in two places** in the TUI
      (`params.zenoh` vs `components[zenoh].enabled`); `--enable zenoh`
      desyncs them. Harmless today, footgun later.
- [x] **#5 TUI config arrow-key navigation**: Down at bottom of left column
      jumps to Start, skipping the right column (only `→` reaches it).
- [x] **#6 Confirm dialog cancels on *any* key** except y/Y/Enter, not just
      N/Esc as labeled.
- [x] **#7 No tests in the `teamplayer` crate** even though cargo tests are
      now wired into colcon.
- [x] `pixi.toml`: `libclang >= 22` pinned alongside `clang >= 21.1,<22` +
      `libllvm21` — major-version mismatch between the toolchain and the
      libclang bindgen actually uses; probably should be `libclang 21.x`.
      Won't fix: `qt-main` (plotjuggler) requires `libclang13 >=22`, so
      libclang 21.x is unsolvable. Documented in a comment in `pixi.toml`.
- [x] TUI `Cargo.toml`: `clap = "~4.4"` tilde-pin stricter than neighbors;
      `nix 0.27` is dated.
- [x] `ProcState::label()` padding vs `↺` spinner widths in narrow terminals.
- [x] `bitbots_bringup/package.xml` lacks `exec_depend`s for several packages
      the TUI launches at runtime (pre-existing pattern, now baked into the
      TUI).

## Tracked temporary measures (PR #944)

- `[patch.crates-io]` pins r2r crates to the `bit-bots/r2r` fork
  (Clang-compatible bindgen). Remove once an upstream r2r release includes
  the fix (sequenceplanner/r2r#133).
