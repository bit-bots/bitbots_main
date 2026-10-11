# Review Todos — `feature/teamplayer_tui` & bitbots_cargo (PR #944)

Results of the detailed review of `feature/teamplayer_tui` (40 commits) and a
re-check of build-system findings against the head of PR #944
(`feature/bitbots_cargo`, head `44ca588`).

## Must-fix before merge

- [ ] **#1 TUI restart race for multi-cmd components.** `App::drain_messages`
      acts on the first `ProcessExited` by restarting `world_model`
      (ball_filter + robot_filter); the second child's exit message then flips
      the *new* instance to Stopped/Crashed and clears its fresh pids, leaving
      new processes untracked. Needs per-process exit accounting or a pending
      child counter.
- [ ] **#2 Partial spawn failure leaks a process.** In `start_component`, if
      the second command of a component fails to spawn, the function returns
      early; the already-spawned child keeps running but its pid is lost on
      the next restart → orphan that survives the final SIGTERM sweep.
- [ ] **#8 Extrinsic calibration publishes bogus `/tf_static` at startup.**
      `src/main.rs` publishes immediately with `CalibrationParams::default()`
      (empty frame ids) before any parameter event arrives. Fix: only publish
      inside the parameter-event loop (i.e. after the first set event has
      been received). Also `get_now().unwrap()` and
      `expect("Failed to publish updated transform")` can panic the task →
      log and continue instead. Confirmed still present on PR #944 head.
      (State does not need to be `Option`: r2r's derived parameter handler
      needs `RosParams + Default`; gating publication on the event stream
      gives the same guarantee.)
- [ ] **#9 Missing/stale deps in `bitbots_extrinsic_calibration/package.xml`.**
      No `tf2_msgs` build_depend although the Rust node publishes
      `tf2_msgs::TFMessage` (IDL traversal in `r2r_cargo` relies on declared
      deps). Docs export still says `<language>c++</language>` /
      `status tested_robot` after the Rust rewrite. Verify the derived
      parameter handler tolerates `use_sim_time` passed from launch.
      Confirmed still present on PR #944 head.
- [ ] **#10 `bitbots_path_planning` declares no dependency on
      `bitbots_global_planner`** although it imports the module; the bogus
      `<depend>cargo</depend>` remains. Add
      `<exec_depend>bitbots_global_planner</exec_depend>` and drop `cargo`.
      Confirmed still present on PR #944 head.
- [ ] **#15 Stale docs.** `visualization.launch` was deleted but
      `bitbots_docs/.../tutorials/launch_files.rst` and
      `extrinsic_calibration.rst` still reference it.
- [ ] Rebase `feature/teamplayer_tui` onto current `main`.

## Should-fix

- [ ] **#3 No SIGKILL escalation / no reaping on quit.** `main.rs` sends
      SIGTERM to each process group and exits immediately; stubborn
      `ros2 launch` trees can outlive the TUI. Add timeout → SIGKILL fallback
      and a short wait/reap loop. Headless Ctrl-C has the same behavior and
      drops queued log lines.
- [ ] **#13 `cargo test` without `--profile colcon`.** All three
      `bitbots_cargo` helpers register tests as plain `cargo test`, forcing a
      full second compile in the dev profile. Add `--profile colcon` (plus
      `pyo3/extension-module` where applicable). Confirmed still present on
      PR #944 head.
- [ ] **#11 (residual) Baked-in cargo target dir.** Partially fixed on
      PR #944 via `CARGO_TARGET_DIR = $PIXI_PROJECT_ROOT/build/cargo_target`
      activation env. Remaining risk: `_cargo_get_target_dir()` runs
      `cargo metadata` at *configure* time and bakes the absolute path into
      `install()`; configuring outside pixi inodes an in-tree path that no
      longer matches the built artifact. Either export `CARGO_TARGET_DIR`
      explicitly in build commands and use it for install paths, resolve
      metadata inside `install(CODE)`, or `FATAL_ERROR` at configure when
      `CARGO_TARGET_DIR` is unset.
- [ ] **#14 Two sources of truth for the robot stack.** The TUI hardcodes
      component commands/defaults while `teamplayer.launch` still exists and
      is what the deploy scripts use via tmux; defaults already diverge
      (e.g. `monitoring`). Either migrate the deploy path to the TUI or scope
      it explicitly as an operator/dev tool.

## Nits / polish

- [ ] **#4 Zenoh state lives in two places** in the TUI
      (`params.zenoh` vs `components[zenoh].enabled`); `--enable zenoh`
      desyncs them. Harmless today, footgun later.
- [ ] **#5 TUI config arrow-key navigation**: Down at bottom of left column
      jumps to Start, skipping the right column (only `→` reaches it).
- [ ] **#6 Confirm dialog cancels on *any* key** except y/Y/Enter, not just
      N/Esc as labeled.
- [ ] **#7 No tests in the `teamplayer` crate** even though cargo tests are
      now wired into colcon.
- [ ] **#12 `bitbots_cargo.cmake.in` cleanups:** `cmake_minimum_required`
      inside the included module; consuming `CMakeLists.txt` still declares
      3.5 while `string(JSON)` needs ≥ 3.19 (late, ugly failure);
      `add_library(dummy INTERFACE)` breaks if `r2r_cargo()` is called twice
      → guard with `if(NOT TARGET dummy)`; `_cargo_get_target_dir` does not
      check the `execute_process` result.
- [ ] `pixi.toml`: `libclang >= 22` pinned alongside `clang >= 21.1,<22` +
      `libllvm21` — major-version mismatch between the toolchain and the
      libclang bindgen actually uses; probably should be `libclang 21.x`.
- [ ] TUI `Cargo.toml`: `clap = "~4.4"` tilde-pin stricter than neighbors;
      `nix 0.27` is dated.
- [ ] `ProcState::label()` padding vs `↺` spinner widths in narrow terminals.
- [ ] `bitbots_bringup/package.xml` lacks `exec_depend`s for several packages
      the TUI launches at runtime (pre-existing pattern, now baked into the
      TUI).

## Tracked temporary measures (PR #944)

- `[patch.crates-io]` pins r2r crates to the `bit-bots/r2r` fork
  (Clang-compatible bindgen). Remove once an upstream r2r release includes
  the fix (sequenceplanner/r2r#133).
