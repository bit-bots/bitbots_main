# Agent Instructions

## Workspace Overview

This repository contains the Hamburg Bit-Bots software stack for RoboCup
humanoid soccer robots. It is a ROS 2 workspace with packages for behavior,
motion, navigation, perception, world modeling, team communication, simulation,
robot support, shared messages, and operational tooling. Packages are grouped by
capability under `src/`; reusable or externally maintained projects are imported
under `src/lib`.

The stack is implemented primarily in C++ and Python, with selected Rust
components. ROS 2 packages use `ament_cmake` or Python packaging and are built and
tested with `colcon` through Pixi tasks. Pixi and RoboStack provide the
reproducible ROS and development environment. Common foundational technologies
include:

- ROS 2 client libraries, launch, parameters, actions, services, topics, TF, and
  plugin infrastructure.
- CMake, Eigen, OpenCV, and pybind11 for native robotics and vision code.
- Python with pytest, mypy, Ruff, and ROS Python tooling.
- The Dynamic Stack Decider for behavior and state-machine-like control.
- MuJoCo for simulation and ONNX Runtime for learned models where applicable.
- pre-commit, clang-format, and cppcheck for repository-wide quality checks
  (available through Pixi).

Do not assume every package uses every technology. Inspect the affected
package's manifests and nearby code before choosing tools or patterns.

## Working Principles

- Read the affected package and its surrounding code before editing.
  Prefer established package patterns, helper APIs, and naming conventions.
- Keep changes scoped to the requested behavior.
  Do not perform unrelated refactors or reformat unrelated files.
- Preserve existing user changes in a dirty worktree.
  Never discard or overwrite changes that are unrelated to the task.
- Inspect `package.xml`, `CMakeLists.txt`, `setup.py`, and `setup.cfg` as
  applicable before changing a package's build, dependencies, or entry points.
- Update documentation, configuration examples, and tests when changing public
  behavior, parameters, interfaces, or developer workflows.
- When working on code comments or documentation, do not write down concrete values,
  as they might change later, thus making the documentation outdated.
  Instead, describe the expected behavior or refer to the relevant code sections.

## Development Environment

This ROS 2 workspace is managed by Pixi. Run development commands through the
repository's Pixi environments; do not invoke `colcon`, ROS 2 tools, or formatters
directly from the host shell.

- Use the `default` environment for normal development.
  It contains the `ros` and `format` features.
- Use the `format` environment only for formatting-only work.
- Use the `robot` environment only when robot-specific dependencies are needed.
- Prefer `pixi run -e <environment> <command>` over `pixi shell`.
  A persistent shell can become stale after environment changes.
- Use `pixi task list` to inspect available repository tasks.

Common commands:

- Build the workspace with `pixi run -e default build`.
  Use the argument `--parallel-workers 2` for resource constrained environments
  (< 8 CPU cores, < 8 GB unused RAM), but prefer the default parallelism on CI
  and powerful developer machines.
- Build selected packages with
  `pixi run -e default build --packages-select <package...>`.
- Run all tests with `pixi run -e default test`.
- Test selected packages with
  `pixi run -e default test --packages-select <package...>`.
- Run formatting and linting with `pixi run -e default format`.
  Review the resulting diff because this task may modify files.
- Run one-off tools with `pixi run -e default <command>`.
- Clean all workspace build artifacts with `pixi run -e default clean`.
- Clean one package with `pixi run -e default clean <package>`.
- Use `pixi clean` only to reset Pixi's local environment data.
  This requires downloading dependencies and rebuilding afterward.

The Pixi environments provide the pinned compiler, ROS 2 installation,
dependencies, activation variables, and workspace setup used by CI. Direct host
commands may use incompatible installations or incomplete environment state.

## Dependencies

- Search the configured channels first with `pixi search <package>`.
- Determine to which section a dependency belongs to in `pixi.toml`.
- Before adding or moving a dependency, propose the suitable feature and
  environment to the user, explain why, and ask for confirmation. Do not edit
  dependency declarations or regenerate the lockfile until the user confirms.
- Prefer Conda dependencies over PyPI dependencies when a suitable package is
  available on the configured channels.
- Keep version constraints consistent with neighboring entries and explain any
  new pin or upper bound in a comment when it is not self-evident.

## ROS Packages

- Follow the package's existing Python or CMake structure rather than creating a
  new layout.
- Keep `package.xml`, build-system declarations, exports, and runtime imports in
  sync when adding or removing dependencies.
- When changing a message, service, or action definition, identify and rebuild
  the interface package and affected consumers. Update mocks, tests, and
  documentation that depend on the interface.
- For packages using `generate_parameter_library`, edit the source parameter
  definition rather than generated headers or installed output.
- Reuse existing launch patterns and substitutions. Keep launch argument,
  parameter, topic, and namespace names consistent across launch files, config
  files, and node declarations.

## Configuration

- Treat template and default configuration files as the canonical examples.
  Update them together when they describe the same parameter set.
- Preserve robot-specific configuration overrides unless the requested change
  explicitly applies to those robots.
- Do not silently change calibration, joint limits, hardware addresses, network
  settings, or safety thresholds.
- Validate renamed or added parameters against their declarations and all launch
  files that load them.

## Generated Files

- Do not manually edit files marked as generated or files produced in `build/`,
  `install/`, or `log/`.
- Locate and edit the source schema, parameter definition, model, or generator,
  then regenerate output through the repository's normal tooling.
- Treat lockfiles as generated artifacts, but commit their updates when an
  approved dependency change requires them.
- Before editing large vendored, minified, protocol-generated, or model files,
  verify that they are intended source files and not generator output.

## Testing and Validation

- Start with the narrowest relevant package build and test commands. Broaden
  validation when changing shared libraries, interfaces, launch behavior, or
  cross-package contracts.
- Add or update focused tests for bug fixes and behavioral changes.
- Run formatting after code changes and inspect all formatter modifications.
- Report commands that could not be run and the concrete reason.
- Identify tests that require a robot, simulator, GPU, camera, audio device,
  network access, or other unavailable hardware. Do not claim these tests passed
  based only on unit-test results.

## Hardware and Deployment Safety

- Do not deploy, start robot processes, enable motors, command motion, play
  animations, or change hardware state without explicit user approval.
- Do not modify calibration or robot-specific hardware configuration without
  explicit confirmation of the target robot and intended values.
- Prefer simulation or offline validation when it covers the requested behavior.
- Call out commands that can move hardware or affect devices before running
  them, even when the command is wrapped in Pixi.

## Repository Boundaries

- Directories under `src/lib` with a `.gitrepo` file are imported using
  git-subrepo. Treat them as separately maintained upstream projects.
- Avoid modifying imported libraries for a repository-local workaround unless
  the task explicitly requires an upstream library change.
- When an imported library must change, keep the change focused, run that
  library's own tests where available, and note that the corresponding upstream
  repository may need the same change.
- Do not commit build output, logs, caches, downloaded models, or local IDE state
  unless the repository intentionally tracks that artifact.

## Git Conventions

- Make changes on a branch and open a pull request against `main`.
- Prefix branch names with `feature/`, `fix/`, or `refactor/` when applicable.
- Write commit subjects as `<type>: <description>`, using a concise type such as
  `feat`, `fix`, `refactor`, `docs`, `test`, or `chore`.
- Explain why a non-obvious change is needed in the commit body or pull request
  description.
- Keep commits focused and do not include unrelated formatter or generated-file
  churn.

## Dynamic Stack Decider

Several robot behaviors use the Dynamic Stack Decider (DSD). Read
`src/lib/dynamic_stack_decider/README.md` before changing DSD behavior or syntax.

- DSD behavior files form a tree: decisions use `$Decision`, actions use
  `@Action`, and `-->` marks the behavior entry point.
- Implement decisions and actions as Python classes in the package's `*_dsd`
  directory, following nearby elements and their shared blackboard APIs.
- Keep decision result strings consistent with branches in the corresponding
  `.dsd` file. Decision results conventionally use uppercase names.
- Validate DSD changes with the package's parsing test, such as
  `test_dsd_file.py` or `test_dsd_valid.py`, through the selected-package Pixi
  test command.
