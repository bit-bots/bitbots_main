# Agent Instructions

## Development Environment

This ROS 2 workspace is managed by Pixi. Run development commands through the
repository's default Pixi environment; do not invoke `colcon`, ROS tools, or
formatters directly from the host shell.

- Build the workspace with `pixi run -e default build`.
- Build selected packages with
  `pixi run -e default build --packages-select <package...>`.
- Run tests with `pixi run -e default test`.
- Test selected packages with
  `pixi run -e default test --packages-select <package...>`.
- Run formatting and linting with `pixi run -e default format`.
- Use `pixi run -e default <command>` for other one-off commands that need the
  workspace dependencies. Prefer this over `pixi shell`, whose state can become
  stale after environment changes.
- Search for available dependencies with `pixi search <package>`.
- Clean all build artifacts with `pixi run -e default clean`, or clean one
  package with `pixi run -e default clean <package>`.
- Remove the information in the .pixi folder with `pixi clean`. This is
  useful for troubleshooting or resetting the Pixi environment, but it will
  require re-downloading dependencies and rebuilding the workspace.
- Use `pixi task list` to inspect available repository tasks.

The default Pixi environment includes the ROS 2 and formatting features and
provides the repository's pinned toolchain, dependencies, activation variables,
and workspace setup. Calling `colcon` directly can use an unrelated host
installation or an incomplete environment and may reject supported arguments or
produce results that differ from CI.

## Git Conventions

Make changes on a branch and open a pull request against `main`. Prefix branch
names with `feature/`, `fix/`, or `refactor/` when applicable.

Commit messages should be in the format `<type>: <description>`, where `<type>` is
one of `feat`, `fix`, `refactor`, `docs`, `test`, or `chore`.
The description should be concise and descriptive of the change.

## Dynamic Stack Decider

Several robot behaviors use the Dynamic Stack Decider (DSD).
Read `src/lib/dynamic_stack_decider/README.md` before changing DSD behavior or syntax.

- DSD behavior files form a tree: decisions use the `$Decision` syntax, actions
  use `@Action`, and `-->` marks the behavior entry point.
- Implement decisions and actions as Python classes in the package's
  `*_dsd` directory, following nearby   elements and their shared blackboard APIs.
- Keep decision result strings consistent with the branches in the corresponding
  `.dsd` file. Decision results conventionally use uppercase names.
- Validate DSD changes with the package's DSD parsing test, such as
  `test_dsd_file.py` or `test_dsd_valid.py`, through the selected-package Pixi
  test command.
