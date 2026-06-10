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
- Use `pixi run -e default <command>` for other one-off commands that need the workspace
  dependencies, or enter the environment with `pixi shell`.
- Use `pixi task list` to inspect available repository tasks.

The default Pixi environment includes the ROS 2 and formatting features and
provides the repository's pinned toolchain, dependencies, activation variables,
and workspace setup. Calling `colcon` directly can use an unrelated host
installation or an incomplete environment and may reject supported arguments or
produce results that differ from CI.
