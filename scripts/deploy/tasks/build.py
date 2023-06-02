
def build(target: Target, package: str = '', pre_clean: bool = False) -> None:
    """
    Build the source code using colcon on the given Target.
    If no package is given, all packages are built.

    :param target: The Target to build the package on
    :param package: The package to build, if empty all packages are built
    :param pre_clean: Whether to clean the build directory before building
    """
    if package and pre_clean:
        cmd_clean = f"colcon clean packages -y --packages-select {package}"
    elif pre_clean:
        cmd_clean = 'rm -rf build install log;'
    else:
        cmd_clean = ' '

    package_option = f"--packages-up-to {package}" if package else ''
    cmd = (
        "sync;"
        f"cd {target.workspace};"
        "source /opt/ros/rolling/setup.zsh;"
        "source install/setup.zsh;"
        f"{cmd_clean};"
        "ISOLATED_CPUS=\"$(grep -oP 'isolcpus=\K([\d,]+)' /proc/cmdline)\";"
        f"chrt -r 1 taskset -c ${{ISOLATED_CPUS:-0-15}} colcon build --symlink-install {package_option} --continue-on-error || exit 1;"
        "sync;"
    )

    build_result = _execute_on_target(target, cmd)
    if not build_result.ok:
        print_err(f"Build on {target.hostname} failed")
        sys.exit(build_result.exited)
