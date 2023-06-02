# TODO: also install pip upgrades


def _internet_available_on_target(target: Target) -> bool:
    """
    Check if the Target has an internet connection by pinging apt repos.
    
    :param target: The Target to check for an internet connection
    :return: Whether the Target has an internet connection
    """
    print_info(f"Checking internet connection on {target.hostname}")

    apt_mirror = "de.archive.ubuntu.com"
    return _execute_on_target(target, f"timeout --foreground 0.5 curl -sSLI {apt_mirror}").ok



def install_rosdeps(target: Target) -> None:
    """
    Install ROS dependencies using the rosdep tool on the given Target.

    :param target: The Target to install the ROS dependencies on
    """
    if _internet_available_on_target(target):
        print_info(f"Installing rosdeps on {target.hostname}")
        target_src_path = os.path.join(target.workspace, "src")

        cmd = f"rosdep install -y --ignore-src --from-paths {target_src_path}"

        rosdep_result = _execute_on_target(target, cmd)
        if not rosdep_result.ok:
            print_warn(f"Rosdep install on {target.hostname} exited with code {rosdep_result.exited}. Check its output for more info")
    else: 
        print_info(f"Skipping rosdep install on {target.hostname} as we do not have internet")

