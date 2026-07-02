import concurrent.futures
from collections.abc import Callable

from fabric import Group, GroupResult, Result

from deploy.misc import Connection, hide_output, print_debug, print_info

_INTERNET_STATUS_ATTR = "_bitbots_deploy_internet_available"
_PIXI_REPOSITORY_URL = "https://data.bit-bots.de/conda-misc/output/"
_PIXI_REPOSITORY_CHECK_COMMAND = f"curl -sSfLI --max-time 5 {_PIXI_REPOSITORY_URL}"


def robot_has_internet(connection: Connection) -> bool:
    cached_status: bool | None = getattr(connection, _INTERNET_STATUS_ATTR, None)
    if cached_status is not None:
        return cached_status

    print_debug(
        f"Checking for internet connection from {connection.original_host} to Pixi repository: {_PIXI_REPOSITORY_URL}"
    )

    cmd = _PIXI_REPOSITORY_CHECK_COMMAND
    print_debug(f"Calling '{cmd}' on: {connection.host}")
    result = connection.run(cmd, hide=hide_output(), warn=True)
    available = result.ok

    setattr(connection, _INTERNET_STATUS_ATTR, available)
    print_debug(
        f"Internet connection to Pixi repository on {connection.original_host} "
        f"is{' ' if available else ' NOT '}available."
    )
    if not available:
        print_info(
            f"Internet connection is not available on {connection.original_host}. "
            "Remote pixi run commands will use --as-is."
        )
    return available


def clear_robot_internet_status(connection: Connection) -> None:
    if hasattr(connection, _INTERNET_STATUS_ATTR):
        delattr(connection, _INTERNET_STATUS_ATTR)


def pixi_run_command(connection: Connection, remote_workspace: str, pixi_command: str) -> str:
    as_is_argument = "" if robot_has_internet(connection) else " --as-is"
    return f"cd {remote_workspace} && pixi run{as_is_argument} --environment robot {pixi_command}"


def pixi_install_if_online(connection: Connection, remote_workspace: str) -> Result:
    if not robot_has_internet(connection):
        command = "# Skipping pixi install because the robot has no internet connection."
        stdout = "Skipped pixi install because the robot has no internet connection."
        print_debug(stdout)
        return Result(connection=connection, command=command, stdout=stdout, exited=0)

    cmd = f"cd {remote_workspace} && pixi install --environment robot"
    print_debug("Installing dependencies on remote host to verify synchronization and architecture match.")
    print_debug(f"Calling '{cmd}' on: {connection.host}")
    return connection.run(cmd, hide=hide_output(), warn=True)


def run_for_connections(connections: Group, command_factory: Callable[[Connection], str]) -> GroupResult:
    def _run_single(connection: Connection) -> Result:
        cmd = command_factory(connection)
        print_debug(f"Calling '{cmd}' on: {connection.host}")
        return connection.run(cmd, hide=hide_output(), warn=True)

    results = GroupResult()
    with concurrent.futures.ThreadPoolExecutor(max_workers=len(connections)) as executor:
        futures = [executor.submit(_run_single, connection) for connection in connections]

    for future in futures:
        result = future.result()
        results[result.connection] = result
    return results
