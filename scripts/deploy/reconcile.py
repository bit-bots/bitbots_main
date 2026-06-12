from __future__ import annotations

import asyncio
import hashlib
import inspect
import json
import re
import shlex
import subprocess
from collections.abc import Callable
from pathlib import Path

import yaml
from deploy.artifacts import ArtifactServer, EnvironmentArtifact, environment_cache_key
from deploy.models import (
    TEAMPLAYER_COMPONENTS,
    CommandFailedError,
    CommandResult,
    DesiredState,
    RobotStatus,
    RobotTarget,
    Step,
    StepReport,
    StepState,
)
from deploy.transport import OpenSSHTransport, default_runtime_dir

StatusCallback = Callable[[RobotStatus], object]


class RobotReconciler:
    def __init__(
        self,
        target: RobotTarget,
        workspace: Path,
        artifact: EnvironmentArtifact | None,
        artifact_server: ArtifactServer | None,
        retry_interval: float = 2.0,
        on_status: StatusCallback | None = None,
        transport: OpenSSHTransport | None = None,
    ) -> None:
        self.target = target
        self.workspace = workspace
        self.artifact = artifact
        self.artifact_server = artifact_server
        self.retry_interval = retry_interval
        self.on_status = on_status
        self.transport = transport or OpenSSHTransport(target, default_runtime_dir())
        self.status = RobotStatus(target=target)
        self._desired: DesiredState | None = None
        self._wake = asyncio.Event()
        self._stop = False
        self._retry_failed = False
        self._task: asyncio.Task[None] | None = None
        self._probe_task: asyncio.Task[None] | None = None
        self._log_task: asyncio.Task[None] | None = None
        self._stream_logs = False
        self._failed_generation: int | None = None
        self._applied: DesiredState | None = None

    def start(self) -> None:
        if self._task is None:
            self._task = asyncio.create_task(self._run(), name=f"deploy-{self.target.name}")
            self._probe_task = asyncio.create_task(
                self._probe_connections(),
                name=f"probe-{self.target.name}",
            )

    def apply(self, desired: DesiredState) -> None:
        self._desired = desired
        self.status.generation = desired.generation
        self._failed_generation = None
        self._wake.set()
        self._notify()

    def retry(self) -> None:
        self._retry_failed = True
        self._failed_generation = None
        self._wake.set()

    def cancel(self) -> None:
        self._desired = None
        self.transport.cancel_current()
        self._failed_generation = None
        self._retry_failed = False
        self._wake.set()
        for step, report in self.status.steps.items():
            if report.state in {StepState.PENDING, StepState.RUNNING, StepState.WAITING}:
                self.status.steps[step] = StepReport(StepState.SKIPPED, "Cancelled")
        self._notify()

    def set_log_stream(self, enabled: bool) -> None:
        self._stream_logs = enabled
        if enabled and self._log_task is None:
            self._log_task = asyncio.create_task(
                self._watch_teamplayer(),
                name=f"logs-{self.target.name}",
            )
        elif not enabled and self._log_task is not None:
            self._log_task.cancel()
            self._log_task = None

    async def close(self) -> None:
        self._stop = True
        self._wake.set()
        if self._task is not None:
            await self._task
        if self._probe_task is not None:
            await self._probe_task
        if self._log_task is not None:
            self._log_task.cancel()
            try:
                await self._log_task
            except asyncio.CancelledError:
                pass
        await self.transport.close()

    async def attach(self) -> int:
        return await self.transport.attach_tmux()

    async def refresh_status(self) -> None:
        try:
            result = await self.transport.run(
                self._teamplayer_control_command("status --json"),
                check=False,
            )
            self.status.connected = result.returncode != 255
            self._parse_teamplayer_status(result.stdout)
        except CommandFailedError:
            self.status.connected = False
        self._notify()

    async def _run(self) -> None:
        while not self._stop:
            if self._desired is None:
                await self._wait()
                continue
            desired = self._desired
            if self._failed_generation == desired.generation and not self._retry_failed:
                await self._wait()
                continue
            self._retry_failed = False
            try:
                await self._reconcile(desired)
            except CommandFailedError as exc:
                if self._desired is not desired:
                    running = self._running_step()
                    if running is not None:
                        self._set_step(running, StepState.SKIPPED, "Cancelled")
                    continue
                if not exc.transient:
                    self._failed_generation = desired.generation
                await self._record_failure(exc)
                if exc.transient:
                    await asyncio.sleep(self.retry_interval)
                    continue
                await self._wait(desired)
            except Exception as exc:
                self._set_step(
                    self._running_step() or Step.CONNECT,
                    StepState.FAILED,
                    str(exc),
                    details=repr(exc),
                )
                self._failed_generation = desired.generation
                await self._wait(desired)

    async def _probe_connections(self) -> None:
        while not self._stop:
            try:
                await self.transport.connect()
                connected = True
                result = await self.transport.run(
                    self._teamplayer_control_command("status --json"),
                    check=False,
                )
                self._parse_teamplayer_status(result.stdout)
                remote_fingerprint = await self.transport.run(
                    f"cat {shell_path(self.target.workspace)}/.deploy/source-fingerprint",
                    check=False,
                )
                self.status.applied_source_fingerprint = remote_fingerprint.stdout.strip()
            except CommandFailedError:
                connected = False
            if connected != self.status.connected:
                self.status.connected = connected
                self._notify()
            await asyncio.sleep(self.retry_interval)

    async def _watch_teamplayer(self) -> None:
        while self._stream_logs and not self._stop:
            try:
                await self.transport.stream(
                    self._teamplayer_control_command("watch --jsonl"),
                    self._teamplayer_event,
                )
            except (CommandFailedError, OSError):
                if self._stream_logs and not self._stop:
                    await asyncio.sleep(self.retry_interval)

    async def _teamplayer_event(self, line: str) -> None:
        try:
            event = json.loads(line)
        except json.JSONDecodeError:
            self.status.append_log(line)
            self._notify()
            return
        if event.get("type") == "status":
            self.status.teamplayer = event.get("status", {})
        elif event.get("type") == "log":
            component = event.get("component", "teamplayer")
            stream = event.get("stream", "stdout")
            self.status.append_log(f"[INFO] [{component}/{stream}] {event.get('line', '')}")
        elif event.get("type") == "component":
            self.status.append_log(f"[INFO] [{event.get('key', 'component')}] {event.get('state', 'unknown')}")
        self._notify()

    def _parse_teamplayer_status(self, output: str) -> None:
        if not output:
            self.status.teamplayer = {}
            return
        try:
            response = json.loads(output.splitlines()[-1])
        except json.JSONDecodeError:
            return
        if response.get("type") == "status":
            self.status.teamplayer = response.get("status", {})

    def _teamplayer_control_command(self, arguments: str) -> str:
        workspace = shell_path(self.target.workspace)
        return f"cd {workspace} && pixi run -e robot --as-is teamplayer ctl {arguments}"

    async def _wait(self, expected: DesiredState | None = None) -> None:
        self._wake.clear()
        if self._stop or (expected is not None and self._desired is not expected):
            return
        await self._wake.wait()

    async def _reconcile(self, desired: DesiredState) -> None:
        self.status.source_fingerprint = source_fingerprint(self.workspace)
        await self._step(Step.CONNECT, "Connecting", self.transport.connect)
        self.status.connected = True
        remote_fingerprint = await self.transport.run(
            f"cat {shell_path(self.target.workspace)}/.deploy/source-fingerprint",
            check=False,
        )
        self.status.applied_source_fingerprint = remote_fingerprint.stdout.strip()
        if self._superseded(desired):
            return

        source_changed = desired.sync_source and self.status.stale
        if source_changed:
            await self._sync_source()
            self.status.applied_source_fingerprint = self.status.source_fingerprint
        else:
            self._set_step(Step.SOURCE, StepState.SKIPPED, "Source is already applied")
        if self._superseded(desired):
            return

        env_changed = False
        if self.artifact is not None:
            env_changed = await self._ensure_environment()
        else:
            if desired.build or desired.launch:
                self._set_step(
                    Step.ENVIRONMENT,
                    StepState.RUNNING,
                    "Checking offline package cache",
                )
                raise CommandFailedError(
                    CommandResult(
                        ("environment-preflight",),
                        2,
                        stderr=(
                            "No matching offline environment cache. Run "
                            "`pixi run deploy cache prepare` while Internet access is available."
                        ),
                    )
                )
            self._set_step(
                Step.ENVIRONMENT,
                StepState.SKIPPED,
                "Environment provisioning is not required",
            )
        if self._superseded(desired):
            return

        previous = self._applied
        configuration_changed = previous is None or previous.match != desired.match or previous.ball != desired.ball
        if configuration_changed:
            await self._write_configuration(desired)
        else:
            self._set_step(
                Step.CONFIGURATION,
                StepState.SKIPPED,
                "Configuration is already applied",
            )
        if self._superseded(desired):
            return

        build_changed = previous is None or not previous.build or previous.build_packages != desired.build_packages
        build_ran = desired.build and (env_changed or source_changed or desired.clean_build or build_changed)
        if build_ran:
            await self._build(desired)
        elif desired.build:
            self._set_step(Step.BUILD, StepState.SKIPPED, "Build is up to date")
        else:
            self._set_step(Step.BUILD, StepState.SKIPPED, "Build disabled")
        if self._superseded(desired):
            return

        wifi_changed = previous is None or previous.match.wifi_profile != desired.match.wifi_profile
        if wifi_changed:
            await self._wifi(desired)
        else:
            self._set_step(Step.WIFI, StepState.SKIPPED, "WiFi is already applied")
        if self._superseded(desired):
            return

        if desired.launch:
            teamplayer_changed = (
                previous is None
                or not previous.launch
                or previous.components != desired.components
                or configuration_changed
                or build_ran
                or env_changed
            )
            if teamplayer_changed:
                await self._teamplayer(
                    desired,
                    restart_components=configuration_changed or build_ran or env_changed,
                )
            else:
                self._set_step(
                    Step.TEAMPLAYER,
                    StepState.SKIPPED,
                    "Teamplayer state is already applied",
                )
        else:
            if previous is None or previous.launch:
                await self._stop_teamplayer()
            else:
                self._set_step(
                    Step.TEAMPLAYER,
                    StepState.SKIPPED,
                    "Teamplayer is already stopped",
                )

        if self._desired is desired:
            self._applied = desired
            self.status.applied_generation = desired.generation
            self._notify()
            await self._wait(desired)

    def _superseded(self, desired: DesiredState) -> bool:
        return self._stop or self._desired is not desired

    async def _ensure_environment(self) -> bool:
        assert self.artifact is not None
        assert self.artifact_server is not None
        if self.artifact.key != environment_cache_key(self.workspace):
            raise CommandFailedError(
                CommandResult(
                    ("environment-preflight",),
                    2,
                    stderr=(
                        "The robot lock changed after the offline cache was selected. Run "
                        "`pixi run deploy cache prepare` with Internet access, then restart the deploy tool."
                    ),
                )
            )
        workspace = shell_path(self.target.workspace)
        key = self.artifact.key
        marker = f"{workspace}/.deploy/environment/{key}.complete"
        result = await self.transport.run(f"test -f {marker}", check=False)
        config = self.artifact_server.pixi_config()
        config_path = f"{workspace}/.deploy/pixi-offline.toml"
        install = (
            f"cd {workspace} && "
            f"pixi install -e robot --frozen --config-file {config_path} "
            "--concurrent-downloads 4 --concurrent-solves 1; "
            f"status=$?; rm -f {config_path}; exit $status"
        )

        async def provision() -> CommandResult:
            await self.transport.run(
                f"mkdir -p {workspace}/.deploy/environment; "
                f"cat > {config_path}.tmp && mv {config_path}.tmp {config_path}",
                stdin=config,
                output=self._log,
            )
            install_result = await self.transport.run(install, output=self._log)
            await self.transport.run(
                f"touch {marker}.tmp && mv {marker}.tmp {marker}",
                output=self._log,
            )
            return install_result

        try:
            await self._step(
                Step.ENVIRONMENT,
                (
                    f"Verifying normal Pixi robot environment {key[:12]}"
                    if result.ok
                    else f"Installing normal Pixi robot environment {key[:12]}"
                ),
                provision,
            )
        except CommandFailedError as exc:
            if _environment_failure_is_transient(exc.result):
                exc.transient = True
            raise
        return not result.ok

    async def _sync_source(self) -> None:
        await self._step(
            Step.SOURCE,
            "Synchronizing source",
            lambda: self.transport.rsync(
                self.workspace,
                self.target.workspace,
                self.workspace / ".rsyncignore",
                output=self._log,
            ),
        )
        workspace = shell_path(self.target.workspace)
        await self.transport.run(
            f"mkdir -p {workspace}/.deploy; "
            f"printf '%s\\n' {shlex.quote(self.status.source_fingerprint)} > "
            f"{workspace}/.deploy/source-fingerprint"
        )

    async def _write_configuration(self, desired: DesiredState) -> None:
        settings = dict(desired.match.game_settings)
        settings["bot_id"] = robot_id(self.target)
        settings["ball"] = {"diameter": desired.ball.diameter}
        document = {"parameter_blackboard": {"ros__parameters": settings}}
        content = yaml.safe_dump(document, sort_keys=False)
        workspace = shell_path(self.target.workspace)
        destination = f"{workspace}/.deploy/config/game_settings.yaml"
        command = f"mkdir -p {workspace}/.deploy/config; cat > {destination}.tmp && mv {destination}.tmp {destination}"
        await self._step(
            Step.CONFIGURATION,
            f"Applying {desired.match.name} / {desired.ball.name}",
            lambda: self.transport.run(command, stdin=content, output=self._log),
        )

    async def _build(self, desired: DesiredState) -> None:
        workspace = shell_path(self.target.workspace)
        clean = ""
        if desired.clean_build:
            if desired.build_packages:
                paths = [
                    f"{directory}/{shlex.quote(package)}"
                    for package in desired.build_packages
                    for directory in ("build", "install")
                ]
                clean = f"rm -rf {' '.join(paths)}; "
            else:
                clean = "rm -rf build install log; "
        packages = ""
        if desired.build_packages:
            packages = " --packages-select " + " ".join(shlex.quote(package) for package in desired.build_packages)
        command = f"cd {workspace}; {clean}pixi run -e robot --as-is build{packages}"
        await self._step(
            Step.BUILD,
            "Building workspace",
            lambda: self.transport.run(command, output=self._log),
        )

    async def _wifi(self, desired: DesiredState) -> None:
        if not desired.match.wifi_profile:
            self._set_step(
                Step.WIFI,
                StepState.SKIPPED,
                "No WiFi profile configured; leaving WiFi unchanged",
            )
            return
        profile = shlex.quote(desired.match.wifi_profile)
        await self._step(
            Step.WIFI,
            f"Activating {desired.match.wifi_profile}",
            lambda: self.transport.run(
                f"sudo -n nmcli connection up {profile}",
                output=self._log,
            ),
        )

    async def _teamplayer(self, desired: DesiredState, *, restart_components: bool) -> None:
        workspace = shell_path(self.target.workspace)
        component_args = " ".join(f"--enable {shlex.quote(component)}" for component in sorted(desired.components))
        disabled_args = " ".join(
            f"--disable {shlex.quote(component)}"
            for component in TEAMPLAYER_COMPONENTS
            if component not in desired.components
        )
        settings = f"{workspace}/.deploy/config/game_settings.yaml"
        launch = (
            f"cd {workspace} && exec pixi run -e robot --as-is teamplayer "
            f"--headless --fieldname {shlex.quote(desired.match.field)} "
            f"--game-settings {settings} {component_args} {disabled_args}"
        )
        ctl = "pixi run -e robot --as-is teamplayer ctl"
        update_config = f"{ctl} set-config --fieldname {shlex.quote(desired.match.field)} --game-settings {settings}"
        apply_components = f"{ctl} set-components {component_args}"
        control_action = f"{ctl} {'restart' if restart_components else 'start'} all"
        command = (
            "if tmux has-session -t teamplayer 2>/dev/null; then "
            f"cd {workspace}; {update_config} && {apply_components} && {control_action}; "
            "else "
            f"tmux new-session -d -s teamplayer {shlex.quote(launch)}; "
            "fi"
        )
        await self._step(
            Step.TEAMPLAYER,
            "Adopting or starting teamplayer",
            lambda: self.transport.run(command, output=self._log),
        )

    async def _stop_teamplayer(self) -> None:
        command = (
            f"if tmux has-session -t teamplayer 2>/dev/null; then {self._teamplayer_control_command('stop all')}; fi"
        )
        await self._step(
            Step.TEAMPLAYER,
            "Stopping teamplayer components",
            lambda: self.transport.run(command, output=self._log),
        )

    async def _step(self, step: Step, summary: str, operation: Callable[[], object]) -> object:
        self._set_step(step, StepState.RUNNING, summary)
        try:
            result = operation()
            if inspect.isawaitable(result):
                result = await result
        except Exception:
            raise
        self._set_step(step, StepState.SUCCEEDED, summary)
        return result

    async def _record_failure(self, exc: CommandFailedError) -> None:
        step = self._running_step() or Step.CONNECT
        self.status.connected = not exc.transient
        self._set_step(
            step,
            StepState.WAITING if exc.transient else StepState.FAILED,
            str(exc),
            details="\n".join(part for part in (exc.result.stdout, exc.result.stderr) if part),
        )

    async def _log(self, line: str) -> None:
        self.status.append_log(line)
        self._notify()

    def _running_step(self) -> Step | None:
        return next(
            (step for step, report in self.status.steps.items() if report.state == StepState.RUNNING),
            None,
        )

    def _set_step(
        self,
        step: Step,
        state: StepState,
        summary: str,
        *,
        details: str = "",
    ) -> None:
        self.status.steps[step] = StepReport(state, summary, details)
        self._notify()

    def _notify(self) -> None:
        if self.on_status is not None:
            self.on_status(self.status)


class DeploymentSupervisor:
    def __init__(
        self,
        targets: list[RobotTarget],
        workspace: Path,
        artifact: EnvironmentArtifact | None,
        artifact_server: ArtifactServer | None,
        retry_interval: float = 2.0,
        on_status: StatusCallback | None = None,
    ) -> None:
        self.workspace = workspace
        self.artifact = artifact
        self.artifact_server = artifact_server
        self.retry_interval = retry_interval
        self.on_status = on_status
        self._source_task: asyncio.Task[None] | None = None
        self.controllers = {
            target.name: RobotReconciler(
                target,
                workspace,
                artifact,
                artifact_server,
                retry_interval,
                on_status,
            )
            for target in targets
        }

    def start(self) -> None:
        for controller in self.controllers.values():
            controller.start()
        if self._source_task is None:
            self._source_task = asyncio.create_task(
                self._monitor_source(),
                name="deploy-source-monitor",
            )

    def apply(self, names: list[str], desired: DesiredState) -> None:
        for name in names:
            self.controllers[name].apply(desired)

    def add_target(self, target: RobotTarget) -> RobotReconciler:
        existing = self.controllers.get(target.name)
        if existing is not None:
            return existing
        controller = RobotReconciler(
            target,
            self.workspace,
            self.artifact,
            self.artifact_server,
            self.retry_interval,
            self.on_status,
        )
        self.controllers[target.name] = controller
        controller.start()
        return controller

    async def close(self) -> None:
        if self._source_task is not None:
            self._source_task.cancel()
            try:
                await self._source_task
            except asyncio.CancelledError:
                pass
        await asyncio.gather(*(controller.close() for controller in self.controllers.values()))

    async def _monitor_source(self) -> None:
        while True:
            fingerprint = await asyncio.to_thread(source_fingerprint, self.workspace)
            for controller in self.controllers.values():
                if controller.status.source_fingerprint != fingerprint:
                    controller.status.source_fingerprint = fingerprint
                    controller._notify()
            await asyncio.sleep(2)


def source_fingerprint(workspace: Path) -> str:
    head = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=workspace,
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    diff = subprocess.run(
        ["git", "diff", "--binary", "HEAD"],
        cwd=workspace,
        check=True,
        capture_output=True,
    ).stdout
    untracked = subprocess.run(
        ["git", "ls-files", "--others", "--exclude-standard", "-z"],
        cwd=workspace,
        check=True,
        capture_output=True,
    ).stdout.split(b"\0")
    digest = hashlib.sha256(head.encode() + diff)
    for relative in sorted(path for path in untracked if path):
        digest.update(relative)
        digest.update((workspace / relative.decode()).read_bytes())
    return digest.hexdigest()


def robot_id(target: RobotTarget) -> int:
    match = re.search(r"(\d+)$", target.name)
    return int(match.group(1)) if match else 1


def shell_path(path: str) -> str:
    if path.startswith("~/"):
        return '"$HOME"/' + shlex.quote(path[2:])
    return shlex.quote(path)


def _environment_failure_is_transient(result: CommandResult) -> bool:
    if result.returncode == 255:
        return True
    output = f"{result.stdout}\n{result.stderr}".lower()
    transient_markers = (
        "connection refused",
        "connection reset",
        "connection timed out",
        "could not connect",
        "download failed",
        "error sending request",
        "failed to download",
        "network is unreachable",
        "operation timed out",
        "temporary failure",
    )
    return any(marker in output for marker in transient_markers)
