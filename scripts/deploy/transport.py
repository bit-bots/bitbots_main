from __future__ import annotations

import asyncio
import hashlib
import os
import shlex
from collections.abc import Awaitable, Callable
from pathlib import Path

from deploy.models import CommandFailedError, CommandResult, RobotTarget

OutputCallback = Callable[[str], Awaitable[None] | None]


class OpenSSHTransport:
    def __init__(
        self,
        target: RobotTarget,
        runtime_dir: Path,
        connect_timeout: int = 5,
    ) -> None:
        self.target = target
        self.connect_timeout = connect_timeout
        digest = hashlib.sha256(target.destination.encode()).hexdigest()[:12]
        self.control_path = runtime_dir / f"ssh-{digest}.sock"
        self._master: asyncio.subprocess.Process | None = None
        self._lock = asyncio.Lock()
        self._active: set[asyncio.subprocess.Process] = set()

    async def connect(self) -> None:
        async with self._lock:
            if await self.check():
                return
            await self.close()
            self.control_path.parent.mkdir(parents=True, exist_ok=True)
            self._master = await asyncio.create_subprocess_exec(
                "ssh",
                "-MN",
                "-o",
                "BatchMode=yes",
                "-o",
                f"ConnectTimeout={self.connect_timeout}",
                "-o",
                "ControlMaster=yes",
                "-o",
                "ControlPersist=no",
                "-o",
                f"ControlPath={self.control_path}",
                self.target.destination,
                stdout=asyncio.subprocess.DEVNULL,
                stderr=asyncio.subprocess.PIPE,
            )
            for _ in range(max(1, self.connect_timeout * 10)):
                if await self.check():
                    return
                if self._master.returncode is not None:
                    stderr = await self._master.stderr.read() if self._master.stderr else b""
                    result = CommandResult(
                        ("ssh", self.target.destination),
                        self._master.returncode,
                        stderr=stderr.decode(errors="replace"),
                    )
                    raise CommandFailedError(result, transient=True)
                await asyncio.sleep(0.1)
            await self.close()
            raise CommandFailedError(
                CommandResult(
                    ("ssh", self.target.destination),
                    255,
                    stderr="SSH connection timed out",
                ),
                transient=True,
            )

    async def check(self) -> bool:
        if not self.control_path.exists():
            return False
        proc = await asyncio.create_subprocess_exec(
            "ssh",
            "-S",
            str(self.control_path),
            "-O",
            "check",
            self.target.destination,
            stdout=asyncio.subprocess.DEVNULL,
            stderr=asyncio.subprocess.DEVNULL,
        )
        return await proc.wait() == 0

    async def run(
        self,
        command: str,
        *,
        stdin: str | None = None,
        output: OutputCallback | None = None,
        check: bool = True,
    ) -> CommandResult:
        await self.connect()
        args = (
            "ssh",
            "-S",
            str(self.control_path),
            self.target.destination,
            command,
        )
        result = await _run_process(args, stdin=stdin, output=output, active=self._active)
        if check and not result.ok:
            raise CommandFailedError(result, transient=result.returncode == 255)
        return result

    async def rsync(
        self,
        source: Path,
        destination: str,
        exclude_file: Path,
        output: OutputCallback | None = None,
    ) -> CommandResult:
        await self.connect()
        ssh_command = f"ssh -S {shlex.quote(str(self.control_path))}"
        args = (
            "rsync",
            "--archive",
            "--delete",
            "--checksum",
            "--info=progress2",
            f"--exclude-from={exclude_file}",
            "-e",
            ssh_command,
            f"{source}/",
            f"{self.target.destination}:{destination}/",
        )
        result = await _run_process(args, output=output, active=self._active)
        if not result.ok:
            raise CommandFailedError(
                result,
                transient=result.returncode in {10, 12, 30, 35, 255},
            )
        return result

    async def attach_tmux(self, session: str = "teamplayer") -> int:
        await self.connect()
        proc = await asyncio.create_subprocess_exec(
            "ssh",
            "-S",
            str(self.control_path),
            "-t",
            self.target.destination,
            f"tmux attach-session -t {shlex.quote(session)}",
        )
        self._active.add(proc)
        try:
            return await proc.wait()
        finally:
            self._active.discard(proc)

    async def stream(self, command: str, output: OutputCallback) -> None:
        await self.connect()
        args = (
            "ssh",
            "-S",
            str(self.control_path),
            self.target.destination,
            command,
        )
        proc = await asyncio.create_subprocess_exec(
            *args,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
        assert proc.stdout is not None
        assert proc.stderr is not None

        async def pump(stream: asyncio.StreamReader) -> None:
            while line := await stream.readline():
                text = line.decode(errors="replace").rstrip()
                maybe_awaitable = output(text)
                if maybe_awaitable is not None:
                    await maybe_awaitable

        try:
            await asyncio.gather(pump(proc.stdout), pump(proc.stderr))
        except asyncio.CancelledError:
            proc.terminate()
            await proc.wait()
            raise
        returncode = await proc.wait()
        if returncode:
            raise CommandFailedError(
                CommandResult(args, returncode, stderr="teamplayer event stream ended"),
                transient=returncode == 255,
            )

    def cancel_current(self) -> None:
        for process in tuple(self._active):
            if process.returncode is None:
                process.terminate()

    async def close(self) -> None:
        master, self._master = self._master, None
        if master is not None and master.returncode is None:
            master.terminate()
            try:
                await asyncio.wait_for(master.wait(), timeout=2)
            except asyncio.TimeoutError:
                master.kill()
                await master.wait()
        self.control_path.unlink(missing_ok=True)


async def _run_process(
    args: tuple[str, ...],
    *,
    stdin: str | None = None,
    output: OutputCallback | None = None,
    active: set[asyncio.subprocess.Process] | None = None,
) -> CommandResult:
    proc = await asyncio.create_subprocess_exec(
        *args,
        stdin=asyncio.subprocess.PIPE if stdin is not None else None,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
    )
    if active is not None:
        active.add(proc)
    stdout_lines: list[str] = []
    stderr_lines: list[str] = []

    async def pump(stream: asyncio.StreamReader, target: list[str]) -> None:
        while line := await stream.readline():
            text = line.decode(errors="replace").rstrip()
            target.append(text)
            if output is not None:
                maybe_awaitable = output(text)
                if maybe_awaitable is not None:
                    await maybe_awaitable

    if stdin is not None and proc.stdin is not None:
        proc.stdin.write(stdin.encode())
        await proc.stdin.drain()
        proc.stdin.close()
    assert proc.stdout is not None
    assert proc.stderr is not None
    try:
        await asyncio.gather(pump(proc.stdout, stdout_lines), pump(proc.stderr, stderr_lines))
        return CommandResult(
            command=args,
            returncode=await proc.wait(),
            stdout="\n".join(stdout_lines),
            stderr="\n".join(stderr_lines),
        )
    finally:
        if active is not None:
            active.discard(proc)


def default_runtime_dir() -> Path:
    base = Path(os.environ.get("XDG_RUNTIME_DIR", f"/tmp/bitbots-deploy-{os.getuid()}"))
    return base / "bitbots-deploy"
