from __future__ import annotations

import hashlib
import http.server
import json
import os
import secrets
import shutil
import socket
import subprocess
import threading
from dataclasses import dataclass
from pathlib import Path

import yaml

ACTIVATION_VERSION = "1"


@dataclass(frozen=True)
class EnvironmentArtifact:
    key: str
    path: Path
    sha256: str


class ArtifactStore:
    def __init__(self, workspace: Path, cache_dir: Path | None = None) -> None:
        self.workspace = workspace
        self.cache_dir = cache_dir or (
            Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "bitbots-deploy" / "environments"
        )

    def key(self) -> str:
        digest = hashlib.sha256()
        lock = yaml.safe_load((self.workspace / "pixi.lock").read_text())
        digest.update(
            yaml.safe_dump(
                lock["environments"]["robot"],
                sort_keys=True,
            ).encode()
        )
        digest.update((self.workspace / "scripts/deploy/robot_env.sh").read_bytes())
        digest.update(b"linux-aarch64")
        digest.update(ACTIVATION_VERSION.encode())
        return digest.hexdigest()

    def get(self) -> EnvironmentArtifact | None:
        key = self.key()
        metadata_path = self.cache_dir / f"{key}.json"
        if not metadata_path.exists():
            return None
        metadata = json.loads(metadata_path.read_text())
        path = self.cache_dir / metadata["file"]
        if not path.exists() or _sha256(path) != metadata["sha256"]:
            return None
        return EnvironmentArtifact(key=key, path=path, sha256=metadata["sha256"])

    def prepare(self) -> EnvironmentArtifact:
        key = self.key()
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        output = self.cache_dir / f"robot-{key}.sh"
        temporary = output.with_suffix(".partial")
        temporary.unlink(missing_ok=True)
        subprocess.run(
            [
                "pixi-pack",
                "--environment",
                "robot",
                "--platform",
                "linux-aarch64",
                "--create-executable",
                "--output-file",
                str(temporary),
            ],
            cwd=self.workspace,
            check=True,
        )
        temporary.chmod(0o755)
        temporary.replace(output)
        artifact = EnvironmentArtifact(key=key, path=output, sha256=_sha256(output))
        (self.cache_dir / f"{key}.json").write_text(
            json.dumps(
                {"file": output.name, "sha256": artifact.sha256},
                indent=2,
            )
            + "\n"
        )
        return artifact


class ArtifactServer:
    def __init__(self, artifact: EnvironmentArtifact, bind: str = "0.0.0.0") -> None:
        self.artifact = artifact
        self.bind = bind
        self.token = secrets.token_urlsafe(24)
        self._server: http.server.ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        artifact = self.artifact
        token = self.token

        class Handler(http.server.BaseHTTPRequestHandler):
            def do_GET(self) -> None:  # noqa: N802
                if self.path != f"/{token}/{artifact.path.name}":
                    self.send_error(http.HTTPStatus.NOT_FOUND)
                    return
                size = artifact.path.stat().st_size
                start = 0
                range_header = self.headers.get("Range")
                if range_header and range_header.startswith("bytes="):
                    start = int(range_header.removeprefix("bytes=").split("-", 1)[0])
                if start < 0 or start >= size:
                    self.send_error(http.HTTPStatus.REQUESTED_RANGE_NOT_SATISFIABLE)
                    return
                self.send_response(http.HTTPStatus.PARTIAL_CONTENT if start else http.HTTPStatus.OK)
                self.send_header("Content-Length", str(size - start))
                self.send_header("Accept-Ranges", "bytes")
                if start:
                    self.send_header("Content-Range", f"bytes {start}-{size - 1}/{size}")
                self.end_headers()
                with artifact.path.open("rb") as source:
                    source.seek(start)
                    shutil.copyfileobj(source, self.wfile)

            def log_message(self, format: str, *args: object) -> None:
                return

        self._server = http.server.ThreadingHTTPServer((self.bind, 0), Handler)
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()

    @property
    def port(self) -> int:
        if self._server is None:
            raise RuntimeError("Artifact server is not running")
        return self._server.server_port

    def url(self, address: str | None = None) -> str:
        address = address or self.bind
        return f"http://{address}:{self.port}/{self.token}/{self.artifact.path.name}"

    def stop(self) -> None:
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None
        if self._thread is not None:
            self._thread.join(timeout=2)
            self._thread = None


def source_address_for(host: str) -> str:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect((host, 9))
        return str(sock.getsockname()[0])
    finally:
        sock.close()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        while chunk := file.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()
