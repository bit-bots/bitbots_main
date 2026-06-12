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
from urllib.parse import unquote, urlparse

import yaml

CACHE_FORMAT_VERSION = "2"


@dataclass(frozen=True)
class EnvironmentArtifact:
    key: str
    path: Path
    sha256: str
    channels: tuple[str, ...] = ()
    pypi_bases: tuple[str, ...] = ()


class ArtifactStore:
    def __init__(self, workspace: Path, cache_dir: Path | None = None) -> None:
        self.workspace = workspace
        self.cache_dir = cache_dir or (
            Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "bitbots-deploy" / "environments"
        )

    def _lock_data(self) -> dict:
        return yaml.safe_load((self.workspace / "pixi.lock").read_text())

    def key(self) -> str:
        return environment_cache_key(self.workspace)

    def get(self) -> EnvironmentArtifact | None:
        key = self.key()
        metadata_path = self.cache_dir / f"{key}.json"
        if not metadata_path.exists():
            return None
        metadata = json.loads(metadata_path.read_text())
        path = self.cache_dir / metadata["directory"]
        if not path.is_dir() or not (path / "channel/linux-aarch64/repodata.json").is_file():
            return None
        return EnvironmentArtifact(
            key=key,
            path=path,
            sha256=metadata["sha256"],
            channels=tuple(metadata["channels"]),
            pypi_bases=tuple(metadata["pypi_bases"]),
        )

    def prepare(self) -> EnvironmentArtifact:
        existing = self.get()
        if existing is not None:
            return existing

        key = self.key()
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        output = self.cache_dir / f"robot-{key}"
        temporary = self.cache_dir / f"robot-{key}.partial"
        shutil.rmtree(temporary, ignore_errors=True)
        subprocess.run(
            [
                "pixi-pack",
                "--environment",
                "robot",
                "--platform",
                "linux-aarch64",
                "--directory-only",
                "--output-file",
                str(temporary),
                "--use-cache",
                str(self.cache_dir / "packages"),
                "--config",
                str(self.workspace / "scripts/deploy/pixi-pack-config.toml"),
            ],
            cwd=self.workspace,
            check=True,
        )
        if not (temporary / "channel/linux-aarch64/repodata.json").is_file():
            raise RuntimeError("pixi-pack did not create the linux-aarch64 channel")

        lock = self._lock_data()
        environment = lock["environments"]["robot"]
        channels = tuple(channel["url"].rstrip("/") for channel in environment["channels"])
        pypi_bases = _pypi_mirror_bases(environment["packages"]["linux-aarch64"])
        sha256 = _tree_digest(temporary)

        shutil.rmtree(output, ignore_errors=True)
        temporary.replace(output)
        artifact = EnvironmentArtifact(
            key=key,
            path=output,
            sha256=sha256,
            channels=channels,
            pypi_bases=pypi_bases,
        )
        (self.cache_dir / f"{key}.json").write_text(
            json.dumps(
                {
                    "directory": output.name,
                    "sha256": artifact.sha256,
                    "channels": artifact.channels,
                    "pypi_bases": artifact.pypi_bases,
                },
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
            def do_HEAD(self) -> None:  # noqa: N802
                self._serve(send_body=False)

            def do_GET(self) -> None:  # noqa: N802
                self._serve(send_body=True)

            def _serve(self, *, send_body: bool) -> None:
                path = self._resolve_path()
                if path is None or not path.is_file():
                    self.send_error(http.HTTPStatus.NOT_FOUND)
                    return
                size = path.stat().st_size
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
                if send_body:
                    with path.open("rb") as source:
                        source.seek(start)
                        shutil.copyfileobj(source, self.wfile)

            def _resolve_path(self) -> Path | None:
                request_path = unquote(urlparse(self.path).path)
                prefix = f"/{token}/"
                if not request_path.startswith(prefix):
                    return None
                relative = request_path.removeprefix(prefix)
                if relative.startswith("conda/"):
                    candidate = artifact.path / "channel" / relative.removeprefix("conda/")
                elif relative.startswith("pypi/"):
                    candidate = artifact.path / "pypi" / Path(relative).name
                else:
                    return None
                try:
                    candidate.resolve().relative_to(artifact.path.resolve())
                except ValueError:
                    return None
                return candidate

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

    def base_url(self, address: str | None = None) -> str:
        address = address or self.bind
        return f"http://{address}:{self.port}/{self.token}"

    def pixi_config(self, address: str | None = None) -> str:
        base_url = self.base_url(address)
        lines = [
            "[concurrency]",
            "downloads = 4",
            "solves = 1",
            "",
            "[repodata-config]",
            "disable-bzip2 = true",
            "disable-sharded = true",
            "disable-zstd = true",
            "",
            "[pypi-config]",
            f"allow-insecure-host = [{json.dumps(urlparse(base_url).netloc)}]",
            "",
            "[mirrors]",
        ]
        for channel in self.artifact.channels:
            lines.append(f"{json.dumps(channel)} = [{json.dumps(f'{base_url}/conda')}]")
        for pypi_base in self.artifact.pypi_bases:
            lines.append(f"{json.dumps(pypi_base)} = [{json.dumps(f'{base_url}/pypi')}]")
        return "\n".join(lines) + "\n"

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


def environment_cache_key(workspace: Path) -> str:
    digest = hashlib.sha256()
    lock = yaml.safe_load((workspace / "pixi.lock").read_text())
    digest.update(
        yaml.safe_dump(
            lock["environments"]["robot"],
            sort_keys=True,
        ).encode()
    )
    digest.update((workspace / "scripts/deploy/pixi-pack-config.toml").read_bytes())
    digest.update(b"linux-aarch64")
    digest.update(CACHE_FORMAT_VERSION.encode())
    return digest.hexdigest()


def _pypi_mirror_bases(packages: list[dict[str, str]]) -> tuple[str, ...]:
    bases = set()
    for package in packages:
        if url := package.get("pypi"):
            parsed = urlparse(url)
            first_path_segment = parsed.path.strip("/").split("/", 1)[0]
            path = f"/{first_path_segment}" if first_path_segment else ""
            bases.add(f"{parsed.scheme}://{parsed.netloc}{path}")
    return tuple(sorted(bases))


def _tree_digest(path: Path) -> str:
    digest = hashlib.sha256()
    for item in sorted(candidate for candidate in path.rglob("*") if candidate.is_file()):
        relative = item.relative_to(path)
        digest.update(str(relative).encode())
        digest.update(str(item.stat().st_size).encode())
        if item.name.endswith(".json") or item.name.endswith(".yml"):
            digest.update(item.read_bytes())
    return digest.hexdigest()
