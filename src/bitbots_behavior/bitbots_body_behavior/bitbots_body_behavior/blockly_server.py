#!/usr/bin/env python3
"""Dedicated HTTP server for the Simple Behavior Blockly interface.

Serves the Blockly frontend static files and provides REST endpoints for
saving, loading, listing, and deleting visual behavior designs.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from functools import partial
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, unquote, urlparse


class BlocklyDesignStore:
    """Manages thread-safe persistence for Blockly designs in the filesystem."""

    def __init__(self, designs_directory: Path | str | None = None) -> None:
        self._lock = threading.Lock()
        self.directory = self._resolve_designs_directory(designs_directory)

    @staticmethod
    def _resolve_designs_directory(path: Path | str | None) -> Path:
        if path is None or not str(path).strip():
            path = "~/.bitbots/simple_behavior_designs"
        resolved = Path(path).expanduser().resolve()
        resolved.mkdir(parents=True, exist_ok=True)
        return resolved

    @staticmethod
    def sanitize_design_name(name: str) -> str:
        if not isinstance(name, str):
            raise ValueError("Design name must be a string.")
        trimmed = name.strip()
        if not trimmed:
            raise ValueError("Design name cannot be empty.")
        if "/" in trimmed or "\\" in trimmed or "\0" in trimmed or ".." in trimmed:
            raise ValueError("Design name contains invalid characters.")
        if trimmed.endswith(".json") or trimmed.endswith(".xml"):
            trimmed = Path(trimmed).stem
        if not trimmed or trimmed.startswith("."):
            raise ValueError("Design name cannot start with a dot or be empty.")
        return trimmed

    def list_designs(self) -> list[dict[str, Any]]:
        designs: list[dict[str, Any]] = []
        if not self.directory.is_dir():
            return designs

        with self._lock:
            for file_path in sorted(
                self.directory.iterdir(),
                key=lambda p: p.stat().st_mtime if p.is_file() else 0.0,
                reverse=True,
            ):
                if not file_path.is_file():
                    continue
                suffix = file_path.suffix.lower()
                if suffix not in (".json", ".xml"):
                    continue

                stat = file_path.stat()
                design_name = file_path.stem
                if suffix == ".json":
                    try:
                        data = json.loads(file_path.read_text(encoding="utf-8"))
                        if isinstance(data, dict) and "name" in data and isinstance(data["name"], str):
                            design_name = data["name"]
                    except Exception:
                        pass

                designs.append(
                    {
                        "name": design_name,
                        "filename": file_path.name,
                        "modified": stat.st_mtime,
                        "size": stat.st_size,
                    }
                )
        return designs

    def load_design(self, name: str) -> dict[str, Any]:
        safe_name = self.sanitize_design_name(name)
        with self._lock:
            json_path = self.directory / f"{safe_name}.json"
            xml_path = self.directory / f"{safe_name}.xml"

            if json_path.is_file():
                try:
                    data = json.loads(json_path.read_text(encoding="utf-8"))
                    if isinstance(data, dict):
                        return {
                            "ok": True,
                            "name": data.get("name", safe_name),
                            "xml": data.get("xml", ""),
                            "script": data.get("script", ""),
                            "saved_at": data.get("saved_at", json_path.stat().st_mtime),
                            "metadata": data.get("metadata", {}),
                        }
                except json.JSONDecodeError as error:
                    raise ValueError(f"Design file is corrupted: {error}") from error

            if xml_path.is_file():
                xml_content = xml_path.read_text(encoding="utf-8")
                return {
                    "ok": True,
                    "name": safe_name,
                    "xml": xml_content,
                    "script": "",
                    "saved_at": xml_path.stat().st_mtime,
                    "metadata": {},
                }

        raise FileNotFoundError(f"Design '{name}' not found.")

    def save_design(
        self,
        name: str,
        xml_content: str,
        script_content: str = "",
        metadata: dict[str, Any] | None = None,
    ) -> str:
        safe_name = self.sanitize_design_name(name)
        if not isinstance(xml_content, str) or not xml_content.strip():
            raise ValueError("Field 'xml' must be a non-empty string.")

        target_file = (self.directory / f"{safe_name}.json").resolve()
        if target_file.parent != self.directory.resolve():
            raise ValueError("Invalid target path.")

        design_data = {
            "name": safe_name,
            "xml": xml_content,
            "script": script_content,
            "saved_at": time.time(),
            "metadata": metadata or {},
        }

        with self._lock:
            target_file.write_text(json.dumps(design_data, indent=2), encoding="utf-8")

        return safe_name

    def delete_design(self, name: str) -> bool:
        safe_name = self.sanitize_design_name(name)
        deleted = False
        with self._lock:
            json_path = self.directory / f"{safe_name}.json"
            xml_path = self.directory / f"{safe_name}.xml"

            if json_path.is_file():
                json_path.unlink()
                deleted = True
            if xml_path.is_file():
                xml_path.unlink()
                deleted = True

        return deleted


class BlocklyRequestHandler(SimpleHTTPRequestHandler):
    """HTTP request handler serving Blockly static files and REST design storage."""

    def __init__(self, *args, store: BlocklyDesignStore | None = None, **kwargs) -> None:
        self.store = store or BlocklyDesignStore()
        super().__init__(*args, **kwargs)

    def _send_json(self, status: HTTPStatus, data: dict[str, Any]) -> None:
        body = json.dumps(data).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, DELETE, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type, Accept")
        self.end_headers()
        self.wfile.write(body)

    def end_headers(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        super().end_headers()

    def do_OPTIONS(self) -> None:  # noqa: N802
        self.send_response(HTTPStatus.NO_CONTENT)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, DELETE, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type, Accept")
        self.end_headers()

    def do_GET(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        path = parsed.path.rstrip("/")
        query_params = parse_qs(parsed.query)

        # Redirect root to simple_behavior_blockly.html
        if path in ("", "/"):
            self.send_response(HTTPStatus.FOUND)
            self.send_header("Location", "/simple_behavior_blockly.html")
            self.end_headers()
            return

        if path == "/health":
            self._send_json(HTTPStatus.OK, {"ok": True, "service": "blockly_server"})
            return

        if path == "/designs":
            design_name = query_params.get("name", [None])[0]
            if design_name:
                try:
                    design = self.store.load_design(design_name)
                    self._send_json(HTTPStatus.OK, design)
                except ValueError as err:
                    self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(err)})
                except FileNotFoundError as err:
                    self._send_json(HTTPStatus.NOT_FOUND, {"ok": False, "error": str(err)})
            else:
                designs = self.store.list_designs()
                self._send_json(HTTPStatus.OK, {"ok": True, "designs": designs})
            return

        if path.startswith("/designs/"):
            design_name = unquote(path[len("/designs/") :])
            try:
                design = self.store.load_design(design_name)
                self._send_json(HTTPStatus.OK, design)
            except ValueError as err:
                self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(err)})
            except FileNotFoundError as err:
                self._send_json(HTTPStatus.NOT_FOUND, {"ok": False, "error": str(err)})
            return

        # Otherwise serve static files from self.directory
        super().do_GET()

    def do_POST(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        path = parsed.path.rstrip("/")

        content_length = int(self.headers.get("Content-Length", "0"))
        request_body = self.rfile.read(content_length)

        try:
            payload = json.loads(request_body) if request_body else {}
        except json.JSONDecodeError:
            self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "Request body must be valid JSON."})
            return

        if not isinstance(payload, dict):
            self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "JSON body must be an object."})
            return

        if path == "/designs" or path.startswith("/designs/") or path in ("/save_design", "/save"):
            design_name = payload.get("name")
            if path.startswith("/designs/"):
                path_name = unquote(path[len("/designs/") :])
                if path_name:
                    design_name = path_name

            xml_content = payload.get("xml") or payload.get("design")
            script_content = payload.get("script", "")
            metadata = payload.get("metadata", {})

            if not isinstance(design_name, str):
                self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "Field 'name' must be a string."})
                return

            if not isinstance(xml_content, str):
                self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "Field 'xml' must be a string."})
                return

            try:
                saved_name = self.store.save_design(
                    name=design_name,
                    xml_content=xml_content,
                    script_content=script_content if isinstance(script_content, str) else "",
                    metadata=metadata if isinstance(metadata, dict) else {},
                )
                self._send_json(
                    HTTPStatus.OK,
                    {"ok": True, "message": f"Design '{saved_name}' saved successfully.", "name": saved_name},
                )
            except ValueError as err:
                self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(err)})
            return

        if path in ("/delete_design", "/delete"):
            parsed_query = parse_qs(parsed.query)
            design_name = payload.get("name") or parsed_query.get("name", [None])[0]
            if not isinstance(design_name, str):
                self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "Field 'name' must be a string."})
                return
            try:
                deleted = self.store.delete_design(design_name)
                if deleted:
                    self._send_json(HTTPStatus.OK, {"ok": True, "message": f"Design '{design_name}' deleted."})
                else:
                    self._send_json(
                        HTTPStatus.NOT_FOUND, {"ok": False, "error": f"Design '{design_name}' not found."}
                    )
            except ValueError as err:
                self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(err)})
            return

        self._send_json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "Not found."})

    def do_DELETE(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        path = parsed.path.rstrip("/")
        query_params = parse_qs(parsed.query)

        if path == "/designs" or path.startswith("/designs/"):
            design_name = None
            if path.startswith("/designs/"):
                design_name = unquote(path[len("/designs/") :])
            if not design_name:
                design_name = query_params.get("name", [None])[0]

            if not design_name:
                self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "Design name is required."})
                return

            try:
                deleted = self.store.delete_design(design_name)
                if deleted:
                    self._send_json(HTTPStatus.OK, {"ok": True, "message": f"Design '{design_name}' deleted."})
                else:
                    self._send_json(
                        HTTPStatus.NOT_FOUND, {"ok": False, "error": f"Design '{design_name}' not found."}
                    )
            except ValueError as err:
                self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(err)})
            return

        self._send_json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "Not found."})

    def log_message(self, format: str, *args: Any) -> None:  # noqa: A003
        # Standard silent/custom log or output to stderr
        sys.stderr.write(f"[{self.log_date_time_string()}] {format % args}\n")


def resolve_default_blockly_directory() -> Path:
    # 1. Current working directory if it contains simple_behavior_blockly.html
    cwd = Path.cwd()
    if (cwd / "simple_behavior_blockly.html").is_file():
        return cwd

    # 2. Package relative directory (source tree)
    src_docs_blockly = Path(__file__).resolve().parent.parent / "docs" / "blockly"
    if (src_docs_blockly / "simple_behavior_blockly.html").is_file():
        return src_docs_blockly

    # 3. Fallback to current working directory
    return cwd


def create_blockly_server(
    host: str = "127.0.0.1",
    port: int = 8000,
    directory: Path | str | None = None,
    designs_directory: Path | str | None = None,
) -> ThreadingHTTPServer:
    static_dir = Path(directory).resolve() if directory else resolve_default_blockly_directory()
    store = BlocklyDesignStore(designs_directory)
    handler = partial(BlocklyRequestHandler, directory=str(static_dir), store=store)
    server = ThreadingHTTPServer((host, port), handler)
    server.daemon_threads = True
    return server


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run dedicated Simple Behavior Blockly web and design storage server."
    )
    parser.add_argument(
        "port_positional",
        nargs="?",
        type=int,
        default=None,
        help="Server port (positional, default 8000).",
    )
    parser.add_argument(
        "-p",
        "--port",
        type=int,
        default=None,
        help="Server port (default 8000).",
    )
    parser.add_argument(
        "-b",
        "--bind",
        "--host",
        dest="host",
        default="0.0.0.0",
        help="Host/IP address to bind to (default 0.0.0.0).",
    )
    parser.add_argument(
        "-d",
        "--directory",
        type=str,
        default=None,
        help="Directory to serve static files from (default: docs/blockly).",
    )
    parser.add_argument(
        "--designs-dir",
        "--designs-directory",
        dest="designs_dir",
        type=str,
        default=os.getenv("BITBOTS_SIMPLE_BEHAVIOR_DESIGNS_DIR", "~/.bitbots/simple_behavior_designs"),
        help="Directory where designs are stored (default: ~/.bitbots/simple_behavior_designs).",
    )

    args = parser.parse_args()
    port = args.port if args.port is not None else (args.port_positional if args.port_positional is not None else 8000)
    static_dir = Path(args.directory).resolve() if args.directory else resolve_default_blockly_directory()
    designs_dir = Path(args.designs_dir).expanduser().resolve()

    server = create_blockly_server(
        host=args.host,
        port=port,
        directory=static_dir,
        designs_directory=designs_dir,
    )

    print(f"Serving Simple Behavior Blockly on http://{args.host}:{port}/")
    print(f"Static directory: {static_dir}")
    print(f"Designs directory: {designs_dir}")
    print("Press Ctrl+C to stop.")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping Blockly server...")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
