"""Read-only HTTP dashboard serving immutable snapshots outside the ROS executor."""

import json
import threading
import time
from collections import deque
from dataclasses import asdict
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib.resources import files

from bitbots_auto_referee.core.state import MatchState


class Dashboard:
    def __init__(self, host: str, port: int):
        self._lock = threading.Lock()
        self._snapshot: dict = {}
        self._events: deque[dict] = deque(maxlen=200)
        self._updated_at = time.monotonic()
        page = files("bitbots_auto_referee.ui").joinpath("dashboard.html").read_bytes()
        dashboard = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):  # noqa: N802
                if self.path == "/":
                    content, content_type = page, "text/html; charset=utf-8"
                elif self.path == "/api/state":
                    with dashboard._lock:
                        content = json.dumps(
                            {
                                **dashboard._snapshot,
                                "events": list(dashboard._events),
                                "snapshot_age": time.monotonic() - dashboard._updated_at,
                            },
                            allow_nan=False,
                        ).encode()
                    content_type = "application/json"
                else:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(content)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Content-Type-Options", "nosniff")
                self.end_headers()
                try:
                    self.wfile.write(content)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def log_message(self, format, *args):
                pass

        self._server = ThreadingHTTPServer((host, port), Handler)
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True, name="autoref-dashboard")
        try:
            self._thread.start()
        except Exception:
            self._server.server_close()
            raise

    def record(self, message: str, simulation_time_ns: int | None = None) -> None:
        with self._lock:
            self._events.appendleft(
                {
                    "time": datetime.now(timezone.utc).isoformat(timespec="seconds"),
                    "simulation_time_ns": simulation_time_ns,
                    "message": message,
                }
            )

    def update(
        self,
        state: MatchState,
        simulation_time_ns: int | None,
        last_touch_team_id: int | None,
        connected: bool | None,
    ) -> None:
        snapshot = {
            "match": asdict(state),
            "simulation_time_ns": simulation_time_ns,
            "last_touch_team_id": last_touch_team_id,
            "robot_connected": connected,
        }
        with self._lock:
            self._snapshot = snapshot
            self._updated_at = time.monotonic()

    def close(self) -> None:
        self._server.shutdown()
        self._server.server_close()
        self._thread.join()
