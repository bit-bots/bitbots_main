import json
import os
import subprocess
import tempfile
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from rclpy.experimental.events_executor import EventsExecutor
from rclpy.node import Node


class SimpleBehaviorProcessManager:
    _STOP_TIMEOUT_SECONDS = 1.0

    def __init__(self, node):
        self._node = node
        self._lock = threading.Lock()
        self._process = None
        self._script_path = Path(tempfile.gettempdir()) / "bitbots_simple_behavior_web_script.py"

    def _validate_script(self, script_source: str, source_name: str) -> tuple[bool, str]:
        if not script_source.strip():
            return False, "Submitted script is empty."

        try:
            compiled_script = compile(script_source, source_name, "exec")
            module_globals: dict[str, object] = {"__name__": "bitbots_user_simple_behavior"}
            exec(compiled_script, module_globals)  # noqa: S102 - required to run user-provided behavior script
        except Exception as error:
            return False, str(error)

        main_function = module_globals.get("main")
        if main_function is None or not callable(main_function):
            return False, "Simple behavior script must provide a callable main() function."

        return True, "ok"

    def _stop_process_locked(self) -> None:
        process = self._process
        if process is None:
            return

        if process.poll() is not None:
            self._process = None
            return

        process.terminate()
        try:
            process.wait(timeout=self._STOP_TIMEOUT_SECONDS)
        except subprocess.TimeoutExpired:
            self._node.get_logger().warn("Timed out while stopping previous simple behavior process. Sending SIGKILL.")
            process.kill()
            process.wait(timeout=self._STOP_TIMEOUT_SECONDS)

        self._process = None

    def _start_process_locked(self) -> None:
        command = [
            "ros2",
            "launch",
            "bitbots_body_behavior",
            "behavior.launch",
            "behavior_mode:=simple",
            f"simple_behavior_script:={self._script_path}",
        ]
        self._process = subprocess.Popen(command, start_new_session=True, text=True, env=os.environ.copy())
        self._node.get_logger().info(f"Started simple behavior process with PID {self._process.pid}.")

    def submit_script(self, script_source: str, source_name: str) -> tuple[bool, str]:
        is_valid, message = self._validate_script(script_source, source_name)
        if not is_valid:
            return False, message

        with self._lock:
            self._script_path.write_text(script_source, encoding="utf-8")
            self._stop_process_locked()
            self._start_process_locked()

        return True, "Script accepted and behavior restarted."

    def get_state(self) -> str:
        with self._lock:
            process = self._process
            if process is None:
                return "idle"
            if process.poll() is None:
                return "running"
            self._process = None
            return "idle"

    def shutdown(self) -> None:
        with self._lock:
            self._stop_process_locked()


class SimpleBehaviorWebWrapper:
    _SERVER_THREAD_JOIN_TIMEOUT_SECONDS = 0.5

    def __init__(self, node):
        self._node = node
        self._process_manager = SimpleBehaviorProcessManager(node)
        self._web_host = str(node.get_parameter("simple_behavior_web_host").value)
        self._web_port = int(node.get_parameter("simple_behavior_web_port").value)
        self._http_server: ThreadingHTTPServer | None = None
        self._http_server_thread: threading.Thread | None = None
        self._start_web_server()

    def _start_web_server(self) -> None:
        wrapper = self

        class RequestHandler(BaseHTTPRequestHandler):
            def _send_json(self, status_code: int, payload: dict[str, object]) -> None:
                body = json.dumps(payload).encode("utf-8")
                self.send_response(status_code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
                self.send_header("Access-Control-Allow-Headers", "Content-Type")
                self.end_headers()
                self.wfile.write(body)

            def do_OPTIONS(self) -> None:  # noqa: N802
                self.send_response(HTTPStatus.NO_CONTENT)
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
                self.send_header("Access-Control-Allow-Headers", "Content-Type")
                self.end_headers()

            def do_GET(self) -> None:  # noqa: N802
                if self.path != "/health":
                    self._send_json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "Not found."})
                    return
                self._send_json(HTTPStatus.OK, {"ok": True, "state": wrapper._process_manager.get_state()})

            def do_POST(self) -> None:  # noqa: N802
                if self.path != "/run":
                    self._send_json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "Not found."})
                    return

                content_length = int(self.headers.get("Content-Length", "0"))
                request_body = self.rfile.read(content_length)

                try:
                    payload = json.loads(request_body)
                except json.JSONDecodeError:
                    self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "Request body must be valid JSON."})
                    return

                if not isinstance(payload, dict):
                    self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "JSON body must be an object."})
                    return

                script_source = payload.get("script")
                source_name = payload.get("name", "simple_behavior_submitted.py")
                if not isinstance(script_source, str):
                    self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "Field 'script' must be a string."})
                    return
                if not isinstance(source_name, str):
                    source_name = "simple_behavior_submitted.py"

                accepted, message = wrapper._process_manager.submit_script(script_source, source_name)
                if accepted:
                    wrapper._node.get_logger().info("Accepted simple behavior script from web endpoint.")
                    self._send_json(HTTPStatus.OK, {"ok": True, "message": message})
                else:
                    self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": message})

            def log_message(self, format, *args) -> None:  # noqa: A003
                return

        try:
            self._http_server = ThreadingHTTPServer((self._web_host, self._web_port), RequestHandler)
        except OSError as error:
            self._node.get_logger().error(
                f"Could not start simple behavior web server on {self._web_host}:{self._web_port}: {error}"
            )
            return

        self._http_server.daemon_threads = True
        self._http_server_thread = threading.Thread(
            target=self._http_server.serve_forever,
            kwargs={"poll_interval": 0.1},
            daemon=True,
            name="simple_behavior_web_server",
        )
        self._http_server_thread.start()
        self._node.get_logger().info(f"Simple behavior web server listening on http://{self._web_host}:{self._web_port}")

    def shutdown(self) -> None:
        http_server = self._http_server
        http_server_thread = self._http_server_thread
        if http_server is not None:
            http_server.shutdown()
            http_server.server_close()
            self._node.get_logger().info("Simple behavior web server stopped.")
        if http_server_thread is not None and http_server_thread.is_alive():
            http_server_thread.join(timeout=self._SERVER_THREAD_JOIN_TIMEOUT_SECONDS)

        self._process_manager.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = Node("simple_behavior_web_wrapper", automatically_declare_parameters_from_overrides=True)
    wrapper = SimpleBehaviorWebWrapper(node)

    executor = EventsExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        wrapper.shutdown()
        node.destroy_node()
        rclpy.shutdown()
