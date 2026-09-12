import ctypes
import errno
import json

import pytest
import sandbox
import verify
from manifest import Normalizer, differences, semantic
from verify import REPO, Worker, WorkerError, assignments, domains_for


@pytest.mark.parametrize(
    ("abi", "error", "message"),
    [(2, 0, "ABI 2 detected"), (-1, errno.ENOSYS, "ENOSYS"), (-1, errno.EOPNOTSUPP, "disabled")],
)
def test_landlock_failure_diagnostics(monkeypatch, tmp_path, abi, error, message):
    class Libc:
        class Syscall:
            def __call__(self, *args):
                ctypes.set_errno(error)
                return abi

        syscall = Syscall()

    monkeypatch.setattr(sandbox.ctypes, "CDLL", lambda *args, **kwargs: Libc())
    with pytest.raises(RuntimeError, match=message):
        sandbox.contain(tmp_path)


def test_worker_startup_failure_preserves_cause(monkeypatch, tmp_path):
    (tmp_path / "worker.py").write_text('raise RuntimeError("Landlock unavailable: test failure")\n')
    monkeypatch.setattr(verify, "HERE", tmp_path)
    with pytest.raises(WorkerError, match="Landlock unavailable: test failure"):
        Worker(tmp_path, tmp_path, "ros", 5)


@pytest.mark.parametrize(
    ("response", "message"),
    [
        ('raise RuntimeError("worker crashed")', "worker crashed"),
        ('print("invalid", flush=True)', "invalid JSON"),
        ('print("{}", flush=True)', "invalid evaluation"),
        ('print("{", end="", flush=True); time.sleep(30)', "timed out"),
    ],
)
def test_worker_transport_failure_is_fatal(monkeypatch, tmp_path, response, message):
    (tmp_path / "worker.py").write_text(
        "import sys, time\nprint('{\"ready\": true}', flush=True)\nsys.stdin.readline()\n" + response + "\n"
    )
    monkeypatch.setattr(verify, "HERE", tmp_path)
    worker = Worker(tmp_path, tmp_path, "ros", 5)
    worker.timeout = 0.2
    try:
        with pytest.raises(WorkerError, match=message):
            worker.evaluate({"file": "unused"})
    finally:
        worker.close()
    assert worker.process.poll() is not None


@pytest.mark.parametrize("completed", [0, 1])
def test_controller_aborts_without_counting_unattempted_cases(monkeypatch, tmp_path, completed):
    workers = []

    class FailingWorker:
        def __init__(self, root, scratch, engine, timeout):
            if not completed and engine == "better":
                raise WorkerError("startup failure")
            self.calls = 0
            self.closed = False
            workers.append(self)

        def evaluate(self, request):
            self.calls += 1
            if self.calls > completed:
                raise WorkerError("evaluation failure")
            return {"records": [], "errors": []}

        def close(self):
            self.closed = True

    monkeypatch.setattr(verify, "Worker", FailingWorker)
    monkeypatch.setattr(verify, "export", lambda *args: None)
    monkeypatch.setattr(verify, "discover", lambda *args: [{"entrypoint": "p/f", "old": "old", "new": "new"}])
    monkeypatch.setattr(
        verify,
        "git",
        lambda *args: (
            '<launch><arg name="sim" default="false"/></launch>'
            if args[-1].endswith(":old")
            else "@launch_this\ndef entry(sim: bool = False):\n    pass\n"
        )
        if args[0] == "show"
        else "revision",
    )
    report = tmp_path / "report"
    monkeypatch.setattr(verify.sys, "argv", ["verify.py", "--output", str(report)])
    assert verify.main() == 2
    summary = json.loads((report / "summary.json").read_text())
    assert summary["aborted"] is True
    assert summary["completed_cases"] == completed
    assert summary["planned_cases"] == 2
    assert summary["results"] == {"equivalent": completed, "different": 0, "unresolved": 0}
    assert (report / "differences.diff").read_text().startswith("ABORTED")
    assert all(worker.closed for worker in workers)
    assert all(worker.calls <= completed + 1 for worker in workers)


def test_default_reduction_preserves_different_defaults_and_conditional_arguments():
    old = '<launch><arg name="a" default="true"/><arg name="b" default="false"/><arg name="c" default="true" if="$(var a)"/></launch>'
    new = "@launch_this\ndef entry(a: bool = True, b: bool = True, c: bool = True):\n    pass\n"
    domains, reductions, _ = domains_for(old, new)
    assert domains["a"]["values"] == ["false", "true"]
    assert domains["b"]["values"] == [None, "false", "true"]
    assert domains["c"]["values"] == [None, "false", "true"]
    assert [r["argument"] for r in reductions] == ["a"]
    assert len(list(assignments(domains))) == 18


def test_domains_do_not_execute_source():
    source = "@launch_this\ndef entry(a: str = dangerous()):\n    raise RuntimeError()\n"
    domains, reductions, bounded = domains_for("<launch/>", source)
    assert list(assignments(domains)) == [{}]
    assert reductions == []
    assert bounded == ["a"]


def test_domain_requires_a_list_instead_of_iterating_a_string():
    with pytest.raises(ValueError, match="nonempty list"):
        domains_for("<launch/>", "@launch_this\ndef entry(sim: bool = False):\n    pass\n", {"sim": "false"})


def test_reduction_keeps_arguments_used_before_their_declaration():
    old = '<launch><node if="$(var sim)" pkg="pkg" exec="exe"/><arg name="sim" default="false"/></launch>'
    new = "@launch_this\ndef entry(sim: bool = False):\n    pass\n"
    domains, reductions, _ = domains_for(old, new)
    assert domains["sim"]["values"] == [None, "false", "true"]
    assert not reductions


def test_reduction_keeps_raw_cli_observations():
    old = '<launch><arg name="sim" default="false"/></launch>'
    new = "import sys\n@launch_this\ndef entry(sim: bool = False):\n    print(sys.argv)\n"
    domains, reductions, _ = domains_for(old, new)
    assert domains["sim"]["values"] == [None, "false", "true"]
    assert not reductions


def test_parameter_precedence_types_and_cache_invalidation(tmp_path):
    config = tmp_path / "config.yaml"
    config.write_text(
        "/**:\n  ros__parameters:\n    enabled: true\n    gain: 2\n/ns/node:\n  ros__parameters:\n    gain: 3\n"
    )
    normalizer = Normalizer([])
    command = [
        "package://pkg/exe",
        "--ros-args",
        "-r",
        "__node:=node",
        "-r",
        "__ns:=/ns",
        "--params-file",
        str(config),
        "-p",
        'enabled:="false"',
    ]
    kwargs = dict(env={}, cwd="/", shell=False, policy={}, source=[])
    first = normalizer.process(command, **kwargs)
    assert first["parameters"] == {"enabled": "false", "gain": 3}
    assert semantic(normalizer.process(command, **kwargs)) == semantic(first)
    assert normalizer.process_hits == 1
    config.write_text("/**:\n  ros__parameters:\n    gain: 4\n")
    assert normalizer.process(command, **kwargs)["parameters"]["gain"] == 4


def test_diff_preserves_types_and_duplicate_nodes():
    node = {"node_name": "node", "namespace": "/", "parameters": {"enabled": True}}
    other = node | {"parameters": {"enabled": 1}}
    assert differences([node], [other])[0]["field"] == "parameters.enabled"
    assert any(d["process"] == "/node#1" for d in differences([node, node], [node]))


@pytest.fixture
def launch_pair(tmp_path):
    root = tmp_path / "revision"
    package = root / "src/test_package"
    package.mkdir(parents=True)
    (package / "package.xml").write_text("<package><name>test_package</name></package>")
    (root / "src/lib").mkdir()
    (root / "src/lib/better_launch").symlink_to(REPO / "src/lib/better_launch", target_is_directory=True)
    workers = []

    def evaluate(old, new, arguments=None):
        (package / "old.launch").write_text(old)
        (package / "new.launch.py").write_text(new)
        results = []
        for engine, file in (("ros", "old.launch"), ("better", "new.launch.py")):
            worker = Worker(root, tmp_path / f"worker-{len(workers)}", engine, 30)
            workers.append(worker)
            results.append(worker.evaluate({"file": f"src/test_package/{file}", "arguments": arguments or {}}))
        return results

    yield evaluate, package, tmp_path
    for worker in workers:
        worker.close()


def test_native_evaluators_equal_without_executable(launch_pair):
    evaluate, _, _ = launch_pair
    old, new = evaluate(
        '<launch><arg name="sim" default="false"/><node pkg="missing_binary" exec="missing" name="test_node" output="screen"><param name="use_sim_time" value="$(var sim)"/></node></launch>',
        'from better_launch import BetterLaunch, launch_this\n@launch_this\ndef entry(sim: bool = False):\n    BetterLaunch().node("missing_binary", "missing", "test_node", params={"use_sim_time": sim}, log_level=None, lifecycle_target=None)\n',
        {"sim": "true"},
    )
    assert not old["errors"], "\n".join(e.get("traceback", e["message"]) for e in old["errors"])
    assert not new["errors"], new
    assert len(old["records"]) == len(new["records"]) == 1
    assert differences(old["records"], new["records"]) == []


def test_native_detects_string_boolean_forwarding(launch_pair):
    evaluate, package, _ = launch_pair
    (package / "child.launch.py").write_text(
        'from better_launch import BetterLaunch, launch_this\n@launch_this\ndef child(enabled: bool = False):\n    if enabled:\n        BetterLaunch().node("pkg", "exe", "unexpected", log_level=None, lifecycle_target=None)\n'
    )
    old, new = evaluate(
        '<launch><arg name="enabled" default="false"/><node if="$(var enabled)" pkg="pkg" exec="exe" name="unexpected" output="screen"/></launch>',
        'from better_launch import BetterLaunch, launch_this\n@launch_this\ndef entry(enabled: str = "false"):\n    BetterLaunch().include("test_package", "child.launch.py", enabled=enabled)\n',
        {"enabled": "false"},
    )
    assert not old["errors"], old
    assert not new["errors"], new
    assert old["records"] == []
    assert new["records"][0]["node_name"] == "unexpected"


@pytest.mark.parametrize(
    "operation",
    [
        'import subprocess; subprocess.run(["/bin/true"])',
        "import socket; socket.socket()",
        "import ctypes; assert ctypes.CDLL(None, use_errno=True).socket(2, 1, 0) == -1",
    ],
)
def test_containment(launch_pair, operation):
    evaluate, _, _ = launch_pair
    _, result = evaluate(
        "<launch/>", operation + "\nfrom better_launch import launch_this\n@launch_this\ndef entry():\n    pass\n"
    )
    if "ctypes" in operation:
        assert result["errors"] == [], result
    else:
        assert any(e["type"] == "UnsafeOperation" for e in result["errors"]), result


def test_landlock_prevents_source_writes(launch_pair):
    evaluate, package, _ = launch_pair
    marker = package / "should_not_exist"
    _, result = evaluate("<launch/>", f'from pathlib import Path\nPath({str(marker)!r}).write_text("bad")\n')
    assert result["errors"], result
    assert not marker.exists()


def test_unknown_action_is_unresolved(launch_pair):
    evaluate, _, _ = launch_pair
    old, _ = evaluate(
        '<launch><include file="/missing/launch.xml"/></launch>',
        "from better_launch import launch_this\n@launch_this\ndef entry():\n    pass\n",
    )
    assert old["errors"]
    json.dumps(old)


def test_timer_is_evaluated_without_waiting(launch_pair):
    evaluate, _, _ = launch_pair
    old, new = evaluate(
        '<launch><timer period="600"><node pkg="pkg" exec="exe" name="delayed" output="screen"/></timer></launch>',
        'from better_launch import BetterLaunch, launch_this\n@launch_this\ndef entry():\n    bl = BetterLaunch()\n    bl.run_later(600.0, lambda: bl.node("pkg", "exe", "delayed", log_level=None, lifecycle_target=None))\n',
    )
    assert not old["errors"], old
    assert not new["errors"], new
    assert old["records"][0]["policy"]["triggers"][0]["after_seconds"] == 600
    assert differences(old["records"], new["records"]) == []


def test_native_component_request_without_ros_graph(launch_pair):
    evaluate, package, _ = launch_pair
    xml = '<launch><node_container pkg="rclcpp_components" exec="component_container" name="container" namespace="/test" output="screen"><composable_node pkg="not_installed" plugin="Example" name="component"><param name="enabled" value="true"/></composable_node></node_container></launch>'
    (package / "components.launch").write_text(xml)
    old, new = evaluate(
        xml,
        'from better_launch import BetterLaunch, launch_this\n@launch_this\ndef entry():\n    BetterLaunch().include("test_package", "components.launch")\n',
    )
    assert not old["errors"], old
    assert not new["errors"], new
    assert old["records"][1]["plugin"] == "Example"
    assert old["records"][1]["parameters"] == {"enabled": True}
    assert differences(old["records"], new["records"]) == []


def test_nested_launcher_is_expanded_without_spawning_it(launch_pair):
    evaluate, package, _ = launch_pair
    (package / "child.launch").write_text(
        '<launch><arg name="sim" default="false"/><node pkg="pkg" exec="exe" name="child" output="screen"><param name="use_sim_time" value="$(var sim)"/></node></launch>'
    )
    (package / "child.launch.py").write_text(
        'from better_launch import BetterLaunch, launch_this\n@launch_this\ndef child(sim: bool = False):\n    BetterLaunch().node("pkg", "exe", "child", params={"use_sim_time": sim}, log_level=None, lifecycle_target=None)\n'
    )
    old, new = evaluate(
        '<launch><executable cmd="ros2 launch test_package child.launch sim:=true" output="screen"/></launch>',
        'from better_launch import BetterLaunch, launch_this\n@launch_this\ndef entry():\n    BetterLaunch().process(["bl", "test_package", "child.launch.py", "--sim", "true"])\n',
    )
    assert not old["errors"], old
    assert not new["errors"], new
    assert len(old["records"]) == len(new["records"]) == 2
    assert old["records"][1]["parameters"] == {"use_sim_time": True}
    assert differences(old["records"], new["records"]) == []


def test_better_launch_swallowed_preparation_error_is_not_success(launch_pair):
    evaluate, package, _ = launch_pair
    (package / "broken.yaml").write_text("this: [is: not: yaml")
    _, new = evaluate(
        "<launch/>",
        'from better_launch import BetterLaunch, launch_this\n@launch_this\ndef entry():\n    bl = BetterLaunch()\n    bl.node("pkg", "exe", "node", params=bl.find("test_package", "broken.yaml"))\n',
    )
    assert new["errors"], new
    assert new["records"] == []


def test_missing_parameter_file_is_not_silently_skipped(launch_pair):
    evaluate, _, _ = launch_pair
    old, new = evaluate(
        '<launch><node pkg="pkg" exec="exe" name="node" output="screen"><param from="/definitely/missing/config.yaml"/></node></launch>',
        'from better_launch import BetterLaunch, launch_this\n@launch_this\ndef entry():\n    BetterLaunch().node("pkg", "exe", "node", param_files="/definitely/missing/config.yaml")\n',
    )
    assert old["errors"], old
    assert new["errors"], new
