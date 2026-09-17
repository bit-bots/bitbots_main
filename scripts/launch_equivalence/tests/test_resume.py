import fcntl
import json

import pytest
import verify


@pytest.fixture
def comparison(monkeypatch, tmp_path):
    class Comparison:
        stop_after = None
        interrupted = False
        requests = []
        workers = []

        def run(self, output, *args):
            monkeypatch.setattr(verify.sys, "argv", ["verify.py", "--output", str(output), *args])
            return verify.main()

    comparison = Comparison()

    class Worker:
        def __init__(self, root, scratch, engine, timeout):
            self.engine = engine
            self.calls = 0
            self.closed = False
            comparison.workers.append(self)

        def evaluate(self, request):
            if comparison.stop_after is not None and self.calls >= comparison.stop_after:
                comparison.interrupted = True
                raise KeyboardInterrupt
            self.calls += 1
            comparison.requests.append((self.engine, request))
            return {
                "records": [{"node_name": "node", "parameters": {"value": 1 if self.engine == "ros" else 2}}],
                "errors": [{"type": "Unsupported", "message": "example"}]
                if request["arguments"]["sim"] == "true"
                else [],
            }

        def close(self):
            self.closed = True

    monkeypatch.setattr(verify, "Worker", Worker)
    monkeypatch.setattr(verify, "export", lambda *args: None)
    monkeypatch.setattr(verify, "discover", lambda *args: [{"entrypoint": "p/f", "old": "old", "new": "new"}])
    monkeypatch.setattr(
        verify,
        "git",
        lambda *args: (
            '<launch><arg name="audio" default="false"/><arg name="sim" default="false"/></launch>'
            if args[-1].endswith(":old")
            else "@launch_this\ndef entry(audio: bool = False, sim: bool = False):\n    pass\n"
        )
        if args[0] == "show"
        else "revision",
    )
    return comparison


def test_resume_matches_uninterrupted_run_and_skips_completed(comparison, tmp_path, monkeypatch):
    complete, resumed = tmp_path / "complete", tmp_path / "resumed"
    assert comparison.run(complete) == 2
    comparison.stop_after = 2
    assert comparison.run(resumed) == 130
    summary = json.loads((resumed / "summary.json").read_text())
    assert summary["aborted"] and summary["completed_cases"] == 2
    assert (resumed / "differences.diff").read_text().startswith("ABORTED after 2 of 4")
    assert all(worker.closed for worker in comparison.workers)
    comparison.stop_after = None
    comparison.requests.clear()
    # Resume uses the saved plan, without resolving refs or rediscovering files.
    monkeypatch.setattr(verify, "git", lambda *args: pytest.fail("unexpected git plan lookup"))
    assert comparison.run(resumed, "--resume", "--max-cases", "1") == 2
    assert len(comparison.requests) == 4
    assert all(request["arguments"]["audio"] == "true" for _, request in comparison.requests)
    for name in ("cases.jsonl", "differences.diff"):
        assert (resumed / name).read_bytes() == (complete / name).read_bytes()
    summary = json.loads((resumed / "summary.json").read_text())
    reference = json.loads((complete / "summary.json").read_text())
    assert summary["results"] == reference["results"]
    assert summary["groups"] == reference["groups"]
    assert not summary["aborted"] and summary["completed_cases"] == 4 and summary["resumed_cases"] == 2
    comparison.requests.clear()
    worker_count = len(comparison.workers)
    monkeypatch.setattr(verify, "export", lambda *args: pytest.fail("finished report needs no exports"))
    assert comparison.run(resumed, "--resume") == 2
    assert not comparison.requests and len(comparison.workers) == worker_count


def test_resume_older_report_without_summary_and_with_partial_last_line(comparison, tmp_path):
    comparison.stop_after = 1
    report = tmp_path / "report"
    assert comparison.run(report) == 130
    (report / "summary.json").unlink()
    (report / "differences.diff").unlink()
    with (report / "cases.jsonl").open("ab") as stream:
        stream.write(b'{"entrypoint":')
    comparison.stop_after = None
    comparison.requests.clear()
    assert comparison.run(report, "--resume") == 2
    summary = json.loads((report / "summary.json").read_text())
    assert summary["resumed_cases"] == 1 and summary["completed_cases"] == 4
    assert len(comparison.requests) == 6
    assert len((report / "cases.jsonl").read_text().splitlines()) == 4


@pytest.mark.parametrize("damage", ["journal", "identity", "finding", "manifest"])
def test_resume_refuses_corruption_without_modifying_journal(comparison, tmp_path, damage):
    comparison.stop_after = 1
    report = tmp_path / "report"
    assert comparison.run(report) == 130
    journal = report / "cases.jsonl"
    case = json.loads(journal.read_text())
    if damage == "journal":
        journal.write_text("invalid json\n")
    elif damage == "identity":
        case["arguments"]["sim"] = "true"
        journal.write_text(json.dumps(case) + "\n")
    else:
        finding = report / f"{case['groups'][0]}.json"
        if damage == "finding":
            finding.unlink()
        else:
            (report / json.loads(finding.read_text())["manifests"]).write_text("invalid")
    with journal.open("ab") as stream:
        stream.write(b"partial")
    before = journal.read_bytes()
    with pytest.raises(SystemExit) as error:
        comparison.run(report, "--resume")
    assert error.value.code == 2
    assert journal.read_bytes() == before


def test_resume_rejects_new_domains_and_concurrent_writer(comparison, tmp_path):
    report = tmp_path / "report"
    assert comparison.run(report) == 2
    with pytest.raises(SystemExit) as error:
        comparison.run(report, "--resume", "--base", "different-revision")
    assert error.value.code == 2
    with (report / ".lock").open("a") as stream:
        fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with pytest.raises(SystemExit) as error:
            comparison.run(report, "--resume")
        assert error.value.code == 2
