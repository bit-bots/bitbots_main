#!/usr/bin/env python3
"""Compare old ROS launch and migrated better-launch without starting processes."""

import argparse
import ast
import itertools
import json
import math
import selectors
import subprocess
import sys
import tarfile
import tempfile
import time
import xml.etree.ElementTree as ET
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from manifest import differences, digest, render_diff, stable

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]


def git(*args):
    return subprocess.check_output(["git", "-C", str(REPO), *args], text=True).strip()


def python_arguments(source):
    """Inspect literal function signatures without importing executable launch code."""
    result = {}
    for node in ast.parse(source).body:
        if not isinstance(node, ast.FunctionDef):
            continue
        if not any(
            (isinstance(d, ast.Name) and d.id == "launch_this")
            or (isinstance(d, ast.Call) and isinstance(d.func, ast.Name) and d.func.id == "launch_this")
            for d in node.decorator_list
        ):
            continue
        args = node.args.posonlyargs + node.args.args
        defaults = [None] * (len(args) - len(node.args.defaults)) + node.args.defaults
        for arg, default in zip(args + node.args.kwonlyargs, defaults + node.args.kw_defaults, strict=True):
            literal = False
            value = None
            if default is not None:
                try:
                    value = ast.literal_eval(default)
                    literal = True
                except (ValueError, TypeError):
                    pass
            result[arg.arg] = {
                "type": ast.unparse(arg.annotation) if arg.annotation else None,
                "default": value,
                "literal": literal,
            }
    return result


def xml_arguments(source):
    result = {}
    declarations_first = True
    for node in ET.fromstring(source):
        if node.tag != "arg":
            declarations_first = False
            continue
        name = node.attrib["name"]
        value = node.get("default")
        literal = (
            declarations_first
            and value is not None
            and "$(" not in value
            and not ("if" in node.attrib or "unless" in node.attrib)
        )
        if name in result:
            literal = False
        result[name] = {"default": value, "literal": literal}
    return result


def binding_only(source):
    """Conservatively exclude entrypoints that may inspect argument presence or raw CLI input."""
    allowed_imports = {"better_launch", "logging", "os", "pathlib", "yaml", "datetime"}
    forbidden_names = {
        "argv",
        "orig_argv",
        "sys",
        "inspect",
        "eval",
        "exec",
        "compile",
        "getattr",
        "globals",
        "locals",
        "vars",
        "__import__",
        "get_current_context",
        "get_parameter_source",
    }
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Import) and any(
            alias.name.split(".")[0] not in allowed_imports for alias in node.names
        ):
            return False
        if isinstance(node, ast.ImportFrom) and (not node.module or node.module.split(".")[0] not in allowed_imports):
            return False
        if isinstance(node, ast.Name) and node.id in forbidden_names:
            return False
        if isinstance(node, ast.Attribute) and node.attr in forbidden_names:
            return False
        if isinstance(node, ast.Constant) and isinstance(node.value, str) and "/proc/" in node.value:
            return False
    return True


def domains_for(old_source, new_source, overrides=None):
    before = xml_arguments(old_source) if old_source.lstrip().startswith("<") else {}
    after = python_arguments(new_source)
    domains, reductions, bounded = {}, [], []
    for name in sorted(before.keys() | after.keys() | (overrides or {}).keys()):
        info = after.get(name, {})
        values = (overrides or {}).get(name)
        if values is None:
            if info.get("type") == "bool" or before.get(name, {}).get("default") in {"true", "false"}:
                values = [None, "false", "true"]
            else:
                values = [None]
                bounded.append(name)
        if (
            not isinstance(values, list)
            or not values
            or not all(value is None or isinstance(value, str) for value in values)
        ):
            raise ValueError(f"Domain {name} must be a nonempty list of strings or null (omitted)")
        original = list(dict.fromkeys(values))
        values = list(original)
        old = before.get(name, {})
        # ROS receives these arguments through IncludeLaunchDescription, not LaunchContext.argv.
        # Literal, unconditional XML defaults produce exactly the same LaunchConfiguration;
        # Click produces exactly the same typed value on the better-launch side.
        if info.get("type") == "bool" and info.get("literal") and old.get("literal") and binding_only(new_source):
            default = str(info["default"]).lower()
            if type(info["default"]) is bool and old["default"] == default and None in values and default in values:
                values.remove(None)
                reductions.append(
                    {
                        "argument": name,
                        "omitted_equals": default,
                        "reason": "identical unconditional XML default and typed Python default",
                    }
                )
        domains[name] = {"values": values, "original": original}
    return domains, reductions, bounded


def assignments(domains):
    keys = list(domains)
    for values in itertools.product(*(domains[key]["values"] for key in keys)):
        yield {key: value for key, value in zip(keys, values, strict=True) if value is not None}


def discover(base, head):
    old_files = set(git("ls-tree", "-r", "--name-only", base).splitlines())
    new_files = git("ls-tree", "-r", "--name-only", head).splitlines()
    result = []
    for new in new_files:
        if "/launch/" not in new or not new.endswith(".launch.py") or new.startswith("src/lib/"):
            continue
        source = git("show", f"{head}:{new}")
        if not python_arguments(source):
            # Functions without arguments are valid launch entrypoints too.
            if "@launch_this" not in source:
                continue
        old = new if new in old_files else new.removesuffix(".py")
        if old in old_files:
            result.append(
                {"old": old, "new": new, "entrypoint": new.split("/launch/")[0].split("/")[-1] + "/" + Path(new).name}
            )
    return result


def export(revision, path):
    path.mkdir()
    with tempfile.TemporaryFile() as archive:
        subprocess.run(["git", "-C", str(REPO), "archive", revision, "src"], stdout=archive, check=True)
        archive.seek(0)
        with tarfile.open(fileobj=archive) as tar:
            tar.extractall(path, filter="data")


class Worker:
    def __init__(self, root, scratch, engine, timeout):
        self.timeout = timeout
        self.stderr = tempfile.TemporaryFile(mode="w+")
        self.process = subprocess.Popen(
            [
                sys.executable,
                "-B",
                str(HERE / "worker.py"),
                "--root",
                str(root),
                "--scratch",
                str(scratch),
                "--engine",
                engine,
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=self.stderr,
            text=True,
            bufsize=1,
        )
        self.selector = selectors.DefaultSelector()
        self.selector.register(self.process.stdout, selectors.EVENT_READ)

    def evaluate(self, request, ancestors=()):
        identity = stable(request)
        if identity in ancestors or len(ancestors) >= 16:
            return {"records": [], "errors": [{"type": "Unsupported", "message": "Recursive nested launch"}]}
        try:
            self.process.stdin.write(stable(request) + "\n")
            self.process.stdin.flush()
            if not self.selector.select(self.timeout):
                self.process.kill()
                raise TimeoutError("Offline evaluator timed out")
            line = self.process.stdout.readline()
            if not line:
                self.stderr.seek(0)
                raise RuntimeError("Worker stopped: " + self.stderr.read()[-6000:])
            result = json.loads(line)
            for child in result.pop("children", []):
                nested = self.evaluate(child["request"], ancestors + (identity,))
                result["records"].extend(
                    record | {"process_boundary": child["boundary"] + "/" + record.get("process_boundary", "")}
                    for record in nested["records"]
                )
                result["errors"].extend(error | {"process_boundary": child["boundary"]} for error in nested["errors"])
                result["cache"] = nested.get("cache", result.get("cache", {}))
            return result
        except (BrokenPipeError, TimeoutError, RuntimeError, json.JSONDecodeError) as exception:
            return {"records": [], "errors": [{"type": type(exception).__name__, "message": str(exception)}]}

    def close(self):
        try:
            self.process.stdin.close()
        except BrokenPipeError:
            pass
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        self.selector.close()
        self.stderr.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", default="main", help="Old ROS launch revision")
    parser.add_argument("--head", default="HEAD", help="Migrated revision; uncommitted files are excluded")
    parser.add_argument("--entry", action="append", help="Package/launch-file selector; repeatable")
    parser.add_argument("--domains", type=Path, help="JSON with per-entrypoint arguments and environment profiles")
    parser.add_argument("--output", type=Path, default=Path("/tmp/launch-equivalence-report"))
    parser.add_argument(
        "--plan", action="store_true", help="Print inventory, domains, and counts without evaluating code"
    )
    parser.add_argument(
        "--defaults-only", action="store_true", help="Compare omitted arguments only; reports limited coverage"
    )
    parser.add_argument("--max-cases", type=int, default=100000, help="Refuse larger matrices; never silently truncate")
    parser.add_argument("--timeout", type=float, default=30, help="Worker response timeout")
    options = parser.parse_args()
    if options.timeout <= 0 or options.max_cases < 1:
        parser.error("Timeout and case limit must be positive")
    base, head = git("rev-parse", f"{options.base}^{{commit}}"), git("rev-parse", f"{options.head}^{{commit}}")
    specification = json.loads(options.domains.read_text()) if options.domains else {}
    profiles = specification.get("environments", {"unset_robot": {}})
    if not profiles:
        parser.error("At least one environment profile is required")
    inventory = discover(base, head)
    if not inventory:
        parser.error("No paired migrated launch entrypoints found in the selected revisions")
    unknown_domains = set(specification.get("arguments", {})) - {pair["entrypoint"] for pair in inventory}
    if unknown_domains:
        parser.error(f"Unknown entrypoints in domains file: {sorted(unknown_domains)}")
    if options.entry:
        selected = set(options.entry)
        inventory = [pair for pair in inventory if pair["entrypoint"] in selected or pair["new"] in selected]
        missing = selected - {key for pair in inventory for key in (pair["entrypoint"], pair["new"])}
        if missing:
            parser.error(f"Unknown entrypoints: {sorted(missing)}")
    plan = []
    for pair in inventory:
        overrides = specification.get("arguments", {}).get(pair["entrypoint"], {})
        domains, reductions, bounded = domains_for(
            git("show", f"{base}:{pair['old']}"), git("show", f"{head}:{pair['new']}"), overrides
        )
        if options.defaults_only:
            domains = {key: {"values": [None], "original": [None]} for key in domains}
            reductions = []
        plan.append(
            pair
            | {
                "domains": domains,
                "reductions": reductions,
                "default_only_arguments": bounded,
                "cases": math.prod(len(d["values"]) for d in domains.values()) * len(profiles),
                "covered_assignments": math.prod(len(d["original"]) for d in domains.values()) * len(profiles),
            }
        )
    planned = sum(pair["cases"] for pair in plan)
    metadata = {
        "base": base,
        "head": head,
        "environments": profiles,
        "defaults_only": options.defaults_only,
        "planned_cases": planned,
        "covered_assignments": sum(pair["covered_assignments"] for pair in plan),
        "entrypoints": plan,
    }
    if options.plan:
        print(json.dumps(metadata, indent=2))
        return 0
    if planned > options.max_cases:
        parser.error(
            f"Matrix requires {planned} cases after reduction. Inspect --plan and supply domains or raise --max-cases."
        )
    if options.output.exists() and any(options.output.iterdir()):
        parser.error("Output directory is not empty; choose a fresh directory to keep reports separate")
    options.output.mkdir(parents=True, exist_ok=True)
    (options.output / "examples").mkdir()
    (options.output / "plan.json").write_text(json.dumps(metadata, indent=2) + "\n")
    groups = {}
    counts = {"equivalent": 0, "different": 0, "unresolved": 0}
    cache = {}
    cache_hits = 0
    process_cache = {}
    started = time.monotonic()
    with tempfile.TemporaryDirectory(prefix="launch-equivalence-") as temporary:
        temporary = Path(temporary)
        old_root, new_root = temporary / "old", temporary / "new"
        export(base, old_root)
        export(head, new_root)
        workers = [
            Worker(old_root, temporary / "old-worker", "ros", options.timeout),
            Worker(new_root, temporary / "new-worker", "better", options.timeout),
        ]
        try:
            with (options.output / "cases.jsonl").open("w") as cases, ThreadPoolExecutor(max_workers=2) as executor:
                for pair in plan:
                    print(f"Comparing {pair['entrypoint']} ({pair['cases']} cases)", flush=True)
                    for profile, environment in profiles.items():
                        for arguments in assignments(pair["domains"]):
                            requests = [
                                dict(file=pair[side], arguments=arguments, environment=environment)
                                for side in ("old", "new")
                            ]
                            outputs, futures = [None, None], []
                            for index, (worker, request) in enumerate(zip(workers, requests, strict=True)):
                                key = stable([index, request])
                                if key in cache:
                                    outputs[index] = cache[key]
                                    cache_hits += 1
                                else:
                                    futures.append((index, key, executor.submit(worker.evaluate, request)))
                            for index, key, future in futures:
                                outputs[index] = cache[key] = future.result()
                                if len(cache) > 128:
                                    cache.pop(next(iter(cache)))
                                process_cache[("old", "new")[index]] = outputs[index].get("cache", {})
                            delta = differences(outputs[0]["records"], outputs[1]["records"])
                            errors = {
                                side: result["errors"]
                                for side, result in zip(("old", "new"), outputs, strict=True)
                                if result["errors"]
                            }
                            status = "unresolved" if errors else "different" if delta else "equivalent"
                            counts[status] += 1
                            case = {
                                "entrypoint": pair["entrypoint"],
                                "arguments": arguments,
                                "environment": profile,
                                "status": status,
                                "differences": delta,
                                "errors": errors,
                            }
                            signatures = []
                            findings = [{"differences": [difference], "errors": {}} for difference in delta]
                            findings.extend(
                                {"differences": [], "errors": {side: [error]}}
                                for side, exceptions in errors.items()
                                for error in exceptions
                            )
                            saved_example = None
                            for finding in findings:
                                signature = digest([pair["entrypoint"], finding])
                                signatures.append(signature)
                                if signature not in groups:
                                    example = case | finding
                                    groups[signature] = {"count": 0, "example": example}
                                    if saved_example is None:
                                        saved_example = (
                                            f"examples/{digest([pair['entrypoint'], profile, arguments])}.json"
                                        )
                                        (options.output / saved_example).write_text(
                                            json.dumps({"old": outputs[0], "new": outputs[1]}, indent=2) + "\n"
                                        )
                                    (options.output / f"{signature}.json").write_text(
                                        json.dumps({"case": example, "manifests": saved_example}, indent=2) + "\n"
                                    )
                                groups[signature]["count"] += 1
                            cases.write(
                                stable(
                                    {key: value for key, value in case.items() if key not in {"differences", "errors"}}
                                    | {"groups": signatures}
                                )
                                + "\n"
                            )
                    print(f"  totals: {counts}", flush=True)
        finally:
            for worker in workers:
                worker.close()
    summary = metadata | {
        "results": counts,
        "cache_hits": cache_hits,
        "process_cache": process_cache,
        "elapsed_seconds": time.monotonic() - started,
        "groups": groups,
        "scope": "launch instructions in the declared finite domains; no runtime equivalence claim",
    }
    (options.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    with (options.output / "differences.diff").open("w") as report:
        for signature, group in groups.items():
            example = group["example"]
            report.write(f"\n{example['entrypoint']} | {group['count']} cases | group {signature}\n")
            report.write(f"arguments={stable(example['arguments'])} environment={example['environment']}\n")
            report.write(render_diff(example["differences"]))
            if example["errors"]:
                report.write("UNRESOLVED: " + stable(example["errors"]) + "\n")
    print(f"{counts}; report: {options.output}")
    return 2 if counts["unresolved"] else 1 if counts["different"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
