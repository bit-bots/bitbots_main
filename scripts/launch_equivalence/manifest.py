"""Stable process manifests and typed, field-level differences."""

import difflib
import hashlib
import json
import re
from pathlib import Path

import yaml


def stable(value):
    return json.dumps(value, sort_keys=True, ensure_ascii=False, allow_nan=False)


def digest(value):
    return hashlib.sha256(stable(value).encode()).hexdigest()


def flatten(value, prefix=""):
    if isinstance(value, dict):
        for key, child in value.items():
            yield from flatten(child, f"{prefix}.{key}" if prefix else str(key))
    else:
        yield prefix, value


def parameter_blocks(data, prefix=""):
    if not isinstance(data, dict):
        raise ValueError("Parameter file must contain a mapping")
    for key, value in data.items():
        if key == "ros__parameters":
            yield prefix or "/**", dict(flatten(value))
        else:
            yield from parameter_blocks(value, f"{prefix}/{key}".replace("//", "/"))


def matches(selector, node):
    tokens = selector.strip("/").split("/")
    pattern = ""
    for token in tokens:
        if token == "**":
            pattern += r"(?:/[^/]+)*"
        elif token == "*":
            pattern += r"/[^/]+"
        elif "*" in token:
            raise ValueError(f"Unsupported parameter selector: {selector}")
        else:
            pattern += "/" + re.escape(token)
    return re.fullmatch(pattern, node) is not None


class Normalizer:
    def __init__(self, replacements):
        self.replacements = sorted(replacements, key=lambda pair: -len(pair[0]))
        self.yaml_cache = {}
        self.cache_hits = 0
        self.process_cache = {}
        self.process_hits = 0

    def clean(self, value):
        if isinstance(value, str):
            for original, replacement in self.replacements:
                value = value.replace(original, replacement)
            return value
        if isinstance(value, dict):
            return {self.clean(k): self.clean(v) for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [self.clean(v) for v in value]
        return value

    def read_params(self, path):
        content = Path(path).read_text()
        if content in self.yaml_cache:
            self.cache_hits += 1
        else:
            self.yaml_cache[content] = list(parameter_blocks(yaml.safe_load(content)))
        return self.yaml_cache[content]

    def process(self, command, *, env, cwd, shell, policy, source):
        """Decode ROS command options, retaining application argument and remap order."""
        command = list(command)
        files = [Path(command[i + 1]).read_text() for i, arg in enumerate(command[:-1]) if arg == "--params-file"]
        cache_command = list(command)
        contents = iter(files)
        for i, arg in enumerate(command[:-1]):
            if arg == "--params-file":
                cache_command[i + 1] = "<parameters:" + digest(next(contents)) + ">"
        cache_key = digest([cache_command, env, cwd, shell, policy])
        if cache_key in self.process_cache:
            self.process_hits += 1
            return self.process_cache[cache_key] | {"source": self.clean(source), "raw_command": self.clean(command)}
        app, remaps, parameters, options = [], [], [], []
        name, namespace = None, "/"
        identity_rules = set()
        ros_args = False
        index = 0
        while index < len(command):
            token = command[index]
            index += 1
            if token == "--ros-args":
                ros_args = True
                continue
            if ros_args and token == "--":
                ros_args = False
                continue
            if not ros_args:
                app.append(token)
                continue
            if token in {
                "-r",
                "--remap",
                "-p",
                "--param",
                "--params-file",
                "--log-level",
                "--log-config-file",
                "--enclave",
                "-e",
            }:
                value = command[index]
                index += 1
                if token in {"-r", "--remap"}:
                    key, target = value.split(":=", 1)
                    if key in {"__node", "__name"}:
                        if "name" in identity_rules:
                            raise ValueError("Multiple node-name remap rules require native remap resolution")
                        identity_rules.add("name")
                        name = target
                    elif key == "__ns":
                        if "namespace" in identity_rules:
                            raise ValueError("Multiple namespace remap rules require native remap resolution")
                        identity_rules.add("namespace")
                        namespace = "/" + target.strip("/")
                    else:
                        remaps.append([key, target])
                elif token in {"-p", "--param"}:
                    key, target = value.split(":=", 1)
                    parameters.append(("inline", key, yaml.safe_load(target)))
                elif token == "--params-file":
                    parameters.append(("file", value, self.read_params(value)))
                elif token == "--log-level":
                    options.append(["--log-level", value.upper()])
                else:
                    options.append([token, value])
            else:
                options.append([token])
        fullname = namespace.rstrip("/") + "/" + name if name else None
        effective = {}
        unresolved_selectors = []
        provenance = []
        for kind, key, value in parameters:
            provenance.append({"kind": kind, "source": self.clean(key)})
            if kind == "inline":
                if ":" in key:
                    qualifier, key = key.split(":", 1)
                    if name is None:
                        unresolved_selectors.append([qualifier, {key: value}])
                        continue
                    if qualifier != name:
                        continue
                effective[key] = value
            else:
                for selector, values in value:
                    if selector == "/**" or (fullname and matches(selector, fullname)):
                        effective.update(values)
                    elif fullname is None:
                        unresolved_selectors.append([selector, values])
        result = self.clean(
            {
                "application_command": app,
                "node_name": name,
                "namespace": namespace,
                "remaps": remaps,
                "parameters": effective,
                "unresolved_parameter_selectors": unresolved_selectors,
                "ros_options": options,
                "environment": env,
                "cwd": cwd,
                "shell": shell,
                "policy": policy,
                "source": source,
                "raw_command": command,
                "parameter_sources": provenance,
            }
        )
        self.process_cache[cache_key] = result
        return dict(result)


def semantic(manifest):
    return {k: v for k, v in manifest.items() if k not in {"source", "raw_command", "parameter_sources"}}


def differences(old, new):
    """Align processes by identity and occurrence, preserving duplicate processes."""

    def indexed(records):
        result = {}
        for record in records:
            command = record.get("application_command", [])
            identity = (
                record.get("node_name")
                or record.get("path")
                or (command[0] if command else record.get("kind", "event"))
            )
            identity = f"/{record.get('process_boundary', '')}/{record.get('namespace', '/')}/{identity}"
            while "//" in identity:
                identity = identity.replace("//", "/")
            occurrence = 0
            while f"{identity}#{occurrence}" in result:
                occurrence += 1
            result[f"{identity}#{occurrence}"] = record
        return result

    left, right = indexed(old), indexed(new)
    result = []
    if list(left) != list(right):
        result.append(
            {"process": "@launch", "field": "order", "old": {"value": list(left)}, "new": {"value": list(right)}}
        )
    for key in sorted(left.keys() | right.keys()):
        before = dict(flatten(semantic(left[key]))) if key in left else {}
        after = dict(flatten(semantic(right[key]))) if key in right else {}
        for field in sorted(before.keys() | after.keys()):
            a = {"value": before[field]} if field in before else {"absent": True}
            b = {"value": after[field]} if field in after else {"absent": True}
            if stable(a) != stable(b):
                result.append({"process": key, "field": field, "old": a, "new": b})
    return result


def render_diff(delta):
    left, right = [], []
    for item in delta:
        label = f"{item['process']}.{item['field']}"
        left.append(f"{label}: {stable(item['old'])}\n")
        right.append(f"{label}: {stable(item['new'])}\n")
    return "".join(difflib.unified_diff(left, right, fromfile="ros2 launch", tofile="better-launch"))
