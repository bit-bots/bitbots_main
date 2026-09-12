"""Native launch evaluation in a contained, disposable worker process."""

import argparse
import datetime
import importlib
import inspect
import io
import json
import logging
import os
import runpy
import shlex
import shutil
import sys
import tempfile
import traceback
import xml.etree.ElementTree as ET
from concurrent.futures import Future
from contextlib import contextmanager, redirect_stderr, redirect_stdout
from pathlib import Path

from manifest import Normalizer
from sandbox import UnsafeOperation, contain


class Captured(BaseException):
    pass


class UnsupportedError(Exception):
    pass


class PackageIndex:
    def __init__(self, root, scratch):
        self.root, self.scratch = root, scratch
        self.packages = {}
        for manifest in sorted(root.glob("src/**/package.xml")):
            try:
                name = ET.parse(manifest).getroot().findtext("name")
            except ET.ParseError:
                continue
            if name in self.packages:
                raise ValueError(f"Duplicate source package {name}")
            self.packages[name] = manifest.parent
        self.external = Path(sys.prefix)
        self.used = {}

    def prefix(self, package):
        if package not in self.packages or package in {
            "launch",
            "launch_xml",
            "launch_yaml",
            "launch_ros",
            "ros2launch",
        }:
            if not (self.external / "share" / package).is_dir():
                raise UnsupportedError(f"Missing package resources: {package}")
            return str(self.external)
        prefix = self.scratch / "packages" / package
        share = prefix / "share" / package
        if not share.exists():
            # Files remain read-only through Landlock; newly generated files stay in scratch.
            def overlay(source, destination):
                destination.mkdir(parents=True)
                for entry in source.iterdir():
                    target = destination / entry.name
                    if entry.is_dir() and not entry.is_symlink():
                        overlay(entry, target)
                    else:
                        target.symlink_to(entry)

            overlay(self.packages[package], share)
        self.used[package] = share
        return str(prefix)

    def share(self, package, **kwargs):
        return str(Path(self.prefix(package)) / "share" / package)


class Evaluator:
    def __init__(self, root, scratch, engine):
        self.root, self.scratch, self.engine = root, scratch, engine
        self.index = PackageIndex(root, scratch)
        self.records, self.errors, self.sources, self.triggers = [], [], [], []
        self.raw_node = None
        self.node_options = {}
        self.saved_env = dict(os.environ)
        self.deferred = []
        self.native_versions = {}
        self.patch_native()

    def patch_native(self):
        import ament_index_python.packages as packages
        import launch
        import xacro
        from launch.actions import ExecuteProcess
        from launch.actions.execute_local import ExecuteLocal
        from launch.substitutions import Command, FindExecutable
        from launch.utilities import perform_substitutions
        from launch_ros.actions import Node as RosNode
        from launch_ros.actions import node as ros_node_module
        from launch_ros.substitutions import ExecutableInPackage

        self.launch = launch
        self.RosNode = RosNode
        self.ExecuteProcess = ExecuteProcess
        self.perform = perform_substitutions
        self.xacro = xacro
        evaluate_parameters = ros_node_module.evaluate_parameters

        def checked_parameters(context, parameters):
            evaluated = evaluate_parameters(context, parameters)
            for parameter in evaluated:
                if isinstance(parameter, Path) and not parameter.is_file():
                    self.error(UnsupportedError(f"Missing parameter file: {parameter}"))
            return evaluated

        ros_node_module.evaluate_parameters = checked_parameters
        self.native_versions["launch"] = str(Path(launch.__file__).relative_to(Path(sys.prefix)))

        originals = {
            packages.get_package_prefix: self.index.prefix,
            packages.get_package_share_directory: self.index.share,
        }
        if self.engine == "better":
            sys.path.insert(0, str(self.root / "src/lib/better_launch"))
            import better_launch

            self.bl_module = importlib.import_module("better_launch.launcher")
            self.bl_settings = importlib.import_module("better_launch.utils.settings")
            self.bl_wrapper = importlib.import_module("better_launch.wrapper")
            self.BL = better_launch.BetterLaunch
            self.BNode = importlib.import_module("better_launch.elements.node").Node
            self.native_versions["better_launch"] = better_launch.__version__

        # Modules often import these helpers by value.
        for module in tuple(sys.modules.values()):
            if module is None or not hasattr(module, "__dict__"):
                continue
            for key, value in list(vars(module).items()):
                if inspect.isfunction(value) and value in originals:
                    setattr(module, key, originals[value])

        def executable(action, context):
            package = self.perform(context, action.package)
            name = self.perform(context, action.executable)
            return f"package://{package}/{name}"

        ExecutableInPackage.perform = executable
        FindExecutable.perform = lambda action, context: "command://" + self.perform(context, action.name)
        Command.perform = lambda action, context: self.command(shlex.split(self.perform(context, action.command)))
        ExecuteLocal.execute = lambda action, context: self.ros_process(action, context)
        # Normal launch logging may create files at import time or during preparation.
        launch.logging.launch_config.log_dir = str(self.scratch / "logs")
        if self.engine == "better":
            self.patch_better()

    def command(self, args):
        if Path(args[0]).name == "cat" and len(args) == 2:
            return Path(args[1]).read_text()
        if Path(args[0]).name != "xacro" and args[0] != "command://xacro":
            raise UnsupportedError(f"Command substitution requires execution: {args!r}")
        mappings = {}
        paths = []
        for value in args[1:]:
            if ":=" in value:
                key, value = value.split(":=", 1)
                mappings[key] = value
            elif value.startswith("-"):
                raise UnsupportedError(f"Unsupported Xacro option: {value}")
            else:
                paths.append(value)
        if len(paths) != 1:
            raise UnsupportedError(f"Expected one Xacro input: {args!r}")
        return self.xacro.process_file(paths[0], mappings=mappings).toxml()

    @contextmanager
    def source(self, name):
        self.sources.append(str(name))
        try:
            yield
        finally:
            self.sources.pop()

    def error(self, exception):
        self.errors.append(
            {
                "source": list(self.sources),
                "type": type(exception).__name__,
                "message": str(exception),
                "traceback": "".join(traceback.format_exception(exception, limit=10)),
            }
        )

    def capture(self, command, env, cwd, shell, policy):
        policy["triggers"] = list(self.triggers)
        # Express inheritance without copying unrelated host environment values into reports.
        environment = {key: value for key, value in env.items() if self.case_env.get(key) != value}
        removed = sorted(self.case_env.keys() - env.keys())
        environment = {"overrides": environment, "removed": removed}
        record = self.normalizer.process(
            command,
            env=environment,
            cwd=cwd or str(self.scratch),
            shell=shell,
            policy=policy,
            source=list(self.sources),
        )
        self.records.append(record)
        program = Path(command[0]).name
        offset = (
            2 if program == "ros2" and len(command) > 1 and command[1] == "launch" else 1 if program == "bl" else None
        )
        if offset is not None:
            if shell:
                raise UnsupportedError("Shell-wrapped nested launches require shell interpretation")
            package, filename = command[offset : offset + 2]
            args = command[offset + 2 :]
            arguments = {}
            if program == "ros2":
                for arg in args:
                    key, value = arg.split(":=", 1)
                    arguments[key] = value
            else:
                if len(args) % 2:
                    raise UnsupportedError("Nested better-launch arguments must be option/value pairs")
                for key, value in zip(args[::2], args[1::2], strict=True):
                    if not key.startswith("--"):
                        raise UnsupportedError(f"Unknown nested launcher option: {key}")
                    arguments[key[2:]] = value
            if package not in self.index.packages:
                raise UnsupportedError(f"Nested launch package unavailable in snapshot: {package}")
            candidates = list(self.index.packages[package].rglob(filename))
            if len(candidates) != 1:
                raise UnsupportedError(f"Cannot uniquely resolve nested launch {package}/{filename}")
            boundary = f"launch-process-{len(self.children)}"
            record["child_boundary"] = boundary
            record["application_command"] = ["<launch>", package, filename.removesuffix(".py")]
            tracked = self.saved_env.keys() | env.keys() | {"ROBOT_NAME", "ROBOCUP_ROBOT_ID", "ROS_DOMAIN_ID"}
            child_env = {
                key: env.get(key)
                for key in tracked
                if self.saved_env.get(key) != env.get(key) or key in {"ROBOT_NAME", "ROBOCUP_ROBOT_ID", "ROS_DOMAIN_ID"}
            }
            self.children.append(
                {
                    "boundary": boundary,
                    "request": {
                        "file": str(candidates[0].relative_to(self.root)),
                        "arguments": arguments,
                        "environment": child_env,
                    },
                }
            )

    def ros_process(self, action, context):
        action.prepare(context)
        details = action.process_details
        output = action._ExecuteLocal__output
        if not isinstance(output, dict):
            output = self.perform(context, output)
        respawn = action._ExecuteLocal__respawn
        retries = action._ExecuteLocal__respawn_max_retries if respawn else 0
        policy = {
            "output": output,
            "respawn_retries": retries,
            "respawn_delay": action._ExecuteLocal__respawn_delay if respawn else 0.0,
            "emulate_tty": action.emulate_tty,
            "lifecycle_target": None,
        }
        self.capture(details["cmd"], details["env"], details["cwd"], action.shell, policy)
        if action._ExecuteLocal__on_exit:
            raise UnsupportedError("Process exit callback requires runtime event modeling")
        return None

    def walk(self, entities, context):
        from launch import LaunchDescription
        from launch.actions import IncludeLaunchDescription, TimerAction
        from launch.utilities.type_utils import perform_typed_substitution
        from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes

        allowed = {
            "DeclareLaunchArgument",
            "SetLaunchConfiguration",
            "GroupAction",
            "PushLaunchConfigurations",
            "PopLaunchConfigurations",
            "ResetLaunchConfigurations",
            "PushEnvironment",
            "PopEnvironment",
            "ResetEnvironment",
            "SetEnvironmentVariable",
            "UnsetEnvironmentVariable",
            "OpaqueFunction",
            "LogInfo",
            "Log",
            "SetParameter",
            "SetParametersFromFile",
            "SetRemap",
            "PushROSNamespace",
        }
        for entity in entities:
            try:
                condition = getattr(entity, "condition", None)
                if condition is not None and not condition.evaluate(context):
                    continue
                if type(entity) is LaunchDescription:
                    self.walk(entity.entities, context)
                elif type(entity) is IncludeLaunchDescription:
                    children = entity.execute(context)
                    with self.source(entity.launch_description_source.location):
                        self.walk(children or [], context)
                elif type(entity) is TimerAction:
                    delay = perform_typed_substitution(context, entity._TimerAction__period, float)
                    cancel = perform_typed_substitution(context, entity._TimerAction__cancel_on_shutdown, bool)
                    self.deferred.append(
                        (
                            delay,
                            cancel,
                            list(entity._TimerAction__actions),
                            dict(context.launch_configurations),
                            dict(context.environment),
                            list(self.sources),
                        )
                    )
                elif type(entity) is LoadComposableNodes:
                    self.components(entity, context)
                elif (
                    type(entity) in {self.RosNode, self.ExecuteProcess, ComposableNodeContainer}
                    or type(entity).__name__ in allowed
                ):
                    children = entity.execute(context)
                    if children:
                        self.walk(children, context)
                else:
                    raise UnsupportedError(f"Unsupported ROS action: {type(entity).__module__}.{type(entity).__name__}")
            except (Exception, UnsafeOperation) as exception:
                self.error(exception)

    def components(self, action, context):
        from launch.utilities import normalize_to_list_of_substitutions
        from launch_ros.actions import ComposableNodeContainer
        from launch_ros.actions.load_composable_nodes import get_composable_node_load_request
        from rclpy.parameter import parameter_value_to_python

        target = action._LoadComposableNodes__target_container
        if isinstance(target, ComposableNodeContainer):
            target = target.node_name
        else:
            target = self.perform(context, normalize_to_list_of_substitutions(target))
        for description in action._LoadComposableNodes__composable_node_descriptions:
            request = get_composable_node_load_request(description, context)
            if request is None:
                continue
            self.records.append(
                self.normalizer.clean(
                    {
                        "kind": "component",
                        "container": target,
                        "package": request.package_name,
                        "plugin": request.plugin_name,
                        "node_name": request.node_name,
                        "namespace": request.node_namespace or "/",
                        "remaps": list(request.remap_rules),
                        "parameters": {
                            param.name: parameter_value_to_python(param.value) for param in request.parameters
                        },
                        "extra_arguments": {
                            param.name: parameter_value_to_python(param.value) for param in request.extra_arguments
                        },
                        "log_level": request.log_level,
                        "source": list(self.sources),
                        "triggers": list(self.triggers),
                    }
                )
            )
            if getattr(description, "node_autostart", False):
                raise UnsupportedError("Lifecycle component autostart requires event modeling")

    def ros_file(self, file, args, context=None):
        from launch import LaunchContext
        from launch.actions import IncludeLaunchDescription
        from launch.launch_description_sources import AnyLaunchDescriptionSource

        if context is None:
            context = LaunchContext()
        action = IncludeLaunchDescription(AnyLaunchDescriptionSource(str(file)), launch_arguments=args.items())
        self.walk([action], context)

    def patch_better(self):
        import subprocess

        from better_launch.elements import abstract_node
        from better_launch.ros import logging as roslog

        self.BL.hello = lambda *_: None

        def unique_name(bl, name="", check_running_nodes=True):
            self.unique_counter += 1
            return f"{name}_anonymous_{self.unique_counter}"

        self.BL.get_unique_name = unique_name
        self.BL.spin = lambda *_args, **_kwargs: None
        self.BL.shutdown = lambda *_args, **_kwargs: None
        self.bl_wrapper._init_signal_handlers = lambda: None
        self.bl_wrapper.init_logging = lambda *_: None
        abstract_node.configure_logger = lambda *_args, **_kwargs: None
        roslog.launch_config.log_dir = str(self.scratch / "logs")
        self.BNode.is_ros2_connected = lambda *_args, **_kwargs: False

        native_find = self.BL.find

        def find(bl, package=None, filename=None, subdir="**"):
            if package == "/lib" or (package == "" and filename):
                return filename
            if package and package.endswith("/lib"):
                return f"package://{package[:-4]}/{filename}"
            return native_find(bl, package, filename, subdir)

        self.BL.find = find
        self.BL.exec = classmethod(lambda cls, cmd: self.command(shlex.split(cmd) if isinstance(cmd, str) else cmd))
        native_which = shutil.which
        shutil.which = (
            lambda command, *args, **kwargs: f"command://{command}"
            if command in {"bl", "ros2"}
            else native_which(command, *args, **kwargs)
        )
        native_node = self.BL.node
        signature = inspect.signature(native_node)

        def node(bl, *args, **kwargs):
            bound = signature.bind(bl, *args, **kwargs)
            bound.apply_defaults()
            self.node_options = bound.arguments
            if not bound.arguments["autostart_process"]:
                raise UnsupportedError("Deferred manual node start is not modeled")
            try:
                return native_node(bl, *args, **kwargs)
            except (Exception, UnsafeOperation) as exception:
                self.error(exception)
                return None

        self.BL.node = node

        native_start = self.BNode.start

        def start(node):
            self.raw_node = node
            self.capture_attempted = False
            try:
                native_start(node)
            except Captured:
                pass
            finally:
                if not self.capture_attempted:
                    self.error(
                        UnsupportedError(
                            f"Better-launch did not reach process capture for {node.fullname}; command preparation failed"
                        )
                    )
                self.raw_node = None

        self.BNode.start = start

        def popen(command, **kwargs):
            if self.raw_node is None:
                raise UnsafeOperation("Unexpected subprocess creation")
            self.capture_attempted = True
            node = self.raw_node
            options = self.node_options
            output = options["output"]
            if hasattr(output, "name"):
                output = output.name.lower()
            target = options["lifecycle_target"]
            if hasattr(target, "name"):
                target = target.name
            try:
                self.capture(
                    command,
                    kwargs["env"],
                    kwargs["cwd"],
                    kwargs["shell"],
                    {
                        "output": output,
                        "respawn_retries": node.max_respawns,
                        "respawn_delay": node.respawn_delay if node.max_respawns else 0.0,
                        "emulate_tty": False,
                        "lifecycle_target": None if node.raw else target,
                    },
                )
            except (Exception, UnsafeOperation) as exception:
                self.error(exception)
            if options["on_exit"]:
                self.error(UnsupportedError("Process exit callback requires runtime event modeling"))
            raise Captured()

        subprocess.Popen = popen

        native_include = self.BL.include

        def include(bl, package, launchfile, subdir=None, **kwargs):
            with self.source(f"{package}/{launchfile}"):
                try:
                    native_include(bl, package, launchfile, subdir, **kwargs)
                except (Exception, UnsafeOperation) as exception:
                    self.error(exception)

        self.BL.include = include

        def ros2_actions(bl, *actions):
            # The actual wrapper shares a LaunchContext among its queued actions.
            if self.ros_context is None:
                self.ros_context = self.launch.LaunchContext()
            self.walk(actions, self.ros_context)

        self.BL.ros2_actions = ros2_actions

        def later(bl, delay, callback, *args, **kwargs):
            self.deferred.append((delay, True, (callback, args, kwargs), None, None, list(self.sources)))
            return Future()

        self.BL.run_later = later
        self.BL.ros2_launch_service = lambda *_args, **_kwargs: self.unsupported("Direct ROS LaunchService access")
        self.BL.ros_adapter = property(lambda _: self.unsupported("ROS graph access"))
        self.BL.compose = lambda *_args, **_kwargs: self.unsupported("Composable node container")

    @staticmethod
    def unsupported(message):
        raise UnsupportedError(message)

    def reset(self, request):
        os.environ.clear()
        os.environ.update(self.saved_env)
        for key in ("ROBOT_NAME", "ROBOCUP_ROBOT_ID", "ROS_DOMAIN_ID"):
            os.environ.pop(key, None)
        for key, value in request.get("environment", {}).items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = str(value)
        os.environ["ROS_LOG_DIR"] = str(self.scratch / "logs")
        os.environ["BL_UI"] = "false"
        self.case_env = dict(os.environ)
        self.records, self.errors, self.sources, self.triggers, self.deferred = [], [], [], [], []
        self.unique_counter = 0
        self.children = []
        self.ros_context = None
        if self.engine == "better":
            self.bl_module.__dict__.pop("__better_launch_instance", None)
            self.bl_module.BetterLaunchMeta._singleton_future = Future()
            self.BL._launchfile = None
            self.BL._launch_func_args = {}
            self.bl_settings._SETTINGS = self.bl_settings._Settings()
        replacements = [
            (str(self.root), "<revision>"),
            (str(self.scratch), "<scratch>"),
            (str(Path(sys.prefix)), "<environment>"),
        ]
        for package, source in self.index.packages.items():
            replacements.extend(
                [
                    (str(source), f"package-share://{package}"),
                    (str(self.scratch / "packages" / package / "share" / package), f"package-share://{package}"),
                ]
            )
        replacements.append((str(Path.home()), "<home>"))
        if not hasattr(self, "normalizer"):
            self.normalizer = Normalizer(replacements)

    def evaluate(self, request):
        self.reset(request)
        path = self.root / request["file"]
        args = request.get("arguments", {})
        stream = io.StringIO()
        with redirect_stdout(stream), redirect_stderr(stream), self.source(path):
            try:
                if self.engine == "ros":
                    self.ros_file(path, args)
                else:
                    sys.argv = [str(path)] + [word for name, value in args.items() for word in (f"--{name}", value)]
                    runpy.run_path(str(path), run_name="__main__")
                pending, self.deferred = self.deferred, []
                for delay, cancel, action, configurations, environment, sources in pending:
                    self.sources = sources
                    self.triggers = [{"after_seconds": delay, "cancel_on_shutdown": cancel}]
                    if configurations is None:
                        callback, callback_args, callback_kwargs = action
                        callback(*callback_args, **callback_kwargs)
                    else:
                        context = self.launch.LaunchContext()
                        context.launch_configurations.update(configurations)
                        context.environment.clear()
                        context.environment.update(environment)
                        self.walk(action, context)
                if self.deferred:
                    self.error(UnsupportedError("Nested timers require event modeling"))
            except (Exception, UnsafeOperation, SystemExit) as exception:
                self.error(exception)
        for share in self.index.used.values():
            for path in share.rglob("*"):
                if path.is_file() and not path.is_symlink():
                    self.records.append(
                        {
                            "kind": "generated_file",
                            "path": self.normalizer.clean(str(path)),
                            "content": path.read_text(),
                        }
                    )
                    path.unlink()
        # A missing native name prevents a sound choice of node-specific YAML selectors.
        for record in self.records:
            if record.get("unresolved_parameter_selectors"):
                self.error(
                    UnsupportedError(
                        f"Cannot resolve parameter selectors without the executable's default node name: {record.get('application_command')}"
                    )
                )
        return {
            "records": self.records,
            "errors": self.normalizer.clean(self.errors),
            "log": self.normalizer.clean(stream.getvalue()[-20000:]),
            "versions": self.native_versions,
            "children": self.children,
            "cache": {
                "unique_processes": len(self.normalizer.process_cache),
                "reused_processes": self.normalizer.process_hits,
            },
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--scratch", type=Path, required=True)
    parser.add_argument("--engine", choices=["ros", "better"], required=True)
    options = parser.parse_args()
    options.scratch.mkdir(parents=True, exist_ok=True)
    (options.scratch / "logs").mkdir(exist_ok=True)
    tempfile.tempdir = str(options.scratch)
    os.chdir(options.scratch)
    # Freeze only the source of wall-clock timestamps, not their formatting or use.
    original_datetime = datetime.datetime

    class FrozenDatetime(original_datetime):
        @classmethod
        def now(cls, tz=None):
            return cls.fromtimestamp(0, tz)

    datetime.datetime = FrozenDatetime
    evaluator = Evaluator(options.root, options.scratch, options.engine)
    contain(options.scratch)
    logging.disable(logging.CRITICAL)
    print(json.dumps({"ready": True}), flush=True)
    for line in sys.stdin:
        try:
            result = evaluator.evaluate(json.loads(line))
        except BaseException as exception:
            result = {
                "records": [],
                "errors": [{"type": type(exception).__name__, "message": str(exception)}],
                "log": traceback.format_exc(),
            }
        print(json.dumps(result), flush=True)


if __name__ == "__main__":
    main()
