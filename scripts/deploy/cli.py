from __future__ import annotations

import argparse
import asyncio
import json
import sys
from pathlib import Path

from deploy.artifacts import ArtifactServer, ArtifactStore, source_address_for
from deploy.models import DEFAULT_COMPONENTS, DesiredState, RobotStatus, StepState
from deploy.profiles import ProfileStore
from deploy.reconcile import DeploymentSupervisor


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Interactive Bit-Bots deployment supervisor")
    parser.add_argument("--user", default="nvidia")
    parser.add_argument("--workspace", default="~/bitbots/bitbots_main")
    parser.add_argument("--retry-interval", type=float, default=2.0)
    parser.add_argument("--artifact-bind", help="Wired LAN address for the artifact server")
    subparsers = parser.add_subparsers(dest="command")

    apply_parser = subparsers.add_parser("apply", help="Apply desired state without the TUI")
    add_target_arguments(apply_parser)
    apply_parser.add_argument("--match", default="lab")
    apply_parser.add_argument("--ball")
    apply_parser.add_argument("--component", action="append")
    apply_parser.add_argument("--no-sync", action="store_true")
    apply_parser.add_argument("--no-build", action="store_true")
    apply_parser.add_argument("--no-launch", action="store_true")
    apply_parser.add_argument("--clean", action="store_true")
    apply_parser.add_argument("--package", action="append", default=[])

    status_parser = subparsers.add_parser("status", help="Show robot and teamplayer status")
    add_target_arguments(status_parser)
    status_parser.add_argument("--json", action="store_true")

    cache_parser = subparsers.add_parser("cache", help="Manage offline environment artifacts")
    cache_subparsers = cache_parser.add_subparsers(dest="cache_command", required=True)
    cache_subparsers.add_parser("prepare", help="Create the linux-aarch64 robot artifact")
    return parser


def add_target_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("targets", nargs="*", help="Robot names, IP addresses, or ALL")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    workspace = Path(__file__).resolve().parents[2]
    deploy_dir = Path(__file__).resolve().parent
    profiles = ProfileStore.load(deploy_dir)
    store = ArtifactStore(workspace)

    if args.command == "cache":
        artifact = store.prepare()
        print(f"Prepared {artifact.path}")
        print(f"key: {artifact.key}")
        print(f"sha256: {artifact.sha256}")
        return 0

    targets = profiles.resolve_targets(
        getattr(args, "targets", []),
        args.user,
        args.workspace,
    )
    artifact = store.get()
    if args.command == "apply" and artifact is None:
        print(
            "No matching offline robot environment artifact. "
            "Run `pixi run deploy cache prepare` while Internet access is available.",
            file=sys.stderr,
        )
        return 2
    server = None
    if artifact is not None and args.command != "status":
        bind = args.artifact_bind or source_address_for(targets[0].host)
        server = ArtifactServer(artifact, bind=bind)
        server.start()
    supervisor = DeploymentSupervisor(
        targets,
        workspace,
        artifact,
        server,
        args.retry_interval,
    )
    try:
        if args.command == "apply":
            return asyncio.run(run_apply(args, profiles, supervisor))
        if args.command == "status":
            return asyncio.run(run_status(args, supervisor))

        from deploy.tui import DeployApp

        DeployApp(profiles, supervisor, default_match="lab").run()
        return 0
    finally:
        if server is not None:
            server.stop()


async def run_apply(
    args: argparse.Namespace,
    profiles: ProfileStore,
    supervisor: DeploymentSupervisor,
) -> int:
    match = profiles.matches[args.match]
    ball = profiles.balls[args.ball or match.ball]
    desired = DesiredState(
        generation=1,
        match=match,
        ball=ball,
        components=frozenset(args.component) if args.component else DEFAULT_COMPONENTS,
        sync_source=not args.no_sync,
        build=not args.no_build,
        build_packages=tuple(args.package),
        clean_build=args.clean,
        launch=not args.no_launch,
    )
    supervisor.start()
    supervisor.apply(list(supervisor.controllers), desired)
    try:
        while True:
            statuses = [controller.status for controller in supervisor.controllers.values()]
            render_batch_status(statuses)
            if all(status.applied_generation == desired.generation for status in statuses):
                return 0
            if any(report.state == StepState.FAILED for status in statuses for report in status.steps.values()):
                return 1
            await asyncio.sleep(0.25)
    finally:
        await supervisor.close()


async def run_status(args: argparse.Namespace, supervisor: DeploymentSupervisor) -> int:
    supervisor.start()
    try:
        await asyncio.gather(*(controller.refresh_status() for controller in supervisor.controllers.values()))
        statuses = [controller.status for controller in supervisor.controllers.values()]
        if args.json:
            print(
                json.dumps(
                    [
                        {
                            "name": status.target.name,
                            "host": status.target.host,
                            "connected": status.connected,
                            "teamplayer": status.teamplayer,
                            "logs": status.logs,
                        }
                        for status in statuses
                    ],
                    indent=2,
                )
            )
        else:
            render_batch_status(statuses)
        return 0 if all(status.connected for status in statuses) else 1
    finally:
        await supervisor.close()


def render_batch_status(statuses: list[RobotStatus]) -> None:
    print("\033[2J\033[H", end="")
    for status in statuses:
        active = next(
            (
                (step.value, report)
                for step, report in status.steps.items()
                if report.state in {StepState.RUNNING, StepState.FAILED, StepState.WAITING}
            ),
            None,
        )
        detail = f"{active[0]}: {active[1].summary}" if active else "idle"
        print(
            f"{status.target.name:<8} "
            f"{'online ' if status.connected else 'offline'} "
            f"{status.applied_generation}/{status.generation} {detail}"
        )


if __name__ == "__main__":
    raise SystemExit(main())
