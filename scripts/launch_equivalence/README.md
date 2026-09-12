# Offline launch comparison

This disposable migration checker compares the resolved instructions produced by ROS launch and better-launch. Run it from the repository through the default Pixi environment. It reads Git snapshots, so uncommitted launch edits are excluded. No workspace build or robot connection is required; the Pixi launch libraries and the better-launch source in the migrated revision are required.

Start by inspecting the matrix:

```sh
pixi run -e default python scripts/launch_equivalence/verify.py \
  --base <old-ref> --head <migrated-ref> --plan
```

Check whether the current host permits the required containment before running experiments:

```sh
pixi run -e default python scripts/launch_equivalence/verify.py --check-sandbox
```

This checks Landlock and installs the seccomp filter in a disposable subprocess without importing launch files. A failed check reports the detected ABI or syscall error. Older kernels, disabled Landlock, and outer container policies can prevent containment. Use a host with the required Landlock support enabled and its syscalls permitted; there is no unsafe bypass. The check covers containment only, not launch dependencies or resources.

Use the entrypoint identifiers printed in that plan to select a smaller comparison:

```sh
pixi run -e default python scripts/launch_equivalence/verify.py \
  --base <old-ref> --head <migrated-ref> \
  --entry <package/launch-file> --domains <domains.json> --output <report-directory>
```

`--defaults-only` checks entrypoints with all arguments omitted. It is useful for finding missing resources before running a matrix, and is explicitly recorded as limited coverage. Without it, boolean arguments receive omitted, false and true inputs. Other arguments are tested only with omission until their domains are supplied. The plan lists these arguments under `default_only_arguments`. Include-only arguments and arguments constructed dynamically may require explicit entries in the domains file. There is no claim to enumerate arbitrary strings, file contents, integers, environments, or runtime events.

`--max-cases` refuses an oversized matrix before evaluation; it never truncates a run. `--timeout` bounds each worker response. Select entrypoints or supply narrower domains when needed. Unknown entrypoint selectors and malformed domains fail instead of yielding an empty success.

Choose a fresh output directory for each run. Nonempty report directories are not overwritten.

## Input domains

The domains file is JSON. Keys under `arguments` are entrypoint identifiers; values map launch argument names to lists of CLI strings or `null` for omission. Environment profiles map variable names to strings, or to `null` to unset them. Profiles are crossed with argument combinations.

```json
{
  "arguments": {
    "bitbots_bringup/teamplayer.launch.py": {
      "sim": [null, "false", "true"],
      "fieldname": [null, "<field-name>"]
    },
    "bitbots_bringup/mujoco_simulation.launch.py": {
      "num_robots": ["<robot-count>"],
      "robot_type": ["<robot-type>"]
    }
  },
  "environments": {
    "unset_robot": {
      "ROBOT_NAME": null,
      "ROBOCUP_ROBOT_ID": null,
      "ROS_DOMAIN_ID": null
    },
    "selected_robot": {
      "ROBOT_NAME": "<robot-name>",
      "ROS_DOMAIN_ID": "<domain-id>"
    }
  }
}
```

Replace placeholders with values appropriate for the resources being compared. For forwarded string switches, supply the empty string and boolean spellings as separate cases. The native decorator and include implementations perform the conversions; the checker does not coerce an included string into a boolean.

The inherited Pixi environment is shared by both sides. Robot identity and ROS domain variables are unset unless supplied by a profile. Reports record process environment changes relative to that common environment, rather than copying unrelated host variables. Use the same Pixi lockfile and profiles when reproducing a result.

## Reduction and caching

The checker merges omission with an explicit boolean default only when the old entrypoint has a literal, unconditional XML declaration before any actions and the new function has the identical literal boolean default. A conservative source check excludes entrypoints with raw CLI introspection or unfamiliar imports. ROS receives arguments through an include action, and Click binds the corresponding typed value. Conditional defaults, differing defaults, and non-boolean inputs are kept separate. Every reduction and both the original and reduced assignment counts are recorded in the plan. These reductions apply to the inspected argument-binding patterns; arbitrary Python introspection through external helpers is outside the supported model.

It exhaustively enumerates the resulting finite Cartesian product. It does not use pairwise sampling or stop testing an output after finding a discrepancy. Identical request results have a bounded cache. Process normalization is cached by the entire resolved command, environment delta, working directory, process policy, and parameter file contents. This reuses repeated leaf results without assuming argument or subtree independence. A generated temporary parameter filename does not invalidate a content-equivalent cache entry; changing the file contents does.

The tool does **not** yet perform general dependency-based subtree pruning. It still evaluates the orchestration for each reduced assignment. In particular, the process-cache counters measure reused normalization work, not skipped whole launch executions. Adding subtree pruning would require proving which shared state and effects a subtree reads and writes on both sides. Repeated observations alone are insufficient proof.

## What is compared

The ROS adapter uses native parsing, conditions, scoped launch configurations, substitutions, node expansion, and process preparation, stopping before process execution. The better-launch adapter uses the source decorator, include and grouping implementation, and command construction through the process-spawn boundary. Regular ROS files included by better-launch use the ROS adapter. These hooks intentionally depend on private APIs: incompatible versions produce unresolved cases.

Manifests retain executable identity, node names and namespaces, ordered remaps and application arguments, typed parameters and precedence, environment changes, working directories, shell mode, output destinations, respawn settings, lifecycle targets, and launch-defined order. Raw commands and parameter-source paths are retained for diagnosis but are excluded from semantic comparison. Missing native node names remain unspecified; the checker does not guess executable defaults from the migrated file. Node-specific parameter selectors without a known name are unresolved.

ROS component load requests are prepared without contacting their container. Wall-clock timestamps are fixed for reproducibility. Simple timers are evaluated without waiting, retaining their delay and cancellation policy. Nested launcher subprocesses are captured as boundaries and recursively evaluated with their arguments and environment. Generated package files are captured and removed between cases. Xacro and the welcome text read are evaluated in-process; other command substitutions are unresolved.

Unsupported actions, lifecycle event transitions, exit callbacks, nested timers, missing Python dependencies, missing resources, ambiguous parameter selectors and preparation failures prevent a clean equivalence verdict. A case may contain both useful differences and unresolved parts. Matching failures are never reported as equivalent. Launch logging and runtime scheduling are not a proof of robot behavior; the comparison covers the instructions described above.

Executable paths are symbolic and package resources come from the respective snapshot, with Pixi resources as the fallback for external packages. Launcher infrastructure uses the installed Pixi version. Source resources are exposed in temporary package directories; the checker does not validate that the package build would install all those files. Generated resources absent from the snapshots remain blockers rather than fabricated data.

## Safety

Launch evaluation runs in dedicated workers. Before any launch source is imported, Linux Landlock restricts writes to worker scratch storage and excludes device access. Seccomp denies process creation, networking, device control, and filesystem metadata mutation; Python audit hooks turn common attempts into explicit errors. The controller may start evaluator workers, but launch code cannot start child processes. Failure to establish containment stops evaluation, with no unsafe fallback. Linux Landlock and the system seccomp library are required; no dependency declarations are changed by the tool.

The worker uses the real launch libraries, resets their known state between cases, disables ROS graph discovery and GUI/spin behavior, and rejects unsupported operations. This is a migration checker for the inspected launch patterns, not a general interpreter for arbitrary side-effectful Python. Hardware execution and runtime integration testing remain separate activities.

## Reports and validation

- `plan.json`: revisions, entrypoints, domains, profiles, reductions and coverage counts.
- `cases.jsonl`: every evaluated case, status, and references to findings.
- `differences.diff`: each distinct field difference or blocker, with an example input and occurrence count.
- `summary.json`: result totals, cache statistics, scope and grouped findings.
- Finding JSON files: example inputs and a reference to the shared manifests under `examples/`, including source chains and diagnostic traces.

Finding deduplication changes presentation only: it does not suppress later experiments. Exit status is successful only when every evaluated case is equivalent; differences and unresolved cases have distinct non-success statuses. A successful result still applies only to the declared domains and supported launch instructions.

Workers must acknowledge successful containment before any cases are submitted. Worker startup failures, exits, protocol failures and timeouts abort the run immediately instead of becoming repeated per-case findings. An aborted report records `aborted`, `failure` and `completed_cases` in `summary.json`, and marks `differences.diff` as incomplete. Completed cases remain available; unevaluated cases are excluded from result counts.

Run the focused tests and format checks through Pixi:

```sh
pixi run -e default pytest -q scripts/launch_equivalence/tests
pixi run -e default ruff check scripts/launch_equivalence
pixi run -e default ruff format --check scripts/launch_equivalence
```
