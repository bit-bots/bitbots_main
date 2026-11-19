# Contributing

This guide describes how we work with git and GitHub in this repository. For
setup, coding style, and testing, see the
[development documentation](src/bitbots_misc/bitbots_docs/docs/manual/development/).

## Branches

- Branch off `main` and open a pull request back into `main`. Do not push to
  `main` directly.
- Name branches `<type>/<short-description>` in lowercase kebab-case, using a
  [commit type](#types), for example `fix/head-mover-camera-offset` or
  `feat/penalty-kick-behavior`.
- Keep one topic per branch. Unrelated fixes, cleanups, and formatter churn go
  into their own branch.
- Merged branches are deleted automatically.

## Titles and commit messages

We follow [Conventional Commits](https://www.conventionalcommits.org):

```text
<type>(<scope>): <description>
```

The same format applies to commit subjects, PR titles, and issue titles.

- **PR titles** matter most. The PR title becomes the commit subject on `main`,
  so it is the one that ends up in the history.
- **Issue titles** name the change that would resolve the issue, for example
  `fix(motion): robot falls when kicking while turning` for a bug or
  `feat(behavior): add penalty kick behavior` for a feature request.
  Questions use `docs`, because the answer usually belongs in the
  documentation.
- The **description** is short, lowercase, and in imperative mood, for example
  `fix(motion): stop head mover from assuming an offset camera`.
- The **scope** is the area the change is about. Omit it for changes that span
  the whole repository, for example `ci: cache pixi environments`. If a change
  touches several areas, use the main one.
- Mark a **breaking change** with `!` after the scope, for example
  `feat(msgs)!: rename ball position field`. Explain the migration in the body.
- Use the commit body to explain *why* a change was needed, not what the diff
  already shows.

A [bot](.github/workflows/conventions.yml) comments on issues and PRs whose
title or branch name does not follow these conventions, using the types and
scopes from the tables below. The comment disappears once the problem is fixed.

### Types

| Type       | Use for                                          |
| ---------- | ------------------------------------------------ |
| `feat`     | New functionality or behavior                    |
| `fix`      | Bug fixes                                        |
| `docs`     | Documentation only                               |
| `style`    | Formatting or whitespace, no code change         |
| `refactor` | Restructuring without behavior change            |
| `perf`     | Performance improvements                         |
| `test`     | Adding or fixing tests                           |
| `build`    | Build system, Pixi environment, or dependencies  |
| `ci`       | CI configuration                                 |
| `chore`    | Maintenance that fits nowhere else               |
| `revert`   | Reverting an earlier commit                      |

### Scopes

Use one of these scopes, or a package name when it is more precise, for example
`bitbots_hcm`.

| Scope          | Area                                                  |
| -------------- | ----------------------------------------------------- |
| `behavior`     | Body behavior and blackboard                          |
| `motion`       | Walking, kicking, animations, HCM, head mover         |
| `navigation`   | Path planning, localization, odometry                 |
| `vision`       | Image processing and object detection                 |
| `world-model`  | Ball and robot filtering                              |
| `team-comm`    | Team communication                                    |
| `sim`          | Simulation                                            |
| `msgs`         | Messages, services, and actions                       |
| `robot`        | Robot descriptions, URDFs, robot-specific config      |
| `hardware`     | Hardware interfaces, drivers, and sensors             |
| `deploy`       | Deployment tooling                                    |
| `tooling`      | Developer tools, scripts, and teleoperation           |
| `docs`         | The documentation package                             |
| `deps`         | Dependency updates                                    |

## Pull requests

- Open a draft PR as soon as you push a branch, even if the work is clearly
  still in progress. Drafts make ongoing work visible to the team, keep it from
  being forgotten, avoid duplicate effort, and invite early feedback. Mark the
  PR ready for review once it is done.
- Fill in the [PR template](.github/pull_request_template.md):
  - **Summary**: one or two sentences on what the PR does.
  - **Proposed changes**: what changed and why it is necessary.
  - **Related issues**: link issues. Use `Closes #123` or `Fixes #123` so the
    issue is closed on merge.
  - **Checklist**: tick only what you actually did. Strike through
    (`~~item~~`) items that do not apply instead of leaving them unchecked
    without comment. State it explicitly if the change was not tested on a
    robot.
- Request a review from people familiar with the affected code. At least one
  approval is required to merge, and CI (build, tests, and code style checks)
  must pass.
- Resolve review comments by changing the code or replying, then re-request
  review.

## Issues

- Use the issue templates for bug reports, feature requests, and questions. A
  bug report needs expected behavior, actual behavior, and steps to reproduce.
- Search for existing issues before opening a new one.
- Work that you cannot finish now belongs in an issue, including follow-up work
  discovered during a PR.
- Assign yourself to an issue when you start working on it.

## Labels

The area of an issue or PR is its title [scope](#scopes), so there are no area
labels. Labels are only used for:

- **Kind** (issues): `bug`, `enhancement`, or `question`. The issue templates
  set these automatically, so you never add them by hand.
- **Optional**: `good first issue` or `help wanted`.
- **Resolution** (when closing without a fix): `duplicate`, `invalid`, or
  `wontfix`.

Dependency labels are set by Dependabot. See the
[label list](https://github.com/bit-bots/bitbots_main/labels) for all labels.

## Projects and triage

Issues and PRs are added to the
[Software project](https://github.com/orgs/bit-bots/projects/24) automatically
and start in the **New** column. New means *not triaged yet*, so nothing should
stay there.

Triage your own issues and PRs right after opening them. Anyone on the team may
triage items others left in New.

1. Check that the title follows the [title format](#titles-and-commit-messages).
2. Set **Priority**. Set **Size** if you can estimate the effort.
3. Move the item out of New:
   - **Backlog**: accepted, but not planned yet or not specified well enough to
     start.
   - **Ready**: clear enough that someone can pick it up.
   - **In progress**: someone is working on it. Assign that person.
   - **In review**: a PR that is ready for review.
4. If an item belongs to another team, move it to that team's project instead,
   for example Robot or Infrastructure.

Keep the status current while you work on an item. Close duplicates and
invalid issues with the matching label instead of triaging them.
