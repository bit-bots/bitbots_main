import json
import os
import threading
from hashlib import md5

from deploy.misc import be_quiet, hide_output, print_debug, print_info, print_success, print_warning
from deploy.tasks import INTERNET_TIMEOUT
from deploy.tasks.abstract_task import AbstractTask
from fabric import Connection, Group, GroupResult, Result
from git import Repo
from git.exc import GitCommandError
from invoke.exceptions import UnexpectedExit
from rich.console import Group as RichGroup
from rich.table import Table
from yaml import safe_load


class OurRepo(Repo):
    def __init__(self, path: str, specified_branch: str) -> None:
        """
        Custom class for a GIT-repository.

        :param path: The path to the repository.
        :param specified_branch: The branch this repository should be in (e.g. as defined in the workspace.repo file).
        """
        super().__init__(path)
        assert not self.bare, f"Repo {self}: Repository is bare. Is it a real GIT-repository?"

        self.specified_branch = specified_branch

        self.warnings: list[str] = []

    def check(self, do_fetch: True) -> list[str]:
        """
        Run all checks for the repository and return the occurred warnings.

        :param do_fetch: If True, run checks that fetch the remote, thus an Internet connection is required. Default is True.
        :return: The warnings.
        """
        self.check_dirty()
        self.check_branch()
        if do_fetch:
            self.check_ahead_behind()
        else:
            print_debug(f"Repo {self}: Skipping check for being ahead or behind of the remote repository.")
        return self.warnings

    def get_commit_hash(self) -> str:
        """
        Get the current commit hash of the repository.

        :return: The commit hash.
        """
        print_debug(f"Repo {self}: Getting current commit hash of the repository...")
        commit_hash: str = self.head.commit.hexsha
        print_debug(f"Repo {self}: Got commit hash: '{commit_hash}'.")
        return commit_hash

    def check_dirty(self) -> bool:
        """
        Check if the repository is dirty (uncommitted changes).

        :return: True if the repository is dirty.
        """
        print_debug(f"Repo {self}: Checking if the repository is dirty...")
        dirty: bool = self.is_dirty(untracked_files=True)
        if dirty:
            print_debug(f"Repo {self}: Repository is dirty (uncommitted changes)!")
            self.warnings.append("Repository is dirty (uncommitted changes)!")
        return dirty

    def check_branch(self) -> bool:
        """
        Check if the repository is on the specified branch.

        :return: True if the repository is on the specified branch.
        """
        print_debug(f"Repo {self}: Checking if the repository is on the specified branch '{self.specified_branch}'...")
        try:
            branch_is_correct: bool = self.active_branch.name == self.specified_branch
            print_debug(f"Repo {self}: Is on branch: '{self.active_branch.name}'.")
            if not branch_is_correct:
                print_debug(
                    f"Repo {self}: Is on the '{self.active_branch}' branch, but should be on '{self.specified_branch}'!"
                )
                self.warnings.append(
                    f"Is on the '{self.active_branch}' branch, but should be on '{self.specified_branch}'!"
                )
        except TypeError:
            return False
        return branch_is_correct

    def check_ahead_behind(self, timeout: float = 5) -> bool:
        """
        Check if the repository is ahead or behind of the remote repository.

        :param timeout: The timeout for the fetch command.
        :return: True if the repository is ahead or behind of the remote repository.
        """
        print_debug(f"Repo {self}: Checking if the repository is ahead or behind of the remote repository.")
        try:
            print_debug(f"Repo {self}: Trying to get remote repository.")
            remote = self.remote()
        except ValueError:
            print_debug(
                f"Repo {self}: No remote repository found. Could not check if the local repository is ahead or behind of the remote repository."
            )
            self.warnings.append(
                "No remote repository found. Could not check if the local repository is ahead or behind of the remote repository."
            )
            return True

        print_debug(f"Repo {self}: Fetching remote repository.")
        try:
            remote.fetch(kill_after_timeout=timeout, verbose=not be_quiet())
        except GitCommandError as e:
            err: str = e.stderr.strip()
            print_debug(
                f"Repo {self}: Fetching remote repository failed: {err}. Could not check if the repository is ahead or behind of the remote repository."
            )
            self.warnings.append(
                f"Fetching remote repository failed: {err}. Could not check if the repository is ahead or behind of the remote repository."
            )
            return True

        print_debug(f"Repo {self}: Checking if behind: Comparing local and remote repository.")
        behind = len(list(self.iter_commits(f"{self.specified_branch}..{remote.name}/{self.specified_branch}")))
        if behind:
            print_debug(
                f"Repo {self}: Your branch is behind the remote branch by {behind} commit{'s' if behind > 1 else ''}."
            )
            self.warnings.append(
                f"Your branch is behind the remote branch by {behind} commit{'s' if behind > 1 else ''}."
            )

        print_debug(f"Repo {self}: Checking if ahead: Comparing remote and local repository.")
        ahead = len(list(self.iter_commits(f"{remote.name}/{self.specified_branch}..{self.specified_branch}")))
        if ahead:
            print_debug(
                f"Repo {self}: Your branch is ahead of the remote branch by {ahead} commit{'s' if ahead > 1 else ''}."
            )
            self.warnings.append(
                f"Your branch is ahead of the remote branch by {ahead} commit{'s' if ahead > 1 else ''}."
            )

        return ahead or behind

    def __str__(self) -> str:
        return os.path.basename(os.path.dirname(self.git_dir))


class CheckReposTask(AbstractTask):
    def __init__(self, only_workspace_status: bool = False) -> None:
        """
        Task to check the local repositories (bitbots_main and others as specified in workspace.repos file).
        Displays the current commit status (with a friendly name) to compare with other people in a hurry.
        Also displays warnings and requires confirmation before proceeding when:
        - A repository is dirty (uncommitted changes).
        - A repository is not on the specified branch.
        - A repository is ahead or behind of the remote repository.

        :param only_workspace_status: If True, only collect the workspace commit hashes and skip the local workspace check.
        """
        super().__init__()
        self._show_status = False

        self.main_repo_path: str = os.path.join(
            os.path.dirname(__file__), "..", "..", ".."
        )  # bitbots_main/scripts/deploy/tasks

        self.results_file: str = os.path.join(
            self.main_repo_path, "src/bitbots_misc/bitbots_utils/config/", "workspace_status.json"
        )

        self.only_workspace_status: bool = only_workspace_status

        self.main_repo: OurRepo = OurRepo(self.main_repo_path, "main")
        self.repos: dict[str, OurRepo] = {"bitbots_main": self.main_repo, **self._get_workspace_repos()}

        self.commit_hashes: dict[str, str] = {}
        self.warnings: dict[str, list[str]] = {}

    def _get_workspace_repos(self) -> dict[str, OurRepo]:
        """
        Get the paths of the workspace repositories.

        :return: List of workspace repository paths.
        """
        result: dict[str, OurRepo] = {}
        with open(os.path.join(self.main_repo_path, "workspace.repos")) as file:
            workspace_repos: dict = safe_load(file)
            for path, repo in workspace_repos["repositories"].items():
                if repo["type"] == "git":
                    our_repo = OurRepo(os.path.join(self.main_repo_path, path), repo.get("version", "main"))
                    result[str(our_repo)] = our_repo
        return result

    def _github_available(self, connection: Connection) -> bool:
        """
        Check if GitHub is available.

        :return: True if GitHub is available.
        """
        github: str = "https://github.com"
        print_debug(f"Checking for internet connection to {github}")

        cmd = f"timeout --foreground {INTERNET_TIMEOUT:.2f} curl -sSLI {github}"
        print_debug(f"Calling {cmd}")
        available = False
        result: Result | None = None
        try:
            result = connection.local(cmd, hide=hide_output())
        except UnexpectedExit:
            pass
        if result is not None:
            available = result.ok
        print_debug(f"Internet connection to github.com is {'' if available else 'NOT'} available.")
        if not available:
            print_info("Internet connection is not available. We may skip some checks!")
        return available

    def _run(self, connections: Group) -> GroupResult:
        """
        For each repo, get the current commit hash and display warnings if necessary.
        Also write the current commit states to a file.

        :param connections: The connections to the targets.
        :return: The result of the check.
        """

        def success(connections: Group) -> GroupResult:
            group_result = GroupResult()
            group_result._successes = {connection: Result(connection=connection) for connection in connections}
            return group_result

        def failure(
            connections: Group,
            warnings: str,
            do_exit: bool,
        ) -> GroupResult:
            group_result = GroupResult()
            results = {
                connection: Result(connection=connection, exited=int(not do_exit), stdout=warnings)
                for connection in connections
            }
            if do_exit:
                group_result._failures = results
            else:
                group_result._successes = results
            return group_result

        workspace_hash, workspace_friendly_name = self._collect_workspace_hashes()
        if self.only_workspace_status:
            print_info(
                f"Current workspace hash: [bold]{workspace_friendly_name}[default] ({workspace_hash[:8]})\nSkipped workspace checks."
            )
            results = success(connections)
            return results

        # Is GitHub available?
        github_available: bool = self._github_available(connections[0])

        # Check all repositories and collect warnings with multiple threads
        threads: list[threading.Thread] = []
        for name, repo in self.repos.items():
            thread = threading.Thread(
                target=lambda name=name, repo=repo: self.warnings.update({name: repo.check(do_fetch=github_available)})
            )
            threads.append(thread)
            thread.start()
        for thread in threads:
            thread.join()

        # Display results
        results: GroupResult
        # Do we have any warnings? If not, all checks for all repos were successful
        if not any([*self.warnings.values()]):  # No warnings = Success
            print_success(
                f"Current workspace hash: [bold]{workspace_friendly_name}[default] ({workspace_hash[:8]})\nYour local workspace is clean and up-to-date."
            )
            results = success(connections)

        # Display warnings and ask for confirmation
        else:  # Warnings = Failure
            warning_table = Table()
            warning_table.add_column("Repository")
            warning_table.add_column("Commit")
            warning_table.add_column("Warnings")
            for repo_name, repo_warnings in sorted(self.warnings.items()):
                if not repo_warnings:
                    continue
                commit_hash: str = self.commit_hashes[repo_name]
                commit_name: str = self._get_friendly_name(commit_hash)
                warnings: str = ""
                for i, warning in enumerate(repo_warnings):
                    warnings += f"{i + 1}. {warning}\n"
                warning_table.add_row(repo_name, f"{commit_name} ({commit_hash[:8]})", warnings)
            print_warning(
                RichGroup(
                    f"Current workspace hash: [bold]{workspace_friendly_name}[default] ({workspace_hash[:8]})",
                    "Your local workspace is [bold][red]BAD!",
                    warning_table,
                    "Please check the warnings and decide if you want to proceed!",
                )
            )
            answer: str = input("Do you still want to proceed with the deployment? [y/N] ").lower()
            proceed = answer and "y" in answer[0]
            if proceed:
                print_debug("Proceeding despite warnings because of user choice.")
                results = failure(connections, warnings, do_exit=False)
            else:
                print_debug("Aborting because of user choice.")
                results = failure(connections, warnings, do_exit=True)
        return results

    def _collect_workspace_hashes(self) -> tuple[str, str]:
        """
        Collect commit hashes and generate the friendly commit names. Writes the results to a file.

        :return: The workspace hash and the friendly name of the workspace hash.
        """
        self.commit_hashes: dict[str, str] = {name: repo.get_commit_hash() for name, repo in self.repos.items()}
        workspace_hash: str = self._get_workspace_hash(self.commit_hashes)
        self.commit_hashes["__WORKSPACE__"] = workspace_hash
        workspace_friendly_name: str = self._get_friendly_name(workspace_hash)
        self._write_commits()
        return workspace_hash, workspace_friendly_name

    def _get_workspace_hash(self, commit_hashes: dict[str, str]) -> str:
        """
        Generate a hash for the workspace based on the commit hashes of the repositories using MD5.

        :param commit_hashes: The commit hashes of the repositories.
        :return: The hash of the workspace.
        """
        return md5("".join(commit_hashes.values()).encode()).hexdigest()

    def _get_friendly_name(self, hash: str) -> str:
        """
        Get a human readable name.
        The string is used to generate a name using friendly adjectives and animals.

        :param hash: The commit hash in hexadecimal characters.
        :return: The human readable commit name.
        """
        friendly_adjectives: list[str] = [
            "Adorable",
            "Beautiful",
            "Caring",
            "Charming",
            "Cheerful",
            "Clean",
            "Compassionate",
            "Content",
            "Delightful",
            "Elegant",
            "Enthusiastic",
            "Fancy",
            "Good-natured",
            "Gorgeous",
            "Gracious",
            "Handsome",
            "Heartwarming",
            "Incredible",
            "Joyful",
            "Kind",
            "Lovely",
            "Magnificent",
            "Nice",
            "Optimistic",
            "Outstanding",
            "Perfect",
            "Quick",
            "Smiling",
            "Sunny",
            "Superb",
            "Uplifting",
            "Vibrant",
            "Warmhearted",
            "Whimsical",
        ]

        friendly_animals: list[str] = [
            "Antelope",
            "Badger",
            "Bear",
            "Beaver",
            "Bison",
            "Buffalo",
            "Bull",
            "Camel",
            "Cheetah",
            "Coyote",
            "Deer",
            "Dolphin",
            "Eagle",
            "Elephant",
            "Fox",
            "Gazelle",
            "Giraffe",
            "Goat",
            "Hawk",
            "Horse",
            "Jaguar",
            "Kangaroo",
            "Leopard",
            "Lion",
            "Llama",
            "Moose",
            "Owl",
            "Panther",
            "Penguin",
            "Puma",
            "Raccoon",
            "Rhino",
            "Seal",
            "Shark",
            "Squirrel",
            "Tiger",
            "Walrus",
            "Whale",
            "Wolf",
            "Zebra",
        ]

        friendly_name: str = f"{friendly_adjectives[int(hash[: len(hash) // 2], 16) % len(friendly_adjectives)]} {friendly_animals[int(hash[len(hash) // 2 :], 16) % len(friendly_animals)]}"
        return friendly_name

    def _write_commits(self) -> None:
        """
        Write the current commit hashes and names to a file.
        """
        state: dict[str, dict[str, str]] = {
            name: {"hash": hash, "name": self._get_friendly_name(hash)} for name, hash in self.commit_hashes.items()
        }
        with open(self.results_file, "w") as file:
            json.dump(state, file, sort_keys=True)
