import os

from deploy.misc import print_debug, print_success, print_warning
from deploy.tasks.abstract_task import AbstractTask
from fabric import Group, GroupResult, Result
from git import Repo


class CheckLocalMainRepoTask(AbstractTask):
    def __init__(self) -> None:
        """
        Task to check the local main repository.
        Displays the current commit hash (with a friendly name) to compare with other people in a hurry.
        Also displays warnings and requires confirmation before proceeding when:
        - The local main repository is dirty (uncommitted changes).
        - The local main repository is not on the main branch.
        - The local main repository is ahead or behind of the remote main repository.
        """
        super().__init__()
        self.repo: Repo = self._get_repo()
        self.warning_reasons: list[str] = []

    def _get_repo(self) -> Repo:
        """
        Get the bitbots_main repository.
        We can locate it by knowing that we are contained in the repository.

        :return: The main repository.
        """
        repo_path: str = os.path.join(os.path.dirname(__file__), "..", "..", "..")  # bitbots_main/scripts/deploy/tasks
        repo: Repo = Repo(repo_path)
        assert not repo.bare, "The bitbots_main repository is bare. Is it a real GIT-repository?"
        return repo

    def _run(self, connections: Group) -> GroupResult:
        """
        Get the current commit hash and display warnings if necessary.
        """
        commit_hash: str = self._get_commit_hash()
        commit_name: str = self._get_friendly_commit_name(commit_hash)
        if self._check_dirty():
            self.warning_reasons.append("The local main repository is dirty (uncommitted changes).")
        if not self._check_branch_main():
            self.warning_reasons.append("The local main repository is not on the main branch.")
        if self._check_ahead_behind():
            self.warning_reasons.append("The local main repository is ahead or behind of the remote main repository.")

        if not self.warning_reasons:
            print_success(
                f"Current commit: [bold]{commit_name}[default] ({commit_hash[:8]})\nYour local main repository is clean and up-to-date."
            )
            group_result = GroupResult()
            group_result._successes = {connection: Result() for connection in connections}
        else:
            warnings: str = "\n".join(self.warning_reasons)
            print_warning(
                f"Current commit: [bold]{commit_name}[default] ({commit_hash[:8]})\n\n"
                "Warnings:\n"
                f"{warnings}\n\n"
                "Please check the warnings and decide if you want to proceed!"
            )
            answer: str = input(r"Do you want to proceed? \[y/N] ").lower()
            proceed = answer and "y" in answer[0]
            group_result = GroupResult()
            if proceed:
                print_debug("Proceeding despite warnings because of user choice.")
                group_result._successes = {}
                for connection in connections:
                    result = Result(connection=connection)
                    result.exited = 0
                    result.stdout = warnings
                    group_result._successes[connection] = result
            else:
                print_debug("Aborting because of user choice.")
                group_result._failures = {}
                for connection in connections:
                    result = Result(connection=connection)
                    result.exited = 1
                    result.stdout = warnings
                    group_result._failures[connection] = result
        print_debug(group_result.failed)
        return group_result

    def _get_commit_hash(self) -> str:
        """
        Get the current commit hash of the main repository.

        :return: The commit hash.
        """
        print_debug("Getting current commit hash of the main repository.")
        commit_hash: str = self.repo.head.commit.hexsha
        print_debug(f"Got commit hash: '{commit_hash}'.")
        return commit_hash

    def _get_friendly_commit_name(self, hash: str) -> str:
        """
        Get a human readable commit name.
        The hash is used to generate a name using friendly adjectives and animals.

        :param hash: The commit hash.
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

        friendly_name: str = f"{friendly_adjectives[int(hash, 16) % len(friendly_adjectives)]} {friendly_animals[int(hash, 16) % len(friendly_animals)]}"
        print_debug(f"Generated friendly commit name: '{friendly_name}'.")
        return friendly_name

    def _check_dirty(self) -> bool:
        """
        Check if the main repository is dirty (uncommitted changes).

        :return: True if the main repository is dirty.
        """
        print_debug("Checking if the main repository is dirty.")
        dirty: bool = self.repo.is_dirty(untracked_files=True)
        print_debug(f"Main repository is dirty?: {dirty}.")
        return dirty

    def _check_branch_main(self) -> bool:
        """
        Check if the main repository is on the main branch.

        :return: True if the main repository is on the main branch.
        """
        print_debug("Checking if the main repository is on the main branch.")
        try:
            active_branch = self.repo.active_branch
        except TypeError:
            return False
        print_debug(f"Main repository is on branch: '{active_branch.name}'.")
        return active_branch.name == "main"

    def _check_ahead_behind(self) -> bool:
        """
        Check if the main repository is ahead or behind of the remote main repository.

        :return: True if the main repository is ahead or behind of the remote main repository.
        """
        print_debug("Checking if the main repository is ahead or behind of the remote main repository.")
        try:
            print_debug("Trying to get remote repository.")
            remote = self.repo.remote()
        except ValueError:
            print_debug("No remote repository found.")
            self.warning_reasons.append(
                "No remote repository found. Could not check if the local main repository is ahead or behind of the remote main repository."
            )
            return True

        print_debug("Fetching remote repository.")
        remote.fetch(kill_after_timeout=5)

        print_debug("Checking if behind: Comparing local and remote main repository.")
        ahead = False
        for _ in self.repo.iter_commits(f"main..{remote.name}/main"):
            ahead = True
        if ahead:
            self.warning_reasons.append("The local main repository is ahead of the remote main repository.")

        print_debug("Checking if ahead: Comparing remote main and local repository.")
        behind = False
        for _ in self.repo.iter_commits(f"{remote.name}/main..main"):
            behind = True
        if behind:
            self.warning_reasons.append("The local main repository is behind of the remote main repository.")

        return ahead or behind
