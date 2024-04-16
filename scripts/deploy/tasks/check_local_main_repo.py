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
                f"Current commit: {commit_name} ({commit_hash})\nYour local main repository is clean and up-to-date."
            )
            group_result = GroupResult()
            group_result.succeeded = {connection: Result(connection) for connection in connections}
        else:
            print_warning(
                f"Current commit: {commit_name} ({commit_hash})\n"
                f"Warnings: {', '.join(self.warning_reasons)}\n"
                "Please check the warnings and decide if you want to proceed!"
            )
            proceed = "y" in input("Do you want to proceed? [y/N] ").lower()
            group_result = GroupResult()
            if proceed:
                group_result.succeeded = {}
                for connection in connections:
                    result = Result(connection=connection)
                    result.exited = 0
                    result.stdout = self.warning_reasons
                    group_result.succeeded[connection] = result
            else:
                group_result.failed = {}
                for connection in connections:
                    result = Result(connection=connection)
                    result.exited = 1
                    result.stdout = self.warning_reasons
                    group_result.failed[connection] = result
            return GroupResult()

    def _get_commit_hash(self) -> str:
        """
        Get the current commit hash of the main repository.

        :return: The commit hash.
        """
        print_debug("Getting current commit hash of the main repository.")
        return self.repo.head.commit.hexsha

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

        return f"{friendly_adjectives[int(hash, 16) % len(friendly_adjectives)]} {friendly_animals[int(hash, 16) % len(friendly_animals)]}"

    def _check_dirty(self) -> bool:
        """
        Check if the main repository is dirty (uncommitted changes).

        :return: True if the main repository is dirty.
        """
        print_debug("Checking if the main repository is dirty.")
        return self.repo.is_dirty(untracked_files=True)

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
        return active_branch.name == "main"

    def _check_ahead_behind(self) -> bool:
        """
        Check if the main repository is ahead or behind of the remote main repository.

        :return: True if the main repository is ahead or behind of the remote main repository.
        """
        print_debug("Checking if the main repository is ahead or behind of the remote main repository.")
        try:
            remote = self.repo.remote()
        except ValueError:
            self.warning_reasons.append(
                "No remote repository found. Could not check if the local main repository is ahead or behind of the remote main repository."
            )
            return True

        remote.fetch(kill_after_timeout=5)
        ahead, behind = self.repo.iter_commits(f"main..{remote.name}/main")
        print(ahead, behind)  # TODO Remove
        return ahead == behind
