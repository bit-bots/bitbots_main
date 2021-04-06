"""Known test restriction so that common restriction can easily be reused in :func:`bitbots_test.decorators.restrict`"""
import os
from bitbots_test.decorators import TestRestriction, restrict


class OnRobot(TestRestriction):
    def should_execute(self) -> bool:
        return os.environ.get("IS_ROBOT", "") != ""

    def get_reason(self) -> str:
        return "this test is only executed on robots"


class NotOnRobot(TestRestriction):
    def should_execute(self) -> bool:
        return os.environ.get("IS_ROBOT", "") == ""

    def get_reason(self) -> str:
        return "this test is not executed on robots"
