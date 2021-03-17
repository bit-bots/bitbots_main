"""Useful decorators for modifying the behavior of test functions as well as :class:`unittest.TestCase` classes."""
from unittest import skipIf, skipUnless
import typing
import sys
import os


def tag(*tags: str) -> typing.Callable:
    """Apply a set of tags to a test function or subclass of :class:`unittest.TestCase`

    This will result in the singular test function or TestCase to only execute when one of the given tags is requested
    to be run by the user.

    The user can specify a set of tags to run by defining the environment variable *TEST_TAGS* as a comma separated
    list of tags to run. Additionally a tag prefixed by '!' will forbid tests with that tag to run.

    Combination of tags:

    - When multiple tags are given to this decorator, requesting one of them will result the test to be run.
    - When this decorator is applied multiple times, all decorators must allow execution in order for the test to be run.
    """
    for t in tags:
        if t.startswith("!"):
            raise ValueError(f"tag {t} starts with forbidden character '!'")
        if "," in t:
            raise ValueError(f"tag {t} contains forbidden character ','")
        if t == "":
            raise ValueError(f"an empty string is not a valid tag")

    def is_test_requested() -> bool:
        requested_tags = [
            t
            for t in os.environ.get("TEST_TAGS", "").split(",")
            if not t.startswith("!")
        ]
        matching_tags = [t for t in tags if t in requested_tags]
        return len(matching_tags) > 0

    def is_test_forbidden() -> bool:
        forbidden_tags = [
            t[1:]
            for t in os.environ.get("TEST_TAGS", "").split(",")
            if t.startswith("!")
        ]
        matching_tags = [t for t in tags if t in forbidden_tags]
        return len(matching_tags) > 0

    def func_decorator(func: typing.Callable) -> typing.Callable:
        return skipIf(
            is_test_forbidden(), f"one of the tags {tags} is forbidden to be run"
        )(
            skipUnless(
                is_test_requested(), f"none of the tags {tags} are requested to be run"
            )(func)
        )

    def class_decorator(cls: typing.Type) -> typing.Type:
        print(f"tags: {list(tags)}, class decorator", file=sys.stderr)
        return cls

    def decorator(decorated: typing.Union[typing.Callable, typing.Type]):
        if type(decorated) == type:
            return class_decorator(decorated)
        else:
            return func_decorator(decorated)

    return decorator
