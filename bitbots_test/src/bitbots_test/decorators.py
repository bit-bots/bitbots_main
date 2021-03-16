"""Useful decorators for modifying the behavior of test functions as well as :class:`unittest.TestCase` classes."""
from unittest import skipUnless
import typing
import sys
import os


def tag(*tags: str) -> typing.Callable:
    """Apply a set of tags to a test function or subclass of :class:`unittest.TestCase`

    This will result in the singular test function or TestCase to only execute when one of the given tags is requested
    to be run by the user.

    Combination of tags:

    - When multiple tags are given to this decorator, requesting one of them will result the test to be run.
    - When this decorator is applied multiple times, all decorators must allow execution in order for the test to be run.
    """
    def should_test_run() -> bool:
        requested_tags = os.environ.get("TEST_TAGS", "").split(",")
        matching_tags = [t for t in tags if t in requested_tags]
        return len(matching_tags) > 0

    def func_decorator(func: typing.Callable) -> typing.Callable:
        return skipUnless(should_test_run(), f"none of the tags {tags} are requested to be run")(func)

    def class_decorator(cls: typing.Type) -> typing.Type:
        print(f"tags: {list(tags)}, class decorator", file=sys.stderr)
        return cls

    def decorator(decorated: typing.Union[typing.Callable, typing.Type]):
        if type(decorated) == type:
            return class_decorator(decorated)
        else:
            return func_decorator(decorated)

    return decorator
