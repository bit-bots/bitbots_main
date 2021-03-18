"""Useful decorators for modifying the behavior of test functions as well as :class:`unittest.TestCase` classes."""
import typing
import os
from unittest import skipIf, skipUnless, skip
from bitbots_test.test_case import TestCase


def tag(*tags: str) -> typing.Callable:
    """
    Apply a set of tags to a test function or subclass of :class:`unittest.TestCase`

    When multiple @tag decorators are defined, their tags will be merged together.

    This will enable the user to filter test execution to a subset of tests by specifying a list of tags to run as well
    as a list of tags to forbid. This is done defining the environment variable *TEST_TAGS* as a comma separated list.

    Behavior when specifying tags on test execution:

    - When no tags are specified by the user, all tests will be run and this decorator has no effect at all.
    - When only a simple list of tags is specified by the user, only tests which have this tag will be run.
    - When only forbidden tags are specified by the user, tests with these tags will not run but all other tests will.
    - When specifying a list of tags as well as forbidden tags, first tests will be filtered to the allowed tags and then
        again all tests with forbidden tags will be removed.
    """
    # sanity checks
    if len(tags) == 0:
        raise ValueError("at least one tag must be specified")
    for t in tags:
        if not isinstance(t, str):
            raise TypeError(f"tags must be of type str and not {type(t).__name__}")
        if t.startswith("!"):
            raise ValueError(f"tag {t} starts with forbidden character '!'")
        if "," in t:
            raise ValueError(f"tag {t} contains forbidden character ','")
        if t == "":
            raise ValueError(f"an empty string is not a valid tag")

    def get_requested_tags() -> typing.Set[str]:
        return set(
            [
                t
                for t in os.environ.get("TEST_TAGS", "").split(",")
                if not t.startswith("!") and t != ""
            ]
        )

    def get_forbidden_tags() -> typing.Set[str]:
        return set(
            [
                t[1:]
                for t in os.environ.get("TEST_TAGS", "").split(",")
                if t.startswith("!")
            ]
        )

    def is_test_requested(func: typing.Callable) -> bool:
        matching_tags = [t for t in func._test_tags_ if t in get_requested_tags()]
        return len(matching_tags) > 0

    def is_test_forbidden(func: typing.Callable) -> bool:
        matching_tags = [t for t in func._test_tags_ if t in get_forbidden_tags()]
        return len(matching_tags) > 0

    def func_decorator(func: typing.Callable) -> typing.Callable:
        if not hasattr(func, "_test_tags_"):
            func._test_tags_ = set(tags)
        else:
            func._test_tags_ = func._test_tags_.union(set(tags))

        # When no tags are specified by the user, all tests will be run and this decorator has no effect at all
        if len(get_requested_tags()) == 0 and len(get_forbidden_tags()) == 0:
            return func

        # When only a simple list of tags is specified by the user, only tests which have this tag will be run
        if len(get_requested_tags()) > 0 and len(get_forbidden_tags()) == 0:
            if is_test_requested(func):
                return func
            else:
                return skip(f"none of the tags {func._test_tags_} are requested to be run")(func)

        # When only forbidden tags are specified by the user, tests with these tags will not run but all other tests will
        if len(get_requested_tags()) == 0 and len(get_forbidden_tags()) > 0:
            if is_test_forbidden(func):
                return skip(f"one of the tags {func._test_tags_} are forbidden from running")(func)
            else:
                return func

        # When specifying a list of tags as well as forbidden tags, first tests will be filtered to the allowed tags
        # and then again all tests with forbidden tags will be removed.
        if len(get_requested_tags()) > 0 and len(get_forbidden_tags()) > 0:
            if is_test_requested(func):
                if not is_test_forbidden(func):
                    return func
                return skip(f"one of the tags {func._test_tags_} are forbidden from running")(func)
            return skip(f"none of the tags {func._test_tags_} are requested to be run")(func)

    def class_decorator(cls: typing.Type) -> typing.Type:
        if not issubclass(cls, TestCase):
            raise ValueError(
                f"@tag decorator can only be used on TestCase subclasses and not {cls.__name__}"
            )

        # manually apply @tag() decorator to all test functions in this TestCase
        test_funcs = [
            t for t in dir(cls) if t.startswith("test_") and callable(getattr(cls, t))
        ]
        for t in test_funcs:
            original_func = getattr(cls, t)
            decorated_func = tag(*tags)(original_func)
            setattr(cls, t, decorated_func)

        return cls

    def decorator(decorated: typing.Union[typing.Callable, typing.Type]):
        if type(decorated) == type:
            return class_decorator(decorated)
        else:
            return func_decorator(decorated)

    return decorator
