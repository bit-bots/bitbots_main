"""Useful decorators for modifying the behavior of test functions as well as :class:`unittest.TestCase` classes."""
from typing import *
import os
import abc
from functools import wraps
from unittest import skip, SkipTest
from bitbots_test.test_case import TestCase


def tag(*tags: str) -> Callable:
    """
    Apply a set of tags to a subclass of :class:`unittest.TestCase`

    When multiple @tag decorators are defined, their tags will be merged together.

    This will enable the user to filter test execution to a subset of tests by specifying a list of tags
    to run as well as a list of tags to forbid. This is done defining the environment variable *TEST_TAGS*
    as a comma separated list.

    Behavior when specifying multiple tags on test execution:

    - When no tags are specified by the user, all tests will be run and this decorator has no effect at all.
    - When a simple list of tags is specified by the user, only tests which have this tag will be run.
    - When only forbidden tags are specified by the user, tests with these tags will not run but all other
      tests will.
    - When specifying a list of tags as well as forbidden tags, first tests will be filtered to the allowed
      tags and then again all tests with forbidden tags will be removed.
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

    def get_requested_tags() -> Set[str]:
        return set(t for t in os.environ.get("TEST_TAGS", "").split(",") if not t.startswith("!") and t != "")

    def get_forbidden_tags() -> Set[str]:
        return set(t[1:] for t in os.environ.get("TEST_TAGS", "").split(",") if t.startswith("!"))

    def is_test_requested(applied_tags: Set[str]) -> bool:
        matching_tags = [t for t in applied_tags if t in get_requested_tags()]
        return len(matching_tags) > 0

    def is_test_forbidden(applied_tags: Set[str]) -> bool:
        matching_tags = [t for t in applied_tags if t in get_forbidden_tags()]
        return len(matching_tags) > 0

    def conditional_exec(func: Callable, applied_tags: Set[str]) -> Any:
        """
        Conditionally execute the given callable but only if the set of applied_tags are specified to be run
        according to the rules described in the @tag docstring.

        :arg applied_tags: The set of tags which are applied to this TestCase
        """

        # When no tags are specified by the user, the function will simply be run
        if len(get_requested_tags()) == 0 and len(get_forbidden_tags()) == 0:
            return func()

        # When only a positive list of tags is specified by the user, the function will only be run
        # if one of the applied_tags is requested
        if len(get_requested_tags()) > 0 and len(get_forbidden_tags()) == 0:
            if is_test_requested(applied_tags):
                return func()
            else:
                raise SkipTest(f"none of the tags {applied_tags} are requested to be run")

        # When only forbidden tags are specified by the user, the function will run by default but not if
        # one of the applied_tags is forbidden
        if len(get_requested_tags()) == 0 and len(get_forbidden_tags()) > 0:
            if is_test_forbidden(applied_tags):
                raise SkipTest(f"one of the tags {applied_tags} are forbidden from running")
            else:
                return func()

        # When specifying a list of tags as well as forbidden tags,
        # first tests will be filtered to the allowed tags
        # and then again all tests with forbidden tags will be removed.
        if len(get_requested_tags()) > 0 and len(get_forbidden_tags()) > 0:
            if is_test_requested(applied_tags):
                if not is_test_forbidden(applied_tags):
                    return func
                raise SkipTest(f"one of the tags {applied_tags} are forbidden from running")
            raise SkipTest(f"none of the tags {applied_tags} are requested to be run")

    def decorator(cls: Type):
        # sanity checks
        if type(cls) != type:
            raise TypeError(f"@tag decorator can only be used on classes and not {type(cls)}")

        if not issubclass(cls, TestCase):
            raise ValueError(f"@tag decorator can only be used on TestCase subclasses and not {cls.__name__}")

        # store the tags in a class attribute so that multiple @tag decorators get merged
        if not hasattr(cls, "tags"):
            cls.tags = set(tags)
        else:
            cls.tags.update(tags)

        # wrap all test functions in this TestCase to only execute if the relevant tags are specified
        test_funcs = [t for t in dir(cls) if t.startswith("test_") and callable(getattr(cls, t))]
        for t in test_funcs:
            original_func = getattr(cls, t)

            @wraps(original_func)
            def new_func(*args, **kwargs):
                applied_tags = getattr(cls, "tags", set())
                return conditional_exec(lambda: original_func(*args, **kwargs), applied_tags)

            setattr(cls, t, new_func)

        return cls

    return decorator


class TestRestriction(abc.ABC):
    @abc.abstractmethod
    def should_execute(self) -> bool:
        pass

    def get_reason(self) -> str:
        return f"{type(self).__name__} restriction prevented test execution"


def restrict(restriction: TestRestriction):
    """
    Restrict the execution of this test function or :class:`unittest.TestCase` to a dynamically evaluated condition
    """

    def decorator(decorated: Union[Callable, Type]):
        if restriction.should_execute():
            return decorated
        else:
            return skip(restriction.get_reason())(decorated)

    return decorator


def error2failure(func: Callable) -> Callable:
    """
    Decorator which catches all exceptions thrown by the test and transforms them into `AssertionError` so that the
    test is not marked as having an error but as having failed.
    """
    @wraps(func)
    def result(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception as e:
            raise AssertionError(f"test caused an unhandled error: {e}") from e

    return result
