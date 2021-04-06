import rosunit
import os
from typing import *
from unittest.mock import Mock
from unittest.case import SkipTest
from hypothesis import given, assume, strategies as st, example
from hypothesis.errors import UnsatisfiedAssumption
from bitbots_test.test_case import TestCase
from bitbots_test.decorators import tag


class CustomStrategies:
    tags = st.text(min_size=1).filter(lambda t: "," not in t and not t.startswith("!"))


class TagTestCase(TestCase):
    @staticmethod
    def _getTestTestCase() -> Tuple[Type[TestCase], Mock]:
        """
        Create a `TestCase` subclass with a mock test function named `test_func`
        """
        test_func = Mock()

        class TestTestCase(TestCase):
            pass

        TestTestCase.test_func = test_func

        return TestTestCase, test_func

    @staticmethod
    def _initDecorator(test_tags: Set[str], user_tags: Set[str]) -> Callable:
        """
        Setup a decorator with `test_tags` and configure the environment variable with `user_tags`
        """
        try:
            os.environ["TEST_TAGS"] = ",".join(user_tags)
            return tag(*test_tags)
        except ValueError:
            raise UnsatisfiedAssumption()

    def _run(
        self,
        test_tags: Set[str],
        user_tags: Set[str],
        should_have_run: bool,
    ):
        TestTestCase, test_func = self._getTestTestCase()
        decorator = self._initDecorator(test_tags, set(user_tags))

        # execution
        try:
            decorator(TestTestCase)().test_func()
        except SkipTest:
            pass

        # verification
        if should_have_run:
            test_func.assert_called()
        else:
            test_func.assert_not_called()

    @given(st.sets(CustomStrategies.tags, min_size=1))
    def test_tagged_test_gets_run_if_no_tags_are_requested(
        self, test_tags: Set[str]
    ):
        self._run(test_tags, set(), True)

    @given(st.sets(CustomStrategies.tags, min_size=1), st.sets(CustomStrategies.tags))
    def test_tagged_test_gets_run_if_tag_is_requested(
        self, test_tags: Set[str], requested_tags: Set[str]
    ):
        requested_tags.add(list(test_tags)[0])
        self._run(test_tags, requested_tags, True)

    @given(
        st.sets(CustomStrategies.tags, min_size=1),
        st.sets(CustomStrategies.tags, min_size=1),
    )
    def test_tagged_test_does_not_get_run_if_tag_is_not_requested(
        self, test_tags: Set[str], requested_tags: Set[str]
    ):
        requested_tags.difference_update(test_tags)
        assume(len(requested_tags) > 0)

        self._run(test_tags, requested_tags, False)

    @given(
        st.sets(CustomStrategies.tags, min_size=1),
        st.sets(CustomStrategies.tags, min_size=1),
    )
    def test_tagged_test_gets_run_if_tag_is_not_forbidden_and_no_tags_are_requested(
        self, test_tags: Set[str], forbidden_tags: Set[str]
    ):
        forbidden_tags.difference_update(test_tags)
        assume(len(forbidden_tags) > 0)

        self._run(test_tags, set([f"!{t}" for t in forbidden_tags]), True)

    @given(st.sets(CustomStrategies.tags, min_size=1), st.sets(CustomStrategies.tags))
    def test_tagged_test_does_not_get_run_if_tag_is_forbidden_and_no_tags_are_requested(
        self, test_tags: Set[str], forbidden_tags: Set[str]
    ):
        forbidden_tags.add(list(test_tags)[0])

        self._run(test_tags, set([f"!{t}" for t in forbidden_tags]), False)

    @given(
        st.sets(CustomStrategies.tags, min_size=1),
        st.sets(CustomStrategies.tags),
        st.sets(CustomStrategies.tags),
    )
    def test_tagged_test_does_not_get_run_if_tag_is_forbidden_and_requested(
        self,
        test_tags: Set[str],
        requested_tags: Set[str],
        forbidden_tags: Set[str],
    ):
        forbidden_tags.add(list(test_tags)[0])
        requested_tags.add(list(test_tags)[0])

        self._run(
            test_tags,
            set([f"!{t}" for t in forbidden_tags] + list(requested_tags)),
            False,
        )

    @given(st.sets(CustomStrategies.tags, min_size=1))
    def test_tags_get_merged(self, tags: Set[str]):
        # setup
        #try:
        #    os.environ["TEST_TAGS"] = list(tags)[-1]
        #except ValueError:
        #    raise UnsatisfiedAssumption()
        TestTestCase, test_func = self._getTestTestCase()

        # execution
        decorated = TestTestCase
        for t in tags:
            decorated = tag(t)(decorated)

        # verification
        self.assertEqual(set(TestTestCase.tags), set(tags))


if __name__ == "__main__":
    rosunit.unitrun("udp_bridge", TagTestCase.__name__, TagTestCase)
