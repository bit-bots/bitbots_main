import rosunit
import os
import typing
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
    def _getTestTestCase() -> typing.Tuple[typing.Type[TestCase], Mock]:
        test_func = Mock()
        test_func.configure_mock(**{"_test_tags_": set()})

        class TestTestCase(TestCase):
            pass

        TestTestCase.test_func = test_func

        return TestTestCase, test_func

    @staticmethod
    def _initDecorator(test_tags: typing.Set[str], user_tags: typing.Set[str]):
        try:
            os.environ["TEST_TAGS"] = ",".join(user_tags)
            return tag(*test_tags)
        except ValueError as e:
            raise UnsatisfiedAssumption()

    def _run(
        self,
        test_tags: typing.Set[str],
        user_tags: typing.Set[str],
        should_have_run: bool,
    ):
        TestTestCase, test_func = self._getTestTestCase()
        decorator = self._initDecorator(test_tags, set(user_tags))

        with self.subTest("decorated function"):
            # execution
            try:
                decorator(test_func)()
            except SkipTest:
                pass

            # verification
            if should_have_run:
                test_func.assert_called()
            else:
                test_func.assert_not_called()

        test_func.reset_mock()
        with self.subTest("decorated class"):
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
        self, test_tags: typing.Set[str]
    ):
        self._run(test_tags, set(), True)

    @given(st.sets(CustomStrategies.tags, min_size=1), st.sets(CustomStrategies.tags))
    def test_tagged_test_gets_run_if_tag_is_requested(
        self, test_tags: typing.Set[str], requested_tags: typing.Set[str]
    ):
        requested_tags.add(list(test_tags)[0])
        self._run(test_tags, requested_tags, True)

    @given(
        st.sets(CustomStrategies.tags, min_size=1),
        st.sets(CustomStrategies.tags, min_size=1),
    )
    def test_tagged_test_does_not_get_run_if_tag_is_not_requested(
        self, test_tags: typing.Set[str], requested_tags: typing.Set[str]
    ):
        requested_tags.difference_update(test_tags)
        assume(len(requested_tags) > 0)

        self._run(test_tags, requested_tags, False)

    @given(
        st.sets(CustomStrategies.tags, min_size=1),
        st.sets(CustomStrategies.tags, min_size=1),
    )
    def test_tagged_test_gets_run_if_tag_is_not_forbidden_and_no_tags_are_requested(
        self, test_tags: typing.Set[str], forbidden_tags: typing.Set[str]
    ):
        forbidden_tags.difference_update(test_tags)
        assume(len(forbidden_tags) > 0)

        self._run(test_tags, set([f"!{t}" for t in forbidden_tags]), True)

    @given(st.sets(CustomStrategies.tags, min_size=1), st.sets(CustomStrategies.tags))
    def test_tagged_test_does_not_get_run_if_tag_is_forbidden_and_no_tags_are_requested(
        self, test_tags: typing.Set[str], forbidden_tags: typing.Set[str]
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
        test_tags: typing.Set[str],
        requested_tags: typing.Set[str],
        forbidden_tags: typing.Set[str],
    ):
        forbidden_tags.add(list(test_tags)[0])
        requested_tags.add(list(test_tags)[0])

        self._run(
            test_tags,
            set([f"!{t}" for t in forbidden_tags] + list(requested_tags)),
            False,
        )

    @given(st.sets(CustomStrategies.tags, min_size=1))
    def test_tags_get_merged(self, tags: typing.Set[str]):
        # setup
        TestTestCase, test_func = self._getTestTestCase()

        with self.subTest("decorated function"):
            # execution
            decorated = test_func
            for t in tags:
                decorated = tag(t)(decorated)
                pass

            # verification
            self.assertEqual(test_func._test_tags_, tags)

        test_func.reset_mock()
        with self.subTest("decorated class"):
            # execution
            decorated = TestTestCase
            for t in tags:
                decorated = tag(t)(TestTestCase)

            # verification
            self.assertEqual(test_func._test_tags_, tags)



if __name__ == "__main__":
    rosunit.unitrun("udp_bridge", TagTestCase.__name__, TagTestCase)
