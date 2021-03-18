import rosunit
import os
import typing
from unittest.mock import Mock
from unittest.case import SkipTest
from hypothesis import given, strategies as st
from hypothesis.errors import UnsatisfiedAssumption
from bitbots_test.test_case import TestCase
from bitbots_test.decorators import tag


class TagTestCase(TestCase):
    @staticmethod
    def _getTestTestCase() -> typing.Tuple[typing.Type[TestCase], Mock]:
        test_func = Mock()

        class TestTestCase(TestCase):
            pass

        TestTestCase.test_func = test_func

        return TestTestCase, test_func

    @staticmethod
    def _initDecorator(test_tags: typing.Set[str], user_tags: typing.Set[str]):
        try:
            os.environ["TEST_TAGS"] = ",".join(user_tags)
            return tag(*test_tags)
        except ValueError:
            raise UnsatisfiedAssumption()

    @given(st.sets(st.text(), min_size=1), st.sets(st.text()))
    def test_tagged_test_gets_run_if_tag_is_requested(
        self, test_tags: typing.Set[str], requested_tags: typing.Set[str]
    ):
        # setup
        requested_tags.add(list(test_tags)[0])
        TestTestCase, test_func = self._getTestTestCase()
        decorator = self._initDecorator(test_tags, requested_tags)

        with self.subTest("decorated function"):
            # execution
            try:
                decorator(test_func)()
            except SkipTest:
                pass

            # verification
            test_func.assert_called_once()

        test_func.reset_mock()
        with self.subTest("decorated class"):
            # execution
            try:
                decorator(TestTestCase)().test_func()
            except SkipTest:
                pass

            # verification
            test_func.assert_called_once()

    @given(st.sets(st.text(), min_size=1), st.sets(st.text()))
    def test_tagged_test_does_not_get_run_if_tag_is_not_requested(
        self, test_tags: typing.Set[str], requested_tags: typing.Set[str]
    ):
        # setup
        requested_tags.difference_update(test_tags)
        TestTestCase, test_func = self._getTestTestCase()
        decorator = self._initDecorator(test_tags, requested_tags)

        with self.subTest("decorated function"):
            # execution
            try:
                decorator(test_func)()
            except SkipTest:
                pass

            # verification
            test_func.assert_not_called()

        test_func.reset_mock()
        with self.subTest("decorated class"):
            # execution
            try:
                decorator(TestTestCase)().test_func()
            except SkipTest:
                pass

            # verification
            test_func.assert_not_called()

    @given(st.sets(st.text(), min_size=1), st.sets(st.text()))
    def test_tagged_test_gets_run_if_tag_is_not_forbidden(
        self, test_tags: typing.Set[str], forbidden_tags: typing.Set[str]
    ):
        # setup
        forbidden_tags.difference_update(test_tags)
        TestTestCase, test_func = self._getTestTestCase()
        decorator = self._initDecorator(
            test_tags, set([f"!{t}" for t in forbidden_tags] + list(test_tags))
        )

        with self.subTest("decorated function"):
            # execution
            try:
                decorator(test_func)()
            except SkipTest:
                pass

            # verification
            test_func.assert_called_once()

        test_func.reset_mock()
        with self.subTest("decorated class"):
            # execution
            try:
                decorator(TestTestCase)().test_func()
            except SkipTest as e:
                print(e)
                pass

            # verification
            test_func.assert_called_once()

    @given(st.sets(st.text(), min_size=1), st.sets(st.text()))
    def test_tagged_test_func_does_not_get_run_if_tag_is_forbidden(
        self, test_tags: typing.Set[str], forbidden_tags: typing.Set[str]
    ):
        # setup
        forbidden_tags.add(list(test_tags)[0])
        TestTestCase, test_func = self._getTestTestCase()
        decorator = self._initDecorator(
            test_tags, set([f"!{t}" for t in forbidden_tags])
        )

        with self.subTest("decorated function"):
            # execution
            try:
                decorator(test_func)()
            except SkipTest:
                pass

            # verification
            test_func.assert_not_called()

        test_func.reset_mock()
        with self.subTest("decorated class"):
            # execution
            try:
                decorator(TestTestCase)().test_func()
            except SkipTest:
                pass

            # verification
            test_func.assert_not_called()


if __name__ == "__main__":
    rosunit.unitrun("udp_bridge", TagTestCase.__name__, TagTestCase)
