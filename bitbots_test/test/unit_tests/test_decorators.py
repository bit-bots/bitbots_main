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
    @given(st.sets(st.text(), min_size=1), st.sets(st.text()))
    def test_tagged_test_func_gets_run_if_tag_is_requested(
        self, test_tags: typing.Set[str], requested_tags: typing.Set[str]
    ):
        # setup
        test_func = Mock()
        requested_tags.add(list(test_tags)[0])

        # execution
        try:
            os.environ["TEST_TAGS"] = ",".join(requested_tags)
            tag(*test_tags)(test_func)()
        except ValueError:
            raise UnsatisfiedAssumption()
        except SkipTest:
            self.fail("Test was skipped although one its tags was specified to be run")

        # verification
        test_func.assert_called_once()

    @given(st.sets(st.text(), min_size=1), st.sets(st.text()))
    def test_tagged_test_func_does_not_get_run_if_tag_is_not_requested(
        self, test_tags: typing.Set[str], requested_tags: typing.Set[str]
    ):
        # setup
        test_func = Mock()
        requested_tags.difference_update(test_tags)

        # execution
        try:
            os.environ["TEST_TAGS"] = ",".join(requested_tags)
            tag(*test_tags)(test_func)()
        except ValueError:
            raise UnsatisfiedAssumption()
        except SkipTest:
            pass

        # verification
        test_func.assert_not_called()

    @given(st.sets(st.text(), min_size=1), st.sets(st.text()))
    def test_tagged_test_func_gets_run_if_tag_is_not_forbidden(
        self, test_tags: typing.Set[str], forbidden_tags: typing.Set[str]
    ):
        # setup
        test_func = Mock()
        forbidden_tags.difference_update(test_tags)

        # execution
        try:
            os.environ["TEST_TAGS"] = ",".join(
                [f"!{t}" for t in forbidden_tags] + list(test_tags)
            )
            tag(*test_tags)(test_func)()
        except ValueError:
            raise UnsatisfiedAssumption()
        except SkipTest:
            self.fail("Test was skipped although none of its tags were forbidden")

        # verification
        test_func.assert_called_once()

    @given(st.sets(st.text(), min_size=1), st.sets(st.text()))
    def test_tagged_test_func_does_not_get_run_if_tag_is_forbidden(
        self, test_tags: typing.Set[str], forbidden_tags: typing.Set[str]
    ):
        # setup
        test_func = Mock()
        forbidden_tags.add(list(test_tags)[0])

        # execution
        try:
            os.environ["TEST_TAGS"] = ",".join(
                [f"!{t}" for t in forbidden_tags] + list(test_tags)
            )
            tag(*test_tags)(test_func)()
        except ValueError:
            raise UnsatisfiedAssumption()
        except SkipTest:
            pass

        # verification
        test_func.assert_not_called()


if __name__ == "__main__":
    rosunit.unitrun("udp_bridge", TagTestCase.__name__, TagTestCase)
