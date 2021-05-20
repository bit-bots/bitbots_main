from pathlib import Path
import inspect
import rosunit
import rostest
from catkin_pkg.package import parse_package
from .test_case import TestCase


def _get_package_name(test_case: type) -> str:
    path = Path(inspect.getfile(test_case))

    while not path.samefile(path.root):
        if (path / "package.xml").exists():
            package = parse_package(path / "package.xml", warnings=[])
            return package.name

        path = path.parent

    raise Exception(f"Could not determine package name for TestCase {test_case}")


def run_unit_tests(*test_cases: type):
    """Run all the specified unit TestCases"""
    for i in test_cases:
        assert issubclass(i, TestCase), f"Test case {i} does not inherit from base bitbots TestCase"
        rosunit.unitrun(_get_package_name(i), i.__name__, i)


def run_rostests(*test_cases: type):
    """Run all the specified integration TestCases"""
    for i in test_cases:
        assert issubclass(i, TestCase), f"Test case {i} does not inherit from base bitbots TestCase"
        rostest.rosrun(_get_package_name(i), i.__name__, i)
