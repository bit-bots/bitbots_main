from datetime import datetime, timedelta

from bitbots_test.test_case import TestCase


class GeneralAssertionMixinTestCase(TestCase):
    def test_assertion_grace_period(self):
        with self.subTest("succeeding assert returns instantly"):
            # setup
            now = datetime.now()

            # execution
            self.with_assertion_grace_period(t=100, f=lambda: self.assertTrue(True))

            # verification
            self.assertTrue(now + timedelta(milliseconds=100) > datetime.now())

        with self.subTest("failing assert needs the grace period"):
            # setup
            now = datetime.now()

            # execution
            try:
                self.with_assertion_grace_period(t=100, f=lambda: self.assertTrue(False))
            except:
                pass

            # verification
            self.assertTrue(now + timedelta(milliseconds=100) < datetime.now())

    def test_assert_ranges(self):
        # assertInRange
        self.assertInRange(0, (0, 1))
        self.assertInRange(0, (-10, 10))
        self.assertInRange(0, (10, -10))
        self.assertRaises(AssertionError, lambda: self.assertInRange(0, (5, 10)))
        self.assertRaises(AssertionError, lambda: self.assertInRange(20, (5, 10)))
        self.assertRaises(AssertionError, lambda: self.assertInRange(0, (10, 5)))
        self.assertRaises(AssertionError, lambda: self.assertInRange(20, (10, 5)))

        # assertNotInRange
        self.assertNotInRange(0, (5, 10))
        self.assertNotInRange(20, (5, 10))
        self.assertNotInRange(0, (10, 5))
        self.assertNotInRange(20, (10, 5))
        self.assertRaises(AssertionError, lambda: self.assertNotInRange(0, (0, 1)))
        self.assertRaises(AssertionError, lambda: self.assertNotInRange(0, (-10, 10)))
        self.assertRaises(AssertionError, lambda: self.assertNotInRange(0, (10, -10)))


if __name__ == "__main__":
    from bitbots_test import run_unit_tests
    run_unit_tests(GeneralAssertionMixinTestCase)
