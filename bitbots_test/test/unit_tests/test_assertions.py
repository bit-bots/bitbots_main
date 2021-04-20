import rosunit
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


if __name__ == "__main__":
    rosunit.unitrun("bitbots_test", GeneralAssertionMixinTestCase.__name__, GeneralAssertionMixinTestCase)
