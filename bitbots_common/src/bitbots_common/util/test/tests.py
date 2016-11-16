#!/usr/bin/env python
#-*-encoding:utf-8-*-
"""

.. moduleauthor:: Timon Giese<timon.giese@bit-bots.de>

History:
    2013-12-16: Test erstellt

"""
import unittest
from bitbots.util.mcanalyse import MotorConnectionAnalyser


class TestMotorConnectionAnalyser(unittest.TestCase):
    """ Testet den MotorConnectionAnalyser
    """

    def setUp(self):
        self.analyser = MotorConnectionAnalyser()

    def test_find_connection_error(self):
        """ Testet das normale auffinden der Fehler
        """

        #right leg missing (error between M10 and M8)
        motors = [10, 12, 14, 16, 18]

        self.assertEqual(self.analyser.find_connection_error(motors), [10],
                         'got wrong error list')

        motors = [16, 18, 3, 5]

        errors = set(self.analyser.find_connection_error(motors))
        solution = set([16, 3])

        self.assertSetEqual(errors, solution, "got wrong errors %s and not %s" % (errors, solution))

        motors = []
        for i in range(1, 21):
            motors.append(i)
        errors = self.analyser.find_connection_error(motors)
        solution = [1, 2, 7, 8, 19]

        self.assertEqual(errors, solution, "got wrong errors %s and not %s" % (errors, solution))

    def test_get_error_message(self):
        """ Testet den String-Format-Code auf absturzsicherheit
        """
        try:
            motors = []
            self.analyser.get_error_message(motors)
            motors = [10, 12, 14, 16, 18]
            self.analyser.get_error_message(motors)
            motors = [16, 18, 5, 3]
            self.analyser.get_error_message(motors)
            motors = [1]
            self.analyser.get_error_message(motors)
            motors = [5]
            self.analyser.get_error_message(motors)
            for i in range(1, 21):
                motors.append(i)
            self.analyser.get_error_message(motors)
        except Exception as e:
            self.fail("Got error Message: %s" % e)

    def test_robustness(self):
        """Robustheitstest
        """
        motors = []
        self.analyser.get_error_message(motors)
        motors = ["10", "zwölf", 'kekse', 'bananen', "drei_eier"]
if __name__ == '__main__':
    unittest.main()
