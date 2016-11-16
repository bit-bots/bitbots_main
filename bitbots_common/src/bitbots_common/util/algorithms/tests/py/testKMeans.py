#-*- coding:utf-8 -*-
"""
TestKMeans
^^^^^^^^^^

.. moduleauthor:: sheepy <sheepy@informatik.uni-hamburg.de>

History:
* 5/12/14: Created (sheepy)

"""
import unittest
from bitbots.util.algorithms.kmeans import KMeans


class TestKMeans(unittest.TestCase):

    def test_k_means(self):

        data = [
            (0, 0),
            (5, 5)
        ]

        km = KMeans(data, [(0.1, -0.1), (4, 4)])

        erg = km.compute()

        c1 = erg[0]
        c2 = erg[1]

        self.assertAlmostEqual(0, c1[0])
        self.assertAlmostEqual(0, c1[1])

        self.assertAlmostEqual(5, c2[0])
        self.assertAlmostEqual(5, c2[1])