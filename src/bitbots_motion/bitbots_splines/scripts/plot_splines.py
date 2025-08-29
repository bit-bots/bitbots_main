#!/usr/bin/env python3

import collections

import numpy as np
from matplotlib import pyplot as plt


def poly_coefficients(x, coeffs):
    """Returns a polynomial for ``x`` values for the ``coeffs`` provided.

    The coefficients must be in ascending order (``x**0`` to ``x**o``).
    """
    o = len(coeffs)
    # print('This is a polynomial of order' + str(o))
    y = 0
    for i in range(o):
        y += coeffs[i] * x**i
    return y


def read_spline_data_from_file(filename):
    d = {}
    with open(filename) as file:
        for line in file:
            words = line.split()
            name = words[0].replace("'", "")
            i = 1
            polies = []
            while i < len(words):
                min = float(words[i])
                max = float(words[i + 1])
                nr_coef = int(words[i + 2])
                coefs = []
                for j in range(0, nr_coef):
                    coefs.append(float(words[i + j + 3]))
                polies.append((min, max, nr_coef, coefs))
                i += 3 + nr_coef
            d[name] = polies
    print(d)
    print("\n\n")
    return d


data = read_spline_data_from_file("/tmp/spline_export")
odered_data = collections.OrderedDict(sorted(data.items()))
position = 1
for name in odered_data:
    if ("foot_" not in name) and ("trunk_" not in name):
        continue

    plt.subplot(4, 3, position)
    for poly in data[name]:
        min = poly[0]
        max = poly[1]
        nr_coef = poly[2]
        coeffs = poly[3]
        x = np.linspace(min, max)

        plt.plot(x, poly_coefficients(x, coeffs))
    plt.title(name)
    position += 1

plt.show()
