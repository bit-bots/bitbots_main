class Polynom:
    def __init__(self):
        self.coefs = []


def pos(x, coefs):
    xx = 1.0
    val = 0.0
    for i in range(0, len(coefs)):
        val += xx * coefs[i]
        xx *= x

    return val


def vel(x, coefs):
    xx = 1.0
    val = 0.0
    for i in range(1, len(coefs)):
        val += i * xx * coefs[i]
        xx *= x

    return val


def acc(x, coefs):
    xx = 1.0
    val = 0.0
    for i in range(2, len(coefs)):
        val += (i - 1) * i * xx * coefs[i]
        xx *= x

    return val


def jerk(x, coefs):
    xx = 1.0
    val = 0.0
    for i in range(3, len(coefs)):
        val += (i - 2) * (i - 1) * i * xx * coefs[i]
        xx *= x

    return val
