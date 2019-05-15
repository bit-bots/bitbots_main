import numpy as np
import math

class tranformer:
    def __init__(self, camera_matrix):
        self.invmatrix = np.linalg.inv(camera_matrix)

    def calckeyangle(self, angle, point):
        vector = np.array([point[0], point[1], 1])
        local_angle = math.atan(self.invmatrix.dot(vector)[0])
        return self.float_mod(local_angle + angle, math.tau)

    def float_mod(self,  number, modulo):
        return number - modulo * math.floor(number / modulo)
