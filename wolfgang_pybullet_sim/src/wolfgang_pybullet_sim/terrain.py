import pybullet as p
import pybullet_data as pd
import math
import time

import random

random.seed(10)


class Terrain:
    def __init__(self, max_height=0.01, size=(10, 10), scale=(0.05, 0.05)):
        self.id = -1
        self.collision_shape = -1

        self.max_height = max_height
        self.scale = scale
        self.rows = int(size[0] / scale[0])
        self.cols = int(size[1] / scale[1])
        self.heightfield_data = [0] * self.rows * self.cols

        self.randomize()

    def randomize(self):

        for j in range(int(self.cols / 2)):
            for i in range(int(self.rows / 2)):
                # create a quadratic shape with random height
                height = random.uniform(0, self.max_height)
                self.heightfield_data[2 * i + 2 * j * self.rows] = height
                self.heightfield_data[2 * i + 1 + 2 * j * self.rows] = height
                self.heightfield_data[2 * i + (2 * j + 1) * self.rows] = height
                self.heightfield_data[2 * i + 1 + (2 * j + 1) * self.rows] = height

        # first time we can't replace, also add some visuals
        if self.collision_shape == -1:
            self.collision_shape = p.createCollisionShape(
                shapeType=p.GEOM_HEIGHTFIELD,
                meshScale=[self.scale[0], self.scale[1], 1.0],
                heightfieldTextureScaling=(self.rows - 1) / 2,
                heightfieldData=self.heightfield_data,
                numHeightfieldRows=self.rows,
                numHeightfieldColumns=self.cols)
            self.id = p.createMultiBody(0, self.collision_shape)
            p.resetBasePositionAndOrientation(self.id, [0, 0, 0.0], [0, 0, 0, 1])
            p.changeDynamics(self.id, -1, lateralFriction=1.0)
            p.changeVisualShape(self.id, -1, rgbaColor=[1, 1, 1, 1])
        else:
            p.createCollisionShape(
                shapeType=p.GEOM_HEIGHTFIELD,
                flags=0,
                meshScale=[self.scale[0], self.scale[1], 1.0],
                heightfieldTextureScaling=(self.rows - 1) / 2,
                heightfieldData=self.heightfield_data,
                numHeightfieldRows=self.rows,
                numHeightfieldColumns=self.cols,
                replaceHeightfieldIndex=self.collision_shape)
