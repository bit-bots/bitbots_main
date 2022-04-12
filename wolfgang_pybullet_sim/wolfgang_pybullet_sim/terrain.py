import pybullet as p
import pybullet_data as pd
import math
import time

import random


class Terrain:
    def __init__(self, max_height=0.01, size=(25, 25), scale=(0.1, 0.1), clear_center=False):
        self.id = -1
        self.collision_shape = -1

        self.scale = scale
        self.rows = int(size[0] / scale[0])
        self.cols = int(size[1] / scale[1])
        self.heightfield_data = [0] * self.rows * self.cols
        self.clear_center_size = 2
        self.clear_center = clear_center

        self.randomize(max_height)

    def randomize(self, max_height):
        if max_height > 1:
            #hacky way to make different types of terrain just based on the max_height parameter
            max_height = max_height - 1
            for j in range(int(self.cols / 2)):
                for i in range(int(self.rows / 2)):
                    # create a quadratic shape with random height
                    height = random.uniform(0, max_height)
                    self.heightfield_data[2 * i + 2 * j * self.rows] = height
                    self.heightfield_data[2 * i + 1 + 2 * j * self.rows] = height
                    self.heightfield_data[2 * i + (2 * j + 1) * self.rows] = height
                    self.heightfield_data[2 * i + 1 + (2 * j + 1) * self.rows] = height
        else:
            for j in range(int(self.cols)):
                for i in range(int(self.rows)):
                    self.heightfield_data[i + j * self.rows] = random.uniform(0, max_height)

        if self.clear_center:
            # make sure the center is flat so the robot does not get stuck there on resetting
            for j in range(int(self.cols / 4) - self.clear_center_size, int(self.cols / 4) + self.clear_center_size):
                for i in range(int(self.rows / 4) - self.clear_center_size,
                               int(self.rows / 4) + self.clear_center_size):
                    self.heightfield_data[2 * i + 2 * j * self.rows] = 0
                    self.heightfield_data[2 * i + 1 + 2 * j * self.rows] = 0
                    self.heightfield_data[2 * i + (2 * j + 1) * self.rows] = 0
                    self.heightfield_data[2 * i + 1 + (2 * j + 1) * self.rows] = 0

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
            p.changeVisualShape(self.id, -1, rgbaColor=[1, 1, 1, 1])
        else:
            p.createCollisionShape(
                shapeType=p.GEOM_HEIGHTFIELD,
                flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
                meshScale=[self.scale[0], self.scale[1], 1.0],
                heightfieldTextureScaling=(self.rows - 1) / 2,
                heightfieldData=self.heightfield_data,
                numHeightfieldRows=self.rows,
                numHeightfieldColumns=self.cols,
                replaceHeightfieldIndex=self.collision_shape)
