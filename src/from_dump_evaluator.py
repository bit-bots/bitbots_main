import math
import os
import cv2
import time
import pickle
import yaml
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
from evaluation_data_loader import DataLoader
from worker import VisualCompass


class Evaluator(object):
    def __init__(self):

        self.dimensions = (10, 7)
        dirname = os.path.dirname(__file__)

        relative_config_path = "../config/config.yaml"
        config_path = os.path.join(dirname, relative_config_path)

        with open(config_path, 'r') as stream:
            config = yaml.load(stream)

        pickle_path = os.path.join(dirname, "datadump.pickle")
        with open(pickle_path, 'rb') as stream:
            self.data = pickle.load(stream)

        relative_data_path = config['evaluation_data']
        self.image_path = os.path.join(dirname, relative_data_path)
        
        self.loader = DataLoader(self.image_path, self.dimensions, 16)





    def evaluateAllImages(self, compass, matcher, samples):
        self.data

        for i in range(16):
            self.evaluateDirection(math.pi/8*i, None)
            print(i)
            print("done")
            plt.show()

    def evaluateDirection(self, angle, results):
        ax = plt.subplot(1, 2, 1)

        results = filter(lambda entry: math.isclose(entry["truth_angle"], angle, .1), results)

        U = np.zeros(self.dimensions)
        V = np.zeros(self.dimensions)
        C = np.zeros(self.dimensions)
        for result in results:
            angle = result["angle"]
            confidence = result["confidence"]
            x = result["x"]
            y = result["y"]
            z = np.exp(1j*angle)
            U[x, y] = np.real(z) * confidence
            V[x, y] = np.imag(z) * confidence
            C[x, y] = confidence

        Q = ax.quiver(U, V, C, pivot='mid')
        ax.axis('equal')
        ax.axis('off')

        plt.subplot(1, 2, 2)
        plt.imshow(self.loader.getImage(4, 3, angle))


    def plot_confidence(self, confidences, side):
        a = np.zeros((10, 7))
        for row_index, row in enumerate(confidences):
            for checkpoint_index, checkpoint in enumerate(row):
                a[row_index, checkpoint_index] = float(checkpoint[side])
        plt.imshow(a, cmap='hot', interpolation='nearest')

    def calcUV(self):

        fig, ax = plt.subplots()
        Q = ax.quiver(np.ones((10, 7)), np.zeros((10, 7)), pivot='mid', units='inches')
        plt.show()

if __name__ == "__main__":
    evaluator = Evaluator((10,7), 16)
    evaluator.evaluate()
