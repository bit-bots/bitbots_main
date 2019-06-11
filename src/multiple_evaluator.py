import math
import os
import cv2
import time
import yaml
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
from evaluation_data_loader import DataLoader
from worker import VisualCompass


class BinaryEvaluator(object):
    def __init__(self, dimensions, angle_steps):
        self.dimensions = dimensions
        self.angle_steps = angle_steps

        dirname = os.path.dirname(__file__)
        relative_config_path = "../config/config.yaml"
        config_path = os.path.join(dirname, relative_config_path)

        with open(config_path, 'r') as stream:
            config = yaml.load(stream)

        relative_data_path = config['evaluation_data']
        self.data_path = os.path.join(dirname, relative_data_path)

        self.visualization_path = os.path.join(dirname, config["visualization"])

        self.loader = DataLoader(self.data_path, self.dimensions, self.angle_steps)

        #config['compass_type'] = 'multiple'
        
        self.sample_count = 2 if config['compass_type'] == 'binary' else config['compass_multiple_sample_count']
        self.vc = VisualCompass(config)
    
    def show_img(self, image):
        cv2.imshow("Record", image)
        k = cv2.waitKey(1)

    def evaluate(self):
        self.setTruth()
        self.evaluateAllImages()

    def setTruth(self):
        for i in range(self.sample_count):
            angle = float(i) / self.sample_count * math.radians(360)
            angle = (angle + math.radians(90)) % math.radians(360)
            image = self.loader.getImage(4, 3, angle)
            self.vc.set_truth(angle, image)
            self.show_img(image)
        cv2.destroyAllWindows()

    def evaluateAllImages(self):


        for i in range(16):

            plt.figure(i)
            self.evaluateDirection(math.pi/8*i, None)
            print(i)
            print("done")
            filename = "" + str(i) + ".png"
            plt.savefig(os.path.join(self.visualization_path, filename))
    
    def debug_image_callback(self, debug_image):
        return
        self.show_img(debug_image)
        time.sleep(0.5)

    def evaluateDirection(self, step, ax):
        ax = plt.subplot(1, 2, 1)

        U = np.zeros(self.dimensions)
        V = np.zeros(self.dimensions)
        C = np.zeros(self.dimensions)
        for i in range(self.dimensions[0]):
            for j in range(self.dimensions[1]):
                image = self.loader.getImage(i, j, step)
                angle, confidence = self.vc.process_image(image)
                z = np.exp(1j*angle)
                U[i, j] = np.real(z) * confidence
                V[i, j] = np.imag(z) * confidence
                C[i, j] = confidence

        Q = ax.quiver(U, V, C, pivot='mid')
        ax.axis('equal')
        ax.axis('off')

        plt.subplot(1, 2, 2)
        plt.imshow(self.loader.getImage(4, 3, step))


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

    def float_mod(self,  number, modulo):
        return number - modulo * math.floor(number / modulo)

if __name__ == "__main__":
    evaluator = BinaryEvaluator((10,7), 16)
    evaluator.evaluate()
    # evaluator.calcUV()
