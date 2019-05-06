import math
import os
import cv2
import time
import yaml
import matplotlib.pyplot as plt
import numpy as np
from evaluation_data_loader import DataLoader
from worker import VisualCompass


class BinaryEvaluator(object):
    def __init__(self, data_path, dimensions, angle_steps):
        self.dimensions = dimensions
        self.angle_steps = angle_steps
        self.data_path = data_path
        
        self.loader = DataLoader(self.data_path, self.dimensions, self.angle_steps)

        dirname = os.path.dirname(__file__)
        relative_path = "../config/config.yaml"
        config_path = os.path.join(dirname, relative_path)

        with open(config_path, 'r') as stream:
            config = yaml.load(stream)

        config['compass_type'] = 'binary'
        
        self.vc = VisualCompass(config)
    
    def show_img(self, image):
        cv2.imshow("Record", image)
        k = cv2.waitKey(1)

    def evaluate(self):
        self.setTruth()
        self.evaluateAllImages()

    def setTruth(self):
        angle_1 = float(1)/4*2*math.pi
        angle_2 = float(3)/4*2*math.pi
        side_1_image = self.loader.getImage(4, 3, angle_1)
        side_2_image = self.loader.getImage(4, 3, angle_2)
        self.vc.set_truth(0, side_1_image)
        self.show_img(side_1_image)
        time.sleep(0.1)
        self.vc.set_truth(math.pi, side_2_image)
        self.show_img(side_2_image)
        time.sleep(0.1)
    
    def debug_image_callback(self, debug_image):
        self.show_img(debug_image)
        time.sleep(0.5)
        return

    def evaluateAllImages(self):
        confidences = list()
        fail = 0.0
        unsave = 0.0
        for row in range(self.dimensions[0]):
            confidences.append(list())
            for checkpoint in range(self.dimensions[1]):
                confidences[row].append(list())
                for angle in [4,12]: #,12]:# range(self.angle_steps):
                    image = self.loader.getImage(row, checkpoint, float(angle)/16*2*math.pi)
                    # self.show_img(image)
                    ground_truth = float(angle - 4)/16*2*math.pi
                    print(ground_truth)
                    compass_result = self.vc.process_image(image)#, debugCB=self.debug_image_callback)
                    confidence = compass_result[1]
                    print(compass_result[0])
                    if (abs(ground_truth - compass_result[0]) > 0.0001 and compass_result[1] > 0.5):
                        print("Bad detection", compass_result[0], ground_truth)
                        fail += 1
                        confidence = 0.0
                    if compass_result[1] <= 0.5:
                        # self.show_img(image)
                        unsave += 1
                        confidence = 0.0
                        print("Confidence too low")
                    confidences[row][checkpoint].append(confidence)
        count = (self.dimensions[0] * self.dimensions[1]) * 2
        print(confidences)
        self.plot_confidence(confidences,0)
        self.plot_confidence(confidences,1)
        plt.show()
        print("Fail {}%".format(fail/count*100))
        print("Unsave {}%".format(unsave/count*100))

    def plot_confidence(self, confidences, side):
        a = np.zeros((10, 7))
        plt.subplot(2, 1, side + 1)
        for row_index, row in enumerate(confidences):
            for checkpoint_index, checkpoint in enumerate(row):
                a[row_index, checkpoint_index] = float(checkpoint[side])
        plt.imshow(a, cmap='hot', interpolation='nearest')

if __name__ == "__main__":
    evaluator = BinaryEvaluator("/home/florian/Projekt/bitbots/FieldData/Field_1/", (10,7), 16)
    evaluator.evaluate()