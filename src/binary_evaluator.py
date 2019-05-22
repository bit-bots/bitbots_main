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
        
        self.loader = DataLoader(self.data_path, self.dimensions, self.angle_steps)

        config['compass_type'] = 'multiple'
        
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
            image = self.loader.getImage(4, 3, self.float_mod(angle + math.radians(90), math.radians(360)))
            self.vc.set_truth(angle, image)
            self.show_img(image)
            time.sleep(0.5)
        cv2.destroyAllWindows()
    
    def debug_image_callback(self, debug_image):
        return
        self.show_img(debug_image)
        time.sleep(0.5)

    def evaluateAllImages(self):
        confidences = list()
        fail = 0.0
        unsave = 0.5
        confidence_threshold = 0.4
        for row in range(self.dimensions[0]):
            confidences.append(list())
            for checkpoint in range(self.dimensions[1]):
                confidences[row].append(list())
                fails = list()
                for angle in [4,12]: #,12]:# range(self.angle_steps):
                    image = self.loader.getImage(row, checkpoint, float(angle)/16*2*math.pi)
                    # self.show_img(image)
                    ground_truth = float(angle - 4)/16*2*math.pi
                    compass_result = self.vc.process_image(image, debugCB=self.debug_image_callback)
                    confidence = compass_result[1]
                    fail_val = 0.0
                    if (abs(ground_truth - compass_result[0]) > 0.0001 and compass_result[1] > confidence_threshold):
                        print("Bad detection", compass_result[0], ground_truth)
                        fail += 1
                        fail_val = 1.0
                    if compass_result[1] <= confidence_threshold:
                        if abs(ground_truth - compass_result[0]) > 0.0001:
                            print("Filtered false positive")
                        # self.show_img(image)
                        unsave += 1
                        print("Confidence too low. Value: {}".format(confidence))
                        confidence = 0.0
                    confidences[row][checkpoint].append(confidence)
                    fails.append(fail_val)
                for fail_obj in fails:
                    confidences[row][checkpoint].append(fail_obj)
        count = (self.dimensions[0] * self.dimensions[1]) * 2
        plt.title('Binary evaluation')
        plt.subplot(2, 2, 1)
        plt.title('Confidence values torwards goal 1')
        self.plot_confidence(confidences,0)
        plt.subplot(2, 2, 2)
        plt.title('Confidence values torwards goal 2')
        self.plot_confidence(confidences,1)
        plt.subplot(2, 2, 3)
        plt.title('False positives torwards goal 1')
        self.plot_confidence(confidences,2)
        plt.subplot(2, 2, 4)
        plt.title('False positives torwards goal 2')
        self.plot_confidence(confidences,3)
        plt.show()
        print("Fail {}%".format(fail/count*100))
        print("Unsave {}%".format(unsave/count*100))

    def plot_confidence(self, confidences, side):
        a = np.zeros((10, 7))
        for row_index, row in enumerate(confidences):
            for checkpoint_index, checkpoint in enumerate(row):
                a[row_index, checkpoint_index] = float(checkpoint[side])
        plt.imshow(a, cmap='hot', interpolation='nearest')

    def float_mod(self,  number, modulo):
        return number - modulo * math.floor(number / modulo)

if __name__ == "__main__":
    evaluator = BinaryEvaluator((10,7), 16)
    evaluator.evaluate()
