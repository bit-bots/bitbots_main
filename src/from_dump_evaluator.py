import math
import os
import pickle
import yaml
import matplotlib.pyplot as plt
import numpy as np
from evaluation_data_loader import DataLoader


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

        self.visualization_path = os.path.join(dirname, config["visualization"])
        
        self.loader = DataLoader(self.image_path, self.dimensions, 16)

    def evaluate_all_images(self, compass, matcher, samples):
        filtered_data = self.data
        filtered_data = filter(lambda entry: entry["compass"] == compass, filtered_data)
        filtered_data = filter(lambda entry: entry["matcher"] == matcher, filtered_data)
        filtered_data = filter(lambda entry: entry["samples"] == samples, filtered_data)

        print(str(len(filtered_data)) + " entries left (should be 1).")

        if len(filtered_data) == 1:
            datum = filtered_data[0]

            print("Compass: " + datum["compass"])
            print("Matcher: " + datum["matcher"])
            print("Samples: " + str(datum["samples"]))

            for i in range(16):
                angle = math.pi / 8.0 * i

                plt.figure(i)
                self.evaluate_direction(angle, datum["results"])
                filename = "" + str(i) + ".png"
                plt.savefig(os.path.join(self.visualization_path, filename))

    def evaluate_direction(self, thruth_angle, results):
        ax = plt.subplot(1, 2, 1)

        results = filter(lambda entry: np.isclose(entry["truth_angle"], thruth_angle, .002), results)

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
        plt.imshow(self.loader.getImage(4, 3, thruth_angle))


if __name__ == "__main__":
    evaluator = Evaluator()
    evaluator.evaluate_all_images("multiple", "sift", 16)
