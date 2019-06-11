import math
import os
import pickle
import yaml
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import matplotlib.gridspec as gridspec
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

    def evaluate_all_images(self):
        for datum in self.data:
            if datum["compass"] == "multiple" and datum["matcher"] == "sift":
                self.evaluate_configuration(datum)

    def filter_configuration(self, compass, matcher, samples):
        filtered_data = self.data
        filtered_data = filter(lambda entry: entry["compass"] == compass, filtered_data)
        filtered_data = filter(lambda entry: entry["matcher"] == matcher, filtered_data)
        filtered_data = filter(lambda entry: entry["samples"] == samples, filtered_data)

        return filtered_data

    def evaluate_configuration(self, config):
        print("Compass: " + config["compass"])
        print("Matcher: " + config["matcher"])
        print("Samples: " + str(config["samples"]))

        for i in range(16):
            angle = math.pi / 8.0 * i

            plt.figure(i)
            self.evaluate_direction(angle, config["results"])

            folder_name = config["compass"] + "_" + config["matcher"] + "_" + str(config["samples"])
            path = os.path.join(self.visualization_path, folder_name)
            if not os.path.exists(path):
                os.makedirs(path)
            file_name = "" + str(i) + ".png"
            plt.savefig(os.path.join(path, file_name))

    def evaluate_direction(self, truth_angle, results):
        results = filter(lambda entry: np.isclose(entry["truth_angle"], truth_angle, .002), results)

        U = np.zeros(self.dimensions)
        V = np.zeros(self.dimensions)
        C = np.zeros(self.dimensions)
        for result in results:
            angle = result["angle"]
            confidence = result["confidence"]
            angle_diff = abs(angle - truth_angle)
            if angle_diff > math.pi:
                angle_diff = math.pi * 2 - angle_diff
            correctness = angle_diff / math.pi

            x = result["x"]
            y = result["y"]
            z = np.exp(1j*angle)
            U[x, y] = np.real(z) * confidence
            V[x, y] = np.imag(z) * confidence
            C[x, y] = correctness

        self.plot(U, V, C, truth_angle)

    def plot(self, U, V, C, truth_angle):
        # https://matplotlib.org/users/gridspec.html

        cmap = colors.LinearSegmentedColormap.from_list("", ["green", "yellow", "red"])
        normalizer = colors.Normalize(0, 1)

        ax = plt.subplot(1, 2, 1)
        Q = ax.quiver(U, V, C, pivot='mid', scale_units='xy', scale=1, cmap=cmap, norm=normalizer)
        ax.axis('equal')
        ax.axis('off')

        plt.subplot(1, 2, 2)
        plt.imshow(self.loader.getImage(4, 3, truth_angle))


if __name__ == "__main__":
    evaluator = Evaluator()

    configuration = evaluator.filter_configuration("multiple", "sift", 16)[0]
    evaluator.evaluate_configuration(configuration)

    # evaluator.evaluate_all_images()
