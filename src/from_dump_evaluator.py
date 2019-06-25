# -*- coding: utf-8 -*-
from __future__ import unicode_literals
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
    def __init__(self, confidence_threshold, correctness_threshold):

        self.confidence_threshold = confidence_threshold
        self.correctness_threshold = correctness_threshold

        self.dimensions = (7, 10)
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
        
        self.loader = DataLoader(self.image_path, (10, 7), 16)

    def evaluate_all_configurations(self, confidence_thresholds, plot):
        config_confusion_matrices = []
        for confidence_threshold in confidence_thresholds:
            for datum in self.data:
                if datum["compass"] == "multiple":
                    confusion_matrix = self.evaluate_configuration(datum, confidence_threshold, plot)
                    config_confusion_matrices.append({
                        'compass': datum['compass'],
                        'matcher': datum['matcher'],
                        'samples': datum['samples'],
                        'confidence_threshold': confidence_threshold,
                        'confusion_matrix': confusion_matrix
                    })
        return config_confusion_matrices

    def filter_configuration(self, compass, matcher, samples):
        filtered_data = self.data
        filtered_data = filter(lambda entry: entry["compass"] == compass, filtered_data)
        filtered_data = filter(lambda entry: entry["matcher"] == matcher, filtered_data)
        filtered_data = filter(lambda entry: entry["samples"] == samples, filtered_data)

        return filtered_data

    def evaluate_configuration(self, config, confidence_threshold, plot):
        print("\nCompass: " + config["compass"])
        print("Matcher: " + config["matcher"])
        print("Samples: " + str(config["samples"]))

        total_confusion_matrix = np.zeros((2, 2))
        for i in range(16):
            angle = math.pi / 8.0 * i

            plt.figure(i, dpi=300)
            confusion_matrix = self.evaluate_direction(angle, config["results"], confidence_threshold, plot)
            total_confusion_matrix = np.add(total_confusion_matrix, confusion_matrix)

            folder_name = config["compass"] + "_" + config["matcher"] + "_" + str(config["samples"])
            path = os.path.join(self.visualization_path, folder_name)
            if not os.path.exists(path):
                os.makedirs(path)
            file_name = "" + str(i) + ".png"
            plt.savefig(os.path.join(path, file_name), transparent=True)
        print("Total False Positives: " + str(total_confusion_matrix[0][1]))
        return total_confusion_matrix

    def evaluate_direction(self, truth_angle, results, confidence_threshold, plot):
        results = filter(lambda entry: np.isclose(entry["truth_angle"], truth_angle, .002), results)

        U = np.zeros(self.dimensions)
        V = np.zeros(self.dimensions)
        C = np.zeros(self.dimensions)

        false_positives = []
        true_negatives = []
        false_negatives = []
        true_positives = []

        for result in results:
            angle = result["angle"]
            confidence = result["confidence"]
            angle_diff = abs(angle - truth_angle)
            if angle_diff > math.pi:
                angle_diff = math.pi * 2 - angle_diff
            correctness = 1 - (angle_diff / math.pi)

            y = 9 - result["x"]
            x = 6 - result["y"]
            z = np.exp(1j * (angle + math.pi / 2.))
            U[x, y] = np.real(z) * confidence
            V[x, y] = np.imag(z) * confidence
            C[x, y] = correctness

            if correctness > self.correctness_threshold:
                if confidence > confidence_threshold:
                    true_positives.append((y, x))
                else:
                    false_negatives.append((y, x))
            else:
                if confidence > confidence_threshold:
                    false_positives.append((y, x))
                else:
                    true_negatives.append((y, x))

        if plot:
            self.plot(U, V, C, truth_angle, false_positives)

        confusion_matrix = np.array([[len(true_negatives), len(false_positives)],
                                     [len(false_negatives), len(true_positives)]])
        return confusion_matrix

    def plot(self, U, V, C, truth_angle, false_positives):
        # https://matplotlib.org/users/gridspec.html
        gs = gridspec.GridSpec(2, 2, width_ratios=[10, 1], height_ratios=[1, 8])
        ax_quiver = plt.subplot(gs[:, 0])
        # ax_image = plt.subplot(gs[0, 0])
        ax_truth = plt.subplot(gs[0, 1], xlim=(-.5, .5), ylim=(-.5, .5), aspect="equal")
        ax_colorbar = plt.subplot(gs[1, 1])

        cmap = colors.LinearSegmentedColormap.from_list("", ["red", "yellow", "green"])
        normalizer = colors.Normalize(0, 1)

        quiver = ax_quiver.quiver(U, V, C, pivot='mid', scale_units='xy', scale=1, cmap=cmap, norm=normalizer)
        ax_quiver.axis('equal')
        ax_quiver.set_xlim(-.5, 9.5)
        ax_quiver.set_ylim(-.5, 6.5)

        ax_quiver.set_xticks(range(0, 10, 3))
        ax_quiver.set_xticklabels(map(lambda x: str(x) + "m", range(0, 10, 3)))
        ax_quiver.xaxis.tick_top()

        ax_quiver.set_yticks(range(0, 7, 2))
        ax_quiver.set_yticklabels(map(lambda x: str(x) + "m", range(0, 7, 2)))
        # ax_quiver.axis('off')
        for fp in false_positives:
            circle = plt.Circle(fp, .5, fill=False)
            ax_quiver.add_artist(circle)

        z_truth = np.exp(1j * (truth_angle + math.radians(90)))
        ax_truth.arrow(
            -np.real(z_truth) / 2.,
            -np.imag(z_truth) / 2.,
            np.real(z_truth),
            np.imag(z_truth),
            length_includes_head=True,
            head_width=0.1,
            head_length=0.2)
        ax_truth.axis('off')

        cb = plt.colorbar(quiver, cax=ax_colorbar, use_gridspec=True, orientation='vertical', aspect=50)
        yrange = np.arange(0, 1.1, .25)
        cb.set_ticks(yrange)
        cb.set_ticklabels(map(lambda x: str((1-x)*180) + "°", yrange))

        # ax_image.imshow(self.loader.get_image(4, 3, truth_angle))
        # ax_image.set_title("Hallo\nDies ist ein Test.\nWie viel passt rein?")
        # ax_image.axis('off')
        # ax_colorbar.set_title("Correctness Colorbar")


def safe_to_file(config_confusion_matrices):
    file_path = "confusion_matrices.pickle"

    with open(file_path, 'wb') as stream:
        pickle.dump(config_confusion_matrices, stream)

if __name__ == "__main__":
    evaluator = Evaluator(.5, .75)

    configuration = evaluator.filter_configuration("multiple", "sift", 16)[0]
    evaluator.evaluate_configuration(configuration, .5, True)

    # safe_to_file(evaluator.evaluate_all_configurations([.3, .4, .5], True))
