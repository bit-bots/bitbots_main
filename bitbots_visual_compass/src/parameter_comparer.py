import math
import os
import pickle
import yaml
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import matplotlib.gridspec as gridspec
import numpy as np
from evaluation_data_loader import DataLoader


class Comparer(object):
    def __init__(self):
        dirname = os.path.dirname(__file__)

        pickle_path = os.path.join(dirname, "confusion_matrices.pickle")
        with open(pickle_path, 'rb') as stream:
            self.data = pickle.load(stream)

    def compare_all(self):
        matrices_by_samples = {key: np.zeros((2, 2)) for key in range(2, 17)}
        matrices_by_matcher = {key: np.zeros((2, 2)) for key in ['orb', 'akaze', 'sift']}
        matrices_by_matcher_with_16_03 = {key: np.zeros((2, 2)) for key in ['orb', 'akaze', 'sift']}
        matrices_by_matcher_with_16_04 = {key: np.zeros((2, 2)) for key in ['orb', 'akaze', 'sift']}
        matrices_by_matcher_with_16_05 = {key: np.zeros((2, 2)) for key in ['orb', 'akaze', 'sift']}
        for datum in self.data:
            if datum["compass"] == "multiple":
                matrices_by_samples[datum['samples']] = \
                    np.add(matrices_by_samples[datum['samples']], datum['confusion_matrix'])
                matrices_by_matcher[datum['matcher']] = \
                    np.add(matrices_by_matcher[datum['matcher']], datum['confusion_matrix'])
                if datum["samples"] == 16:
                    if datum['confidence_threshold'] < .35:
                        matrices_by_matcher_with_16_03[datum['matcher']] = \
                            np.add(matrices_by_matcher_with_16_03[datum['matcher']], datum['confusion_matrix'])
                    elif datum['confidence_threshold'] < .45:
                        matrices_by_matcher_with_16_04[datum['matcher']] = \
                            np.add(matrices_by_matcher_with_16_04[datum['matcher']], datum['confusion_matrix'])
                    else:
                        matrices_by_matcher_with_16_05[datum['matcher']] = \
                            np.add(matrices_by_matcher_with_16_05[datum['matcher']], datum['confusion_matrix'])

        # self.plot_splatter(matrices_by_matcher_with_16_05)
        self.plot([matrices_by_matcher_with_16_03, matrices_by_matcher_with_16_04, matrices_by_matcher_with_16_05])
        # print(matrices_by_matcher_with_16_03)

    def plot(self, matrices):
        plt.figure(2, figsize=(8, 6), dpi=300)

        gs = gridspec.GridSpec(3, 2, width_ratios=[1, 1], height_ratios=[5, 5, 1])
        ax_legend = plt.subplot(gs[2, :])
        ax_legend.axis('off')

        ax = None
        for i in [0, 1]:
            for j in [0, 1]:
                ax = plt.subplot(gs[i, j])
                ax.set_title(self.get_label_for_pos(i, j))
                self.plot_single_entry(matrices, (i, j), ax)
        handles, labels = ax.get_legend_handles_labels()
        print (handles)
        print (labels)

        ax_legend.legend(handles, labels, loc='center', ncol=3)
        plt.show()

    def plot_single_entry(self, matrices, pos, ax):

        for i, m in enumerate(matrices):
            x = [offset + (i - 1) * .2 for offset in range(3)]
            y = [matrix[pos[0]][pos[1]] for matrix in m.values()]

            # print (x)
            # print (y)

            ax.bar(x, y, width=.2, label="conf. threshold: " + str(.3 + i/10.))
        ax.set_xticks(range(len(matrices[0])))
        ax.set_xticklabels(matrices[0].keys())

        print(ax.get_legend_handles_labels())

    def plot_splatter(self, matrices):
        map(lambda x: self.plot_matrix(x), matrices.values())
        plt.legend(matrices.keys())
        plt.show()

    def plot_matrix(self, matrix):
        x = []
        y = []
        for i in [0, 1]:
            for j in [0, 1] if i == 0 else [1, 0]:
                x.append((j-.5) * matrix[i][j])
                y.append((-i+.5) * matrix[i][j])
        print("x=" + str(x) + ", y=" + str(y))

        x.append(x[0])
        y.append(y[0])
        plt.plot(x, y)

    def print_matrices(self, matrices):
        for key, matrix in matrices.items():
            print (str(key) + ":")
            print (matrix)

    def get_label_for_pos(self, i, j):
        if i == 0:
            if j == 0:
                return "True Negatives\nwrong angle, low confidence"
            else:
                return "False Positives\nwrong angle, high confidence"
        else:
            if j == 0:
                return "False Negatives\ncorrect angle, low confidence"
            else:
                return "True Positives\ncorrect angle, high confidence"


if __name__ == "__main__":
    comparer = Comparer()

    # configuration = evaluator.filter_configuration("multiple", "sift", 16)[0]
    # evaluator.evaluate_configuration(configuration)

    comparer.compare_all()