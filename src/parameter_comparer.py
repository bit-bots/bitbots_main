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

        #Sself.plot_splatter(matrices_by_matcher_with_16_05)
        self.plot([matrices_by_matcher_with_16_03, matrices_by_matcher_with_16_04, matrices_by_matcher_with_16_05])

    def plot(self, matrices):
        plt.figure(2)
        for i in [0, 1]:
            for j in [0, 1]:
                ax = plt.subplot(2, 2, j+1 + i*2)
                self.plot_single_entry(matrices, (i, j), ax)
        plt.legend(["confidence threshold = " + str(cd) for cd in [.3, .4, .5]])
        plt.show()

    def plot_single_entry(self, matrices, pos, ax):
        x = matrices[0].keys()
        # self.print_matrices(matrices)
        for m in matrices:
            y = [matrix[pos[0]][pos[1]] for matrix in m.values()]
            ax.plot(x, y)

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


if __name__ == "__main__":
    comparer = Comparer()

    # configuration = evaluator.filter_configuration("multiple", "sift", 16)[0]
    # evaluator.evaluate_configuration(configuration)

    comparer.compare_all()