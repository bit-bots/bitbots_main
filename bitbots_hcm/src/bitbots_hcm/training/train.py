#!/usr/bin/env python3
import argparse
import math
import pickle
import random
from pathlib import Path
from os import listdir
from os.path import isfile, join

import optuna
import numpy as np
import sklearn
from optuna.samplers import TPESampler
from sklearn.gaussian_process import GaussianProcessClassifier
from sklearn.metrics import confusion_matrix, accuracy_score
from sklearn.model_selection import train_test_split
from sklearn.neural_network import MLPClassifier

from bitbots_hcm.training.dataset import Dataset
from bitbots_hcm.fall_checker import FallChecker

parser = argparse.ArgumentParser()
parser.add_argument('--storage', help='Database SQLAlchemy string, e.g. postgresql://USER:PASS@SERVER/DB_NAME',
                    default=None, type=str, required=False)
parser.add_argument('--name', help='Name of the study', default=None, type=str, required=False)
parser.add_argument('--iterations', help='iterations for optimization', default=100, type=int, required=False)
parser.add_argument('--knn', help='use knn', dest='knn', action='store_true')
parser.add_argument('--svc', help='use svc', dest='svc', action='store_true')
parser.add_argument('--mlp', help='use mlp', dest='mlp', action='store_true')
parser.add_argument('--gpc', help='use gpc', dest='gpc', action='store_true')
parser.add_argument('--bb', help='use bb', dest='bb', action='store_true')
parser.add_argument('--reduce', help='reduce data', dest='reduce', action='store_true')
parser.add_argument('--directories', help='directories of labelled training data', nargs='+', required=True)
args = parser.parse_args()

# load dataset
merged_dataset = Dataset()
for directory in args.directories:
    data_files = [f for f in listdir(directory) if isfile(join(directory, f))]
    print(F"Loaded datasets {data_files}")
    for file_name in data_files:
        with open(directory + "/" + file_name, 'rb') as file:
            dataset: Dataset = pickle.load(file)
            merged_dataset.append_dataset(dataset)

data_types = {'imu_raw': True, 'imu_orient': True, 'joint_states': False, 'imu_fused': False, 'cop': False}
print(f"Used datatypes are: {data_types}\n Change this in the script if you want to use other combinations.")

x, y = merged_dataset.as_scikit_data(imu_raw=data_types['imu_raw'], imu_orient=data_types['imu_orient'],
                                     joint_states=data_types['joint_states'], imu_fused=data_types['imu_fused'],
                                     cop=data_types['cop'])

print("Transformed to scipy data")
print(F"stable: {y.count(0)} front: {y.count(1)} back: {y.count(2)} left: {y.count(3)} right: {y.count(4)}")


def reduce_data(x, y, max_number=np.inf):
    lowest_number = np.inf
    for i in range(5):
        count = y.count(i)
        lowest_number = min(lowest_number, count)
    lowest_number = min(lowest_number, max_number)
    # make 5 lists with the indexes of frames for each class
    sorted = [[], [], [], [], []]
    for i in range(len(x)):
        sorted[y[i]].append(i)
    print(F'0 {len(sorted[0])} 1 {len(sorted[1])} 2 {len(sorted[2])} 3 {len(sorted[3])} 4 {len(sorted[4])}')
    x_out = []
    y_out = []
    for i in range(5):
        sampled_indexes = random.choices(sorted[i], k=lowest_number)
        for index in sampled_indexes:
            x_out.append(x[index])
            y_out.append(y[index])

    return x_out, y_out


if args.reduce:
    x, y = reduce_data(x, y, 200)
print(F'reduced to {y.count(0)} front: {y.count(1)} back: {y.count(2)} left: {y.count(3)} right: {y.count(4)}')
print(F'stable {y.count(0)} falling {y.count(1) + y.count(2) + y.count(3) + y.count(4)}')
scaler = sklearn.preprocessing.StandardScaler()
scaler.fit(x)

use_fn = False
if use_fn:
    x, x_test, y, y_test = train_test_split(x, y, random_state=0)
    x_test_scaled = scaler.transform(x_test)
    print(F'train is to {y.count(0)} front: {y.count(1)} back: {y.count(2)} left: {y.count(3)} right: {y.count(4)}')
    print(
        F'test is to {y_test.count(0)} front: {y_test.count(1)} back: {y_test.count(2)} left: {y_test.count(3)} right: {y_test.count(4)}')

x_scaled = scaler.transform(x)


def evaluate_classifier(classifier):
    if use_fn:
        classifier.fit(x_scaled, y)
        conf_mat = confusion_matrix(y_test, classifier.predict(x_test_scaled))
        FP = conf_mat.sum(axis=0) - np.diag(conf_mat)
        FN = conf_mat.sum(axis=1) - np.diag(conf_mat)
        TP = np.diag(conf_mat)
        # TN = conf_mat.values.sum() - (FP + FN + TP)
        print(FN)
        return FN[0] * 100 + FN[1] + FN[2] + FN[3] + FN[4]
    else:
        score = sklearn.model_selection.cross_val_score(classifier, x_scaled, y, n_jobs=-1, verbose=0)
        accuracy = score.mean()
        return accuracy


def knn(trial: optuna.Trial):
    n_neighbors = trial.suggest_int("n_neighbors", 1, 10)
    weights = trial.suggest_categorical("weights", ['uniform', 'distance'])
    algorithm = trial.suggest_categorical("algorithm", ['ball_tree', 'kd_tree'])
    leaf_size = trial.suggest_int('leaf_size', 1, 50)
    classifier = sklearn.neighbors.KNeighborsClassifier(n_neighbors=n_neighbors, weights=weights, algorithm=algorithm,
                                                        leaf_size=leaf_size)
    return evaluate_classifier(classifier)


def svc(trial: optuna.Trial):
    svc_c = trial.suggest_loguniform("svc_c", 1e-10, 1e10)
    kernel = trial.suggest_categorical('kernel', ['linear', 'poly', 'rbf', 'sigmoid'])
    degree = trial.suggest_int('degree', 1, 10)
    classifier = sklearn.svm.SVC(C=svc_c, gamma="auto", kernel=kernel, degree=degree)
    return evaluate_classifier(classifier)


def mlp(trial: optuna.Trial):
    hidden_layer_size = trial.suggest_int("hidden_layer_size", 10, 1000, 10)
    hidden_layer_count = trial.suggest_int("hidden_layer_count", 1, 10)
    hidden_layer = (hidden_layer_size,) * hidden_layer_count
    activation_fun = trial.suggest_categorical("activation_fun", ['identity', 'tanh', 'relu'])
    alpha = trial.suggest_float("alpha", 0.000001, 0.0001)
    learning_rate = trial.suggest_categorical('learning_rate', ['constant', 'invscaling', 'adaptive'])
    classifier = MLPClassifier(hidden_layer_sizes=hidden_layer, activation=activation_fun,
                               alpha=alpha, learning_rate=learning_rate)
    return evaluate_classifier(classifier)


def gpc(trial: optuna.Trial):
    multi_class = trial.suggest_categorical("multi_class", ['one_vs_rest', 'one_vs_one'])
    classifier = GaussianProcessClassifier(multi_class=multi_class)
    return evaluate_classifier(classifier)


def bb(trial: optuna.Trial):
    # our previous naive bitbots approach
    thresh_gyro_front = trial.suggest_float("thresh_gyro_front", 0, 20)
    thresh_gyro_side = trial.suggest_float("thresh_gyro_side", 0, 20)
    thresh_orient_front = trial.suggest_float("thresh_orient_front", 0, math.pi)
    tresh_orient_side = trial.suggest_float("tresh_orient_side", 0, math.pi)
    classifier = FallChecker(thresh_gyro_front=thresh_gyro_front, thresh_gyro_side=thresh_gyro_side,
                             thresh_orient_front=thresh_orient_front, tresh_orient_side=tresh_orient_side)
    return evaluate_classifier(classifier)


def pickle_classifier(classifier, type):
    path = "classifiers/" + type + "/"
    Path(path).mkdir(parents=True, exist_ok=True)
    with open(path + "classifier.pkl", "wb") as file:
        pickle.dump(classifier, file)
    with open(path + "scaler.pkl", "wb") as file:
        pickle.dump(scaler, file)
    with open(path + "types.pkl", "wb") as file:
        pickle.dump(data_types, file)


if args.knn:
    objective = knn
    name = 'knn'
elif args.mlp:
    objective = mlp
    name = 'mlp'
elif args.svc:
    objective = svc
    name = 'svc'
elif args.gpc:
    objective = gpc
    name = 'gpc'
elif args.bb:
    objective = bb
    name = 'bb'

seed = np.random.randint(2 ** 32 - 1)
sampler = TPESampler(n_startup_trials=10, seed=seed)
if use_fn:
    direction = 'minimize'
else:
    direction = 'maximize'
study = optuna.create_study(study_name=args.name, storage=args.storage, direction=direction,
                            sampler=sampler, load_if_exists=True)
print("Start optimizing hyper parameters")
study.optimize(objective, n_trials=args.iterations, show_progress_bar=True, n_jobs=1)

print("Train final classifier with optimized hyper parameters")
if args.knn:
    classifier = sklearn.neighbors.KNeighborsClassifier(n_neighbors=study.best_params['n_neighbors'],
                                                        weights=study.best_params['weights'],
                                                        algorithm=study.best_params['algorithm'],
                                                        leaf_size=study.best_params['leaf_size'])
elif args.mlp:
    hidden_layer = (study.best_params['hidden_layer_size'],) * study.best_params['hidden_layer_count']
    classifier = MLPClassifier(hidden_layer_sizes=hidden_layer, activation=study.best_params['activation_fun'],
                               alpha=study.best_params['alpha'], learning_rate=study.best_params['learning_rate'])
elif args.svc:
    classifier = sklearn.svm.SVC(C=study.best_params['svc_c'], gamma="auto", kernel=study.best_params['kernel'],
                                 degree=study.best_params['degree'])
elif args.gpc:
    classifier = GaussianProcessClassifier(multi_class=study.best_params['multi_class'])
elif args.bb:
    classifier = FallChecker(thresh_gyro_front=study.best_params['thresh_gyro_front'],
                             thresh_gyro_side=study.best_params['thresh_gyro_side'],
                             thresh_orient_front=study.best_params['thresh_orient_front'],
                             tresh_orient_side=study.best_params['tresh_orient_side'])
classifier.fit(x_scaled, y)
pickle_classifier(classifier, name)
