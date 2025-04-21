from argparse import ArgumentParser

import matplotlib.pyplot as plt
import numpy as np
import yaml

parser = ArgumentParser()
args = parser.parse_args()


plt.rcParams.update({'font.size': 16})


# folder = "/homes/15guelden/footstep_ws/src/bitbots_main/bitbots_vision/saved_data_donna/realworld_wolfgang/"
#folder = "/homes/15guelden/saved_data/sim_op3/"
#folder = "/homes/21stahl/bitbots_main/bitbots_vision/saved_data_donna/sim_op3/"
#folder = "/homes/21stahl/bitbots_main/bitbots_vision/saved_data_donna/realworld_wolfgang/"
#folder = "/Users/benedict/UHH/bitbots/bitbots_main/bitbots_vision/saved_data_donna/realworld_wolfgang/"
folder = "/Users/benedict/UHH/bitbots/bitbots_main/bitbots_vision/saved_data_donna/sim_op3/"

if folder.endswith("sim_op3/"):
    idx = range(0, 20)
elif folder.endswith("realworld_wolfgang/"):
    idx = range(0, 9)

def load_yaml_file(file_path):
    with open(file_path) as file:
        data = yaml.safe_load(file)
    try:
        distance_measured = data["measured"]["distance"]
    except KeyError:
        distance_measured = data["ground_truth_robot_pose"]["d"]
    distance_base_footprint = data["relative_base_footprint"]["d"]
    distance_baseline = data["relative_baseline"]["d"]
    return distance_measured, distance_base_footprint, distance_baseline


measured_distances = []
base_footprint_distances = []
baseline_distances = []
for i in idx:
    file_path = f"{folder}{i:03d}_data.yaml"
    distance_measured, distance_base_footprint, distance_baseline = load_yaml_file(file_path)
    measured_distances.append(distance_measured)
    base_footprint_distances.append(distance_base_footprint)
    baseline_distances.append(distance_baseline)

# put all into np 2d array
data = np.array([measured_distances, base_footprint_distances, baseline_distances])
# sort by measured distance
data = data[:, np.argsort(data[0])]
# plot data as points

plt.plot(data[0], data[0], label="ideal", color="#ffb000", linewidth=2, zorder=0)

plt.scatter(data[0], data[1], label="base footprint", color="#648fff", marker="+", s=110, linewidths=2, zorder=2)
plt.scatter(data[0], data[2], label="baseline", color="#dc267f", marker="x", s=70, linewidths=2, zorder=1)
plt.xlabel("ground truth distance")
plt.ylabel("estimated distance")
# plot diagonal line
plt.legend()
plt.tight_layout()
fig = plt.gcf()
fig.set_size_inches(5, 5)
plt.show()

"""
plt.plot(data[0], data[0], label="ideal", color="limegreen", linewidth=2, zorder=0)

plt.scatter(data[0], data[1], label="base footprint", color="#648fff", marker="+", s=110, linewidths=2, zorder=2)
plt.scatter(data[0], data[2], label="baseline", color="#dc267f", marker="x", s=70, linewidths=2, zorder=1)
plt.xlabel("ground truth distance")
plt.ylabel("estimated distance")
# plot diagonal line
plt.legend()
plt.tight_layout()
fig = plt.gcf()
fig.set_size_inches(5, 5)
plt.show()
"""

# get number of samples where base footprint distance is cloaser to measured distance
num_base_footprint_closer = 0
num_baseline_closer = 0
for i in range(len(data[0])):
    if abs(data[0][i] - data[1][i]) < abs(data[0][i] - data[2][i]):
        num_base_footprint_closer += 1
    else:
        num_baseline_closer += 1
print(f"Number of samples where base footprint distance is closer to measured distance: {num_base_footprint_closer}")

relative_errors_bf = []
relative_erros_baseline = []
for i in range(len(data[0])):
    relative_error_bf = abs(data[0][i] - data[1][i]) / data[0][i]
    relative_error_baseline = abs(data[0][i] - data[2][i]) / data[0][i]
    relative_errors_bf.append(relative_error_bf)
    relative_erros_baseline.append(relative_error_baseline)
print(f"Mean relative error base footprint: {np.mean(relative_errors_bf):.5f}")
print(f"Std dev relative error base footprint: {np.std(relative_errors_bf):.5f}")
print(f"Mean relative error baseline: {np.mean(relative_erros_baseline):.5f}")
print(f"Std dev relative error baseline: {np.std(relative_erros_baseline):.5f}")

absolute_errors_bf = []
absolute_errors_baseline = []
for i in range(len(data[0])):
    absolute_error_bf = abs(data[0][i] - data[1][i])
    absolute_error_baseline = abs(data[0][i] - data[2][i])
    absolute_errors_bf.append(absolute_error_bf)
    absolute_errors_baseline.append(absolute_error_baseline)
print(f"Mean absolute error base footprint: {np.mean(absolute_errors_bf):.5f}")
print(f"Std dev absolute error base footprint: {np.std(absolute_errors_bf):.5f}")
print(f"Mean absolute error baseline: {np.mean(absolute_errors_baseline):.5f}")
print(f"Std dev absolute error baseline: {np.std(absolute_errors_baseline):.5f}")


# plot absulute distance error for base footprint and baseline against distance
base_footprint_distance_deviation = []
baseline_distance_deviation = []
for i in range(len(measured_distances)):
    base_footprint_distance_deviation.append(abs(base_footprint_distances[i] - measured_distances[i]))
    baseline_distance_deviation.append(abs(baseline_distances[i] - measured_distances[i]))



rcond = 0.2
m, b = np.polyfit(measured_distances, base_footprint_distance_deviation, deg=1, rcond=rcond)
plt.plot(measured_distances, m * np.array(measured_distances) + b, label="base footprint fit", color="#648fff")

m, b = np.polyfit(measured_distances, baseline_distance_deviation, deg=1, rcond=rcond)
plt.plot(measured_distances, m * np.array(measured_distances) + b, label="baseline fit", color="#dc267f")

plt.scatter(measured_distances, base_footprint_distance_deviation, label="base footprint", color="#648fff", marker="+", s=110, linewidths=2, zorder=2)
plt.scatter(measured_distances, baseline_distance_deviation, label="baseline", color="#dc267f", marker="x", s=70, linewidths=2, zorder=1)
#plt.plot(measured_distances, [mean_base_footprint_distance_deviation] * len(measured_distances), label="Mean base footprint")
#plt.plot(measured_distances, [mean_baseline_distance_deviation] * len(measured_distances), label="Mean baseline")

plt.xlabel("ground truth distance")
plt.ylabel("absulute distance deviation")
plt.legend()
plt.tight_layout()
fig = plt.gcf()
fig.set_size_inches(5, 5)
plt.show()

"""
base_footprint_distance_deviation = []
baseline_distance_deviation = []
for i in range(len(measured_distances)):
    base_footprint_distance_deviation.append(base_footprint_distances[i] - measured_distances[i])
    baseline_distance_deviation.append(baseline_distances[i] - measured_distances[i])

mean_base_footprint_distance_deviation = np.mean(base_footprint_distance_deviation)
mean_baseline_distance_deviation = np.mean(baseline_distance_deviation)

plt.scatter(measured_distances, base_footprint_distance_deviation, label="base footprint", marker="x", color="#648fff")
plt.scatter(measured_distances, baseline_distance_deviation, label="baseline", marker="x", color="#dc267f")
plt.plot(measured_distances, [mean_base_footprint_distance_deviation] * len(measured_distances), label="Mean base footprint deviation", color="#648fff")
plt.plot(measured_distances, [mean_baseline_distance_deviation] * len(measured_distances), label="Mean baseline deviation", color="#dc267f")
plt.plot(measured_distances, [0] * len(measured_distances), label="ideal", color="green", linewidth=2)

plt.xlabel("ground truth distance")
plt.ylabel("Signed distance deviation")
plt.legend(loc="upper left")
plt.tight_layout()
fig = plt.gcf()
fig.set_size_inches(5, 5)
plt.show()
"""
