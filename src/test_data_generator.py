import math
import os
import yaml
from evaluation_data_loader import DataLoader
from worker import VisualCompass
from pickle import dump


def generate_configs():
    config_table = []
    for compass in ["multiple", "binary"]:
        for matcher in ["orb", "akaze", "sift"]:
            if compass == "multiple":
                for samples in range(2,17):
                    config_table.append({"compass": compass, "matcher": matcher, "samples": samples})
            else:
                config_table.append({"compass": compass, "matcher": matcher, "samples": 2})
    return config_table


def initialize_evaluation(config_table_entry):
    dimensions = (10, 7)
    angle_steps = 16

    dirname = os.path.dirname(__file__)
    relative_config_path = "../config/config.yaml"
    config_path = os.path.join(dirname, relative_config_path)

    with open(config_path, 'r') as stream:
        config = yaml.load(stream)

    relative_data_path = config['evaluation_data']
    data_path = os.path.join(dirname, relative_data_path)

    loader = DataLoader(data_path, dimensions, angle_steps)

    config["compass_type"] = config_table_entry["compass"]
    config["compass_matcher"] = config_table_entry["matcher"]
    config["compass_multiple_sample_count"] = config_table_entry["samples"]

    compass = VisualCompass(config)
    print (config_table_entry["compass"] + " " + config_table_entry["matcher"] + " " + str(config_table_entry["samples"]))

    return compass, loader, dimensions


def evaluate(config_table_entry):
    compass, loader, dimensions = initialize_evaluation(config_table_entry)

    set_truth(compass, loader, config_table_entry)
    results = evaluate_all_images(compass, loader, dimensions)
    config_table_entry["results"] = results
    return config_table_entry


def set_truth(compass, loader, config_table_entry):
    for i in range(config_table_entry["samples"]):
        angle = float(i) / config_table_entry["samples"] * math.radians(360)
        angle = (angle + math.radians(90)) % math.radians(360)
        image = loader.get_image(4, 3, angle)
        compass.set_truth(angle, image)


def evaluate_all_images(compass, loader, dimensions):
    result = []
    for step in range(16):
        truth_angle = (float(step) * math.pi / 8) % (math.pi * 2)
        for i in range(dimensions[0]):
            for j in range(dimensions[1]):
                image = loader.get_image(i, j, truth_angle)
                angle, confidence = compass.process_image(image)
                result.append({"x": i, "y": j, "angle": angle, "confidence": confidence, "truth_angle": truth_angle})
        print ("step " + str(step) + " done.")
    return result


def print_data(data):
    print (data)
    for d in data[0]["results"]:
        if d["x"] == 4 and d["y"]==3:
            d["angle"] = math.degrees(d["angle"])
            d["truth_angle"] = math.degrees(d["truth_angle"])
            print(d)


if __name__ == "__main__":
    print (generate_configs())
    data = map(evaluate, generate_configs())
    print_data(data)
    file_path = "datadump.pickle"

    with open(file_path, 'wb') as stream:
        dump(data, stream)
