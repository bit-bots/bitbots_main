import yaml
import os
import math

"""
A script used to correct the angles in our test data.
"""

def correct():
    data_path = os.path.dirname(__file__)
    index_path = os.path.join(data_path, "../evaluation_data/FieldData/Field_1/index_old.yaml")
    with open(index_path, 'r') as stream:
        index = yaml.load(stream)
    return map(correct_entry, index)


def correct_entry(entry):
    entry["angle"] = (-entry["angle"]) % (math.pi * 2)
    return entry


if __name__ == "__main__":
    data = correct()
    print (data)
    data_path = os.path.dirname(__file__)
    index_path = os.path.join(data_path, "../evaluation_data/FieldData/Field_1/index.yaml")
    with open(index_path, 'w') as stream:
        index = yaml.dump(data, stream)

