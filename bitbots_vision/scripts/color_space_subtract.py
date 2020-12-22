#!/usr/bin/env python3

import os
import numpy as np
import yaml
import pickle
import argparse

"""
Another small tool for color space enhancement.

This tool is able to subtract color values from one color space file from another.
"""

def init_color_lookup_table(color_path):
    # type: (str) -> None
    """
    Initialization of color space from .yaml or .pickle file

    :param str color_path: path to file containing the accepted colors
    :return: None
    """
    color_lookup_table = np.zeros((256, 256, 256), dtype=np.uint8)
    if color_path.endswith('.yaml'):
        with open(color_path, 'r') as stream:
            try:
                color_values = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print("Unable to open File!!!")
    # pickle-file is stored as '.pickle'
    elif color_path.endswith('.pickle'):
        try:
            with open(color_path, 'rb') as f:
                color_values = pickle.load(f)
        except pickle.PickleError as exc:
            pass

    # compatibility with colorpicker
    if 'color_values' in color_values.keys():
        color_values = color_values['color_values']['greenField']
    length = len(color_values['red'])
    if length == len(color_values['green']) and \
                    length == len(color_values['blue']):
        # setting colors from yaml file to True in color space
        for x in range(length):
            color_lookup_table[color_values['blue'][x],
                        color_values['green'][x],
                        color_values['red'][x]] = 1
    print("Imported color space")
    return color_lookup_table

def compare(positive_color_lookup_table, negative_color_lookup_table):
    mask = np.invert(np.array(negative_color_lookup_table, dtype=np.bool))
    binary_color_lookup_table = np.logical_and(mask, positive_color_lookup_table)
    return np.array(binary_color_lookup_table, dtype=np.uint8)

def generate_color_lists(color_lookup_table):
    color_lookup_table_positions = np.where(color_lookup_table == 1)
    color_lists = ( color_lookup_table_positions[0].tolist(),
                    color_lookup_table_positions[1].tolist(),
                    color_lookup_table_positions[2].tolist())
    return color_lists

def save(filename, color_lookup_table):
    red, green, blue = generate_color_lists(color_lookup_table)

    output_type = "negative_filtered"

    data = dict(
        red = red,
        green = green,
        blue = blue
    )

    filename = f'{filename}_{output_type}.pickle'
    with open(filename, 'wb') as outfile:
        pickle.dump(data, outfile, protocol=2)
        # stores data of ColorLookupTable in file as pickle for efficient loading (yaml is too slow)

    print(f"Output saved to '{filename}'.")

def run(positive_color_lookup_table_path, negative_color_lookup_table_path, output_path):
    print(f"Load positive color space '{positive_color_lookup_table_path}'")
    positive_color_lookup_table = init_color_lookup_table(positive_color_space_path)
    print(np.count_nonzero(positive_color_space))
    print(f"Load negative color space '{negative_color_space_path}'")
    negative_color_space = init_color_space(negative_color_space_path)
    print(np.count_nonzero(negative_color_space))
    print("Filter color spaces")
    filtered_color_space = compare(positive_color_space, negative_color_space)
    print(np.count_nonzero(filtered_color_space))
    print("Finished filtering")
    print("Save color space")
    save(output_path, filtered_color_space)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--positive", help="Color space where the negative color space is subtracted from.")
    parser.add_argument("-n", "--negative", help="Color space which is subtracted from the positive color space.")
    parser.add_argument("-o", "--output", help="Saves the output in a Pickle file.")
    args = parser.parse_args()
    np.warnings.filterwarnings('ignore')

    if args.positive and os.path.exists(args.positive):
        if args.negative and os.path.exists(args.negative):
            if args.output and os.path.exists(args.output):
                run(args.positive, args.negative, args.output)
            else:
                print("Output path incorrect!")
        else:
            print("Negative color space path incorrect!")
    else:
        print("Positive color space path incorrect!")
