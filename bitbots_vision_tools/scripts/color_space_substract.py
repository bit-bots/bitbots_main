#!/usr/bin/env python3

import os
import numpy as np
import yaml
import pickle
import argparse

def init_color_space(color_path):
    # type: (str) -> None
    """
    Initialization of color space from yaml or pickle.txt file

    :param str color_path: path to file containing the accepted colors
    :return: None
    """
    color_space = np.zeros((256, 256, 256), dtype=np.uint8)
    if color_path.endswith('.yaml'):
        with open(color_path, 'r') as stream:
            try:
                color_values = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                # TODO: what now??? Handle the error?
                pass
    # pickle-file is stored as '.txt'
    elif color_path.endswith('.txt'):
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
            color_space[color_values['blue'][x],
                        color_values['green'][x],
                        color_values['red'][x]] = 1
    print("Imported color space")
    return color_space  

def compare(positive_color_space, negative_color_space):
    mask = np.invert(np.array(negative_color_space, dtype=np.bool))
    binary_color_space = np.logical_and(mask, positive_color_space)
    return np.array(binary_color_space, dtype=np.uint8)

def generate_color_lists(color_space):
    color_space_positions = np.where(color_space == 1)
    color_lists = ( color_space_positions[0].tolist(),
                    color_space_positions[1].tolist(),
                    color_space_positions[2].tolist())
    return color_lists

def save(filename, color_space):           
    red, green, blue = generate_color_lists(color_space)

    output_type = "negative_filtered"

    data = dict(
        red = red,
        green = green,
        blue = blue
    )

    filename = '{}_{}.txt'.format(filename, output_type)
    with open(filename, 'wb') as outfile:
        pickle.dump(data, outfile, protocol=2)
        # stores data of colorspace in file as pickle for efficient loading (yaml is too slow)
    
    print("Output saved to '{}'.".format(filename))


def run(positive_color_space_path, negative_color_space_path, output_path):
    print("Load positive color space '{}'".format(positive_color_space_path))
    positive_color_space = init_color_space(positive_color_space_path)
    print(np.count_nonzero(positive_color_space))
    print("Load negative color space '{}'".format(negative_color_space_path))
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
