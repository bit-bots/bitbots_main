import sys
import os
import cv2
import yaml
import argparse

from soccer_field_map_generator.generator import (
    generate_map_image,
    generate_metadata,
    load_config_file,
)


def main():
    parser = argparse.ArgumentParser(description="Generate maps for localization")
    parser.add_argument("output", help="Output file name")
    parser.add_argument(
        "config",
        help="Config file for the generator that specifies the parameters for the map generation",
    )
    parser.add_argument(
        "--metadata",
        help="Also generates a 'map_server.yaml' file with the metadata for the map",
        action="store_true",
    )
    args = parser.parse_args()

    # Check if the config file exists
    if not os.path.isfile(args.config):
        print("Config file does not exist")
        sys.exit(1)

    # Load config file
    with open(args.config, "r") as config_file:
        parameters = load_config_file(config_file)

    # Check if the config file is valid
    if parameters is None:
        print("Invalid config file")
        sys.exit(1)

    # Generate the map image
    image = generate_map_image(parameters)

    # Make output folder full path
    output_path = os.path.abspath(args.output)

    # Check if the output folder exists
    if not os.path.isdir(os.path.dirname(output_path)):
        print("Output folder does not exist")
        sys.exit(1)

    # Save the image
    cv2.imwrite(output_path, image)

    # Generate the metadata
    if args.metadata:
        metadata = generate_metadata(parameters, os.path.basename(output_path))
        metadata_file_name = os.path.join(
            os.path.dirname(output_path), "map_server.yaml"
        )
        with open(metadata_file_name, "w") as metadata_file:
            yaml.dump(metadata, metadata_file, sort_keys=False)


if __name__ == "__main__":
    main()
