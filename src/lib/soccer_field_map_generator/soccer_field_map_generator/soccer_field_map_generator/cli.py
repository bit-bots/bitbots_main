# Copyright (c) 2023 Hamburg Bit-Bots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import argparse
import os
import sys

import cv2
from soccer_field_map_generator.generator import (
    generate_map_image,
    generate_metadata,
    load_config_file,
)
import yaml


def main():
    parser = argparse.ArgumentParser(description='Generate maps for localization')
    parser.add_argument('output', help='Output file name')
    parser.add_argument(
        'config',
        help='Config file for the generator that specifies the parameters for the map generation',
    )
    parser.add_argument(
        '--metadata',
        help="Also generates a 'map_server.yaml' file with the metadata for the map",
        action='store_true',
    )
    args = parser.parse_args()

    # Check if the config file exists
    if not os.path.isfile(args.config):
        print('Config file does not exist')
        sys.exit(1)

    # Load config file
    with open(args.config, 'r') as config_file:
        parameters = load_config_file(config_file)

    # Check if the config file is valid
    if parameters is None:
        print('Invalid config file')
        sys.exit(1)

    # Generate the map image
    image = generate_map_image(parameters)

    # Make output folder full path
    output_path = os.path.abspath(args.output)

    # Check if the output folder exists
    if not os.path.isdir(os.path.dirname(output_path)):
        print('Output folder does not exist')
        sys.exit(1)

    # Save the image
    cv2.imwrite(output_path, image)

    # Generate the metadata
    if args.metadata:
        metadata = generate_metadata(parameters, os.path.basename(output_path))
        metadata_file_name = os.path.join(
            os.path.dirname(output_path), 'map_server.yaml'
        )
        with open(metadata_file_name, 'w') as metadata_file:
            yaml.dump(metadata, metadata_file, sort_keys=False)


if __name__ == '__main__':
    main()
