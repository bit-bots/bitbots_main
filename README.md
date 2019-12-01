Bit-Bots Vision
===============

[![CodeFactor](https://www.codefactor.io/repository/github/bit-bots/bitbots_vision/badge)](https://www.codefactor.io/repository/github/bit-bots/bitbots_vision)
&nbsp;&nbsp;
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

This is the vision ROS package of the Hamburg Bit-Bots.

The vision is able to detect lines, the field itself, the field boundary, goal posts, teammates, enemies and other obstacles.

See the documentation of this package at our website: [doku.bit-bots.de](http://doku.bit-bots.de/meta/manual/software/vision.html)

An earlier version of this pipeline is presented in our paper [An Open Source Vision Pipeline Approach for RoboCup Humanoid Soccer](https://robocup.informatik.uni-hamburg.de/wp-content/uploads/2019/06/vision_paper.pdf). 
When you use this pipeline or parts of it, please cite it.
```
@inproceedings{vision2019,
author={Fiedler, Niklas and Brandt, Hendrik and Gutsche, Jan and Vahl, Florian and Hagge, Jonas and Bestmann, Marc},
year={2019},
title={An Open Source Vision Pipeline Approach for RoboCup Humanoid Soccer},
booktitle={RoboCup 2019: Robot World Cup XXIII},
note = {Accepted},
organization={Springer}
}
```

Its architecture is modular allowing easy implementation of new approaches resulting in a high grade of customizability.

For ball detection, you can choose between an fcnn or multiple yolo implementations.
The goalpost detection also runs via yolo or a conventional detection method,
which is also used for obstacle and robot detection.

The whole system is embedded in the ROS environment and
able to run on many devices including the Nvidia Jetson TX2 in our Wolfgang robots.

In the context of the Hamburg Bit-Bots, the Images are provided by a Basler industry grade ethernet camera.
The camera drivers are not included in this package but can be auto launched.

If you want the vision to run without starting a camera driver simply set the cli launch parameter `camera:=false`.
Every image source, that publishes a `sensor_msgs/Image messages` message is also supported.
The ROS topics and many other parameters are defined in the `visioparams.yaml` config file.
All used parameters are also changeable during run-time using ros dynamic reconfigure.
For simulation usage, different parameters can be defined in the ``simparam.yaml`` which overrides the normal params.
The debug mode with special debug output can be activated using ``debug:=true``.

In ``bitbots_vision/models`` the neural network models are stored. These models are not part of this repository.
Bit-Bots use the `pull_data` script in bitbots_meta.

For the field detection which is needed for the field boundary and
obstacle detection the vision uses RGB lookup table color spaces provided by the wolves_color_picker.
These color spaces can be improved and converted to a pickle file for faster loading times using the colorspace_tool in the vision tools.
The field color space itself can be dynamically adapted in real-time using the dynamic color space heuristic.
Therefore the vision gets more resistant in natural light conditions.


Launchscripts
-------------

To start the vision, use
```
roslaunch bitbots_vision vision_startup.launch
```

`sim:=true` does activate simulation time, switch to simulation color settings and deactivate launching of an image provider
`camera:=false` does deactivate all image providers (e.g. for use with rosbags or in simulation)
`basler:=false`does start wolves image provider instead of the basler camera driver
`dummyball:=true` does not start the ball detection to save resources
`debug:=true` does activate publishing of several debug images which can be inspected in the rqt image view
`use_game_settings:=true` does load additional game settings

**bitbots_vision**
- *vision_startup*: starts the vision and a camera image provider
    - Params:
        - sim [true/FALSE],
        - camera [TRUE/false],
        - basler [TRUE/false],
        - dummyball [true/FALSE],
        - debug [true/FALSE],
        - use_game_settings [true/FALSE]

**bitbots_vision_tools**
- *bitbots_vision_recordbag*: records a rosbag containing the camera image as image_raw and stores it as /tmp/tmpbag.bag


Color space files
-----------------

The vision depends on the usage of color space files which define color lookup tables primarily to detect the green field color and therefore lines and the field boundary.
These files are stored directly in the package directory in `/config/color_spaces/`.
We will provide a tool to generate these files from the camera stream in the future.

Currently, the Bit-Bots vision package supports two file types for color spaces:

- **`.yaml`**
  This format provides a human readable representation of color spaces.
  See below for a example representation.
  ```
  color_values:
        greenField:
            red: [r_1,r_2,..., r_n]
            green: [g_1,g_2,..., g_n]
            blue: [b_1,b_2,..., b_n]
    ```

  Assume the following RGB color value `(0, 153, 51)` is part of the color space.
  The correct representation is:
  ```
    r_i = 0
    g_i = 153
    b_i = 51
    ```

  For large color spaces, loading of such files takes a while.

- **`.pickle`**
  Generally, this is a generated binary representation of color spaces we use to prevent the long loading times of the `.yaml` format.
  We provide a script to generate these files (see [Vision Tools](#vision-tools)).


White Balancer
--------------

This repository also includes the `white_balancer`.
It is a ROS node that color-corrects incoming images with a predefined light temperature.


Vision Tools
------------

In the bitbots_vision_tools directory, special tools for debugging/introspection purposes are provided.

- **`/scripts/color_space_tool.py`**
  A small tool for color space enhancement.

  The tool is able to find main clusters in the color space, interpolate defined distances, add brightness thresholds and convert a yaml encoded to an pickle encoded color space.
  It also visualizes the color space in a browser based 3d graph.

  This tool provides a help page `-h` for further details.

- **`/scripts/color_space_subtract.py`**
  Another small tool for color space enhancement.

  This tool is able to subtracts color values from one color space file from another.

  For further details, see help page `-h`.

- **`/scripts/convert_to_image.py`**
  This is a small script to convert `ImageWithRegionOfInterest` of fcnn to an `Image` message for debug visualization.

- **`/scripts/imageclean.sh`**
  This is a small bash script, to quickly sort a directory of images using feh with shortcuts.
  Start this inside the directory of images to sort.
  
  Press key:
  - `1` -> move current image to **Trash** subdirectory
  - `2` -> Move current image to **Balls** subdirectory
  - `3` -> Move current image to **Goals** subdirectory

- **`/src/colorspace.cpp`**
  *LEGACY* Same as colorspace_tool.py above, but no longer maintained.

  To build, run: `g++ color_space_tool.cpp -l yaml-cpp`
