# Bit-Bots Vision

[![CodeFactor](https://www.codefactor.io/repository/github/bit-bots/bitbots_vision/badge)](https://www.codefactor.io/repository/github/bit-bots/bitbots_vision)
&nbsp;&nbsp;
&nbsp;&nbsp;
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

This is the vision ROS package of the Hamburg Bit-Bots.

The vision is able to detect balls, lines, the field itself, the field boundary, goal posts, teammates, enemies and other obstacles.

## Fixed exposure with automatic gain

The ZED Mini couples its hardware auto-exposure and auto-gain controls. The
bringup package therefore provides an optional software gain controller that
keeps exposure fixed and adjusts `video.gain` from the mean image brightness.

```bash
pixi run -e default ros2 launch bitbots_bringup vision_standalone.launch \
  fixed_exposure_auto_gain:=true camera_exposure:=20
```

The controller defaults are in
`config/fixed_exposure_auto_gain.yaml`. In particular,
`target_brightness`, `brightness_deadband`, and `gain_kp` control its response.

For further information and getting started, see the documentation of this package at our website: [docs.bit-bots.de](https://docs.bit-bots.de/package/bitbots_vision/latest/index.html)

An earlier version of this pipeline is presented in our paper [An Open Source Vision Pipeline Approach for RoboCup Humanoid Soccer](https://robocup.informatik.uni-hamburg.de/wp-content/uploads/2019/06/vision_paper.pdf).
When you use this pipeline or parts of it, please cite it.

```bibtex
@inproceedings{vision2019,
author={Fiedler, Niklas and Brandt, Hendrik and Gutsche, Jan and Vahl, Florian and Hagge, Jonas and Bestmann, Marc},
year={2019},
title={An Open Source Vision Pipeline Approach for RoboCup Humanoid Soccer},
booktitle={RoboCup 2019: Robot World Cup XXIII},
note = {Accepted},
organization={Springer}
}
```
