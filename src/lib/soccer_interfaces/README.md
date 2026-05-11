# soccer_interfaces
A set of packages which contain common soccer interface files

[![Build and Test (humble)](../../actions/workflows/build_and_test_humble.yaml/badge.svg?branch=humble)](../../actions/workflows/build_and_test_humble.yaml?query=branch:humble)
[![Build and Test (jazzy)](../../actions/workflows/build_and_test_jazzy.yaml/badge.svg?branch=rolling)](../../actions/workflows/build_and_test_jazzy.yaml?query=branch:rolling)
[![Build and Test (rolling)](../../actions/workflows/build_and_test_rolling.yaml/badge.svg?branch=rolling)](../../actions/workflows/build_and_test_rolling.yaml?query=branch:rolling)

## Installation

### Binary Installation

Binary installation is available. Source your ROS installation, then run:

```
sudo apt install ros-${ROS_DISTRO}-soccer-interfaces
```

### Source Installation

Alternatively to build from source, source your ROS installation, then run the following in your ROS workspace:

```
// For ROS 2 Iron / Rolling
git clone https://github.com/ros-sports/soccer_interfaces.git src/soccer_interfaces
colcon build

// For ROS 2 Humble
git clone https://github.com/ros-sports/soccer_interfaces.git src/soccer_interfaces --branch humble
colcon build
```

## Documentation

For documentation, see [Soccer Interfaces](https://soccer-interfaces.readthedocs.io/en/latest/).
