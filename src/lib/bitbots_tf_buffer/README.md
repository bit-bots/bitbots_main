# Bit-Bots TF Buffer

This is a nearly drop-in replacement for `tf2_ros.Buffer` in Python. It wraps a C++ node that holds the tf buffer and listener. The interface should be the same as the original `tf2_ros.Buffer` and `tf2_ros.TransformListener`, except for e.g. qos settings that are not supported for now.

## Why?

Sadly rclpy is very slow.
This is not only a Python problem, but also a problem of the underlying rclpy implementation.

TF2 is used in many nodes.
The amount of callbacks that are triggered by TF2 can be very high, especially if there are multiple sources of TF data and you operate on a high frequency. A simple ROS 2 rclpy node can easily max out a CPU core just by processing TF data. This is especially unlucky, if TF is only used at a few places in the code for e.g. low frequency navigation or behavior operations.

This package aims to solve this problem by moving the TF buffer and listener to a C++ node that shares the process with the rclpy node. This way, the TF callbacks are processed in C++ and the Python node only needs to query the buffer for the latest transforms using a simple and performant pybind11 interface when needed.

While spinning up an additional Node is not ideal, the performance gain is significant and the overhead of the additional node is negligible compared to the performance gain.

In addition to that, this solution also reduces the amount of executor deadlock scenarios and enables the usage of an single threaded executor for the rclpy node instead of the multi-threaded executor in many cases resulting in further performance gains.

## Usage

Replace `from tf2_ros import Buffer, TransformListener` with `from bitbots_tf_buffer import
Buffer, TransformListener`.

## Installation

First we need to clone the repository as well as one of its dependencies:

```bash
cd <replace_with_your_ws>/src
git clone git@github.com:bit-bots/bitbots_tf_buffer.git
git clone git@github.com:bit-bots/ros2_python_extension.git
```

Then run rosdep to install the dependencies (most likely you already have them installed):

```bash
rosdep update
rosdep install --from-paths . --ignore-src -r -y
```

Now we can build and use the package:

```bash
colcon build
```
