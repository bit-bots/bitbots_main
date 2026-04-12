<h1 align="center">
   <img src="./images/Picto+STEREOLABS_Black.jpg" alt="Stereolabs" title="Stereolabs" /><br \>
</h1>

# ROS 2 Interfaces package for ZED Cameras

The `zed-ros2-interfaces` repository installs the `zed_msgs` ROS 2 package which defines the custom topics, services, and actions used by the packages defined in the repositories  [ZED RO 2 Wrapper](https://github.com/stereolabs/zed-ros2-wrapper) and [ZED RO 2 Examples](https://github.com/stereolabs/zed-ros2-examples).

**Note:** This package does not require CUDA and can be used to receive ZED data on ROS 2 machines without an NVIDIA GPU.

## Install the package from the binaries for ROS 2 Humble

The package `zed_msgs` is available in binary format in the official Humble repository.

`sudo apt install ros-humble-zed-msgs`

## Install the package from the source code

You can install the `zed_msgs` package from the source code to obtain the latest updates or for distributions other than Humble (e.g. ROS 2 Foxy).

### Build the repository

#### Dependencies

The `zed_msgs` is a colcon package. It depends on the following ROS 2 packages:

- ament_cmake_auto
- builtin_interfaces
- std_msgs
- geometry_msgs
- shape_msgs
- rosidl_default_generators
- rosidl_default_runtime
- ament_lint_auto
- ament_cmake_copyright
- ament_cmake_cppcheck
- ament_cmake_lint_cmake
- ament_cmake_pep257
- ament_cmake_uncrustify
- ament_cmake_xmllint

#### Clone and build

Open a terminal, clone the repository, update the dependencies, and build the packages:

```bash
cd ~/catkin_ws/src
git clone https://github.com/stereolabs/zed-ros2-interfaces.git
cd ../
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
echo source $(pwd)/install/local_setup.bash >> ~/.bashrc
source ~/.bashrc
```

> :pushpin: **Note**: If the command `rosdep` is missing, you can install it using the following method:
>
> `sudo apt-get install python-rosdep python-rosinstall-generator python-vcstool python-rosinstall build-essential`
>
> :pushpin: **Note**: The option `--symlink-install` is important, it allows using symlinks instead of copying files to the ROS 2 folders during the installation, where possible. Each package in ROS 2 must be installed and all the files used by the > nodes must be copied into the installation folders. Using symlinks allows you to modify them in your workspace, reflecting the modification during the next executions without the needing to issue a new colcon build command. This is true only for > all the files that don't need to be compiled (Python scripts, configurations, etc.).
>
> :pushpin: **Note**: If you are using a different console interface like `zsh`, you have to change the `source` command as follows: `echo source $(pwd)/install/local_setup.zsh >> ~/.zshrc and source ~/.zshrc`.

## Custom Topics

- [BoundingBox2Df](https://github.com/stereolabs/zed-ros2-interfaces/blob/master/msg/BoundingBox2Df.msg)
- [BoundingBox2Di](https://github.com/stereolabs/zed-ros2-interfaces/blob/master/msg/BoundingBox2Di.msg)
- [BoundingBox3D](https://github.com/stereolabs/zed-ros2-interfaces/blob/master/msg/BoundingBox3D.msg)
- [DepthInfoStamped](https://github.com/stereolabs/zed-ros2-interfaces/blob/master/msg/DepthInfoStamped.msg)
- [GnssFusionStatus](https://github.com/stereolabs/zed-ros2-interfaces/blob/master/msg/GnssFusionStatus.msg)
- [HealthStatusStamped](https://github.com/stereolabs/zed-ros2-interfaces/blob/master/msg/HealthStatusStamped.msg)
- [Heartbeat](https://github.com/stereolabs/zed-ros2-interfaces/blob/master/msg/Heartbeat.msg)
- [Keypoint2Df](https://github.com/stereolabs/zed-ros2-interfaces/blob/master/msg/Keypoint2Df.msg)
- [Keypoint2Di](https://github.com/stereolabs/zed-ros2-interfaces/blob/master/msg/Keypoint2Di.msg)
- [Keypoint3D](https://github.com/stereolabs/zed-ros2-interfaces/blob/master/msg/Keypoint3D.msg)
- [MagHeadingStatus](https://github.com/stereolabs/zed-ros2-interfaces/blob/master/msg/MagHeadingStatus.msg)
- [Object](https://github.com/stereolabs/zed-ros2-interfaces/blob/master/msg/Object.msg)
- [ObjectsStamped](https://github.com/stereolabs/zed-ros2-interfaces/blob/master/msg/ObjectsStamped.msg)
- [PlaneStamped](https://github.com/stereolabs/zed-ros2-interfaces/blob/master/msg/PlaneStamped.msg)
- [PosTrackStatus](https://github.com/stereolabs/zed-ros2-interfaces/blob/master/msg/PosTrackStatus.msg)
- [Skeleton2D](https://github.com/stereolabs/zed-ros2-interfaces/blob/master/msg/Skeleton2D.msg)
- [Skeleton3D](https://github.com/stereolabs/zed-ros2-interfaces/blob/master/msg/Skeleton3D.msg)
- [SvoStatus](https://github.com/stereolabs/zed-ros2-interfaces/blob/master/msg/SvoStatus.msg)

You can get more information by reading the [Stereolabs online documentation](https://www.stereolabs.com/docs/ros2/zed-node/)

## Custom Services

- [SetPose](https://github.com/stereolabs/zed-ros2-interfaces/blob/master/srv/SetPose.srv)
- [SetROI](https://github.com/stereolabs/zed-ros2-interfaces/blob/master/srv/SetROI.srv)
- [StartSvoRec](https://github.com/stereolabs/zed-ros2-interfaces/blob/master/srv/StartSvoRec.srv)
- [SetSvoFrame.srv](https://github.com/stereolabs/zed-ros2-interfaces/blob/master/srv/SetSvoFrame.srv)
- [SaveAreaMemory.srv](https://github.com/stereolabs/zed-ros2-interfaces/blob/master/srv/SaveAreaMemory.srv)

You can get more information by reading the [Stereolabs online documentation](https://www.stereolabs.com/docs/ros2/zed-node/#services)

