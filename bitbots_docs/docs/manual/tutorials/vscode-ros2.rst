======================
Setup VSCode with ROS2
======================

Open a terminal.
Navigate to your colcon workspace.
`cd ros2_ws`
Source ros
`source /opt/ros/rolling/setup.zsh`
Open VSCode
`code .`
Install the ROS extension (from Microsoft).
You should see a `ROS2.rolling` in the lower left corner.

Now you should be able to build the code with `Ctrl+Shift+B`
You can use muliple commands with `Cntrl+Shift+P` and then type `ROS`.
You can debug launch files but only ones that are written in Python. These can not include further xml lauch files. It makes sense to create a small test lauch with only the node that you wantto debug and start the rest independently.
