===========================================
Run Python 3 Packages in Python 2 Workspace
===========================================

The cv_bridge Issue
===================
In the default setting the workspace is only built using Python 2. Also usually ROS sopports only one Python version at a time.
But on the one hand we need python 2 for tools like rqt, but on the other software like the vision needs Python 3. If we want to build the cv_bridge which is needed for the Vision also in Python 3 we need to follow the following steps.

1. Execute the following commands, to add another workspace beside your main one:

:code:`mkdir ~/secondary_build_ws && cd ~/secondary_build_ws && catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so && catkin config --install`

2. Now we clone OpenCv:

:code:`mkdir src && cd src && git clone -b melodic https://github.com/ros-perception/vision_opencv.git`

3. We build the cv_bridge by running : :code:`catkin build cv_bridge`. Dont worry about the failing tests.
4. In the last step we source the workspace beside the old workspace using: :code:`--extend`.
    - For Bash users:
        Best practice is to add the sourcing to the .bashrc by adding :code:`source ~/secondary_build_ws/devel/setup.sh --extend`
    - For Zsh users:
        Best practice is to add the sourcing to the .bashrc by adding :code:`source ~/secondary_build_ws/devel/setup.zsh --extend`

The tf2 Issue
=============
In addition to the cv_bridge we may also want to build tf2 for Python 3. The solution is similar to the cv_bridge approach.
Instead of OpenCv clone :code:`clone https://github.com/ros/geometry2.git` and build it with: :code:`catkin build tf2`.
