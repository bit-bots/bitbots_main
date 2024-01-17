============================
PyCharm with ROS Integration
============================

Setup Package Path
===================
When following these steps, ROS-packages will be added to your PYTHONPATH.
This leads to PyCharm indexing them and thus you can use auto completion.
If you use `Source ROS`, this is not necessary.

1. Settings --> Project:bitbots_meta --> Project Interpreter
2. Cog at the top right next to the Interpreter --> Show All
3. Lowest button on the right side ("Show paths for the selected interpreter")
4. Add --> `/opt/ros/melodic/lib/python3/dist-packages`
5. Add --> `<catkin_ws>/devel/lib/python3/dist-packages`

`Source ROS`
============
When using this method, ROS will be sourced when starting PyCharm.
This means all ROS-commands and paths are known to PyCharm.
This means PyCharm knows just as much as your Shell and can theoretically do the same things.

1. ::

    cp /usr/share/applications/pycharm-professional.desktop ~/.local/share/applications/pycharm-with-ros.desktop

2. ::

    vim ~/.local/share/applications/pycharm-with-ros.desktop

3. Change the name, otherwise you overwrite the global PyCharm starter.

4. ::

    Exec=bash -c "source <catkin_ws>/devel/setup.bash; /usr/bin/pycharm %f"

Change Launch-files
===================
To test/debug nodes in an efficient way with PyCharm, you can add the argument ``depends_only`` to your launch-file.
This means, only the node you want to test will be started.

After you have started the node externally, using ``roslaunch``, PyCharm will be able to start this node and you can debug.
