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

Debugging
~~~~~~~~~
You can debug launch files but only ones that are written in Python. 
These can not include further xml lauch files. 
It makes sense to create a small test lauch with only the node that you wantto debug and start the rest independently.
An example vscode launch configuration and the corresponding python lauch file can be seen below:

.. code-block:: json

    {
        // Use IntelliSense to learn about possible attributes.
        // Hover to view descriptions of existing attributes.
        // For more information, visit: https://go.microsoft.com/fwlink/?linkid=830387
        "version": "0.2.0",
        "configurations": [
            {
                "name": "head behavior",
                "request": "launch",
                "target": "$HOME/ros2_ws/install/bitbots_head_behavior/share/bitbots_head_behavior/launch/test.py",
                "launch": "[rviz, gz, gzclient, gzserver]",
                "type": "ros"
            }
        ]
    }

.. code-block:: python

    import os
    from ament_index_python import get_package_share_directory
    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        body_config = os.path.join(get_package_share_directory('bitbots_body_behavior'), 'config', 'body_behavior.yaml')
        head_config = os.path.join(get_package_share_directory('bitbots_head_behavior'), 'config', 'head_config.yaml') 

        node = Node(
                package='bitbots_head_behavior',
                namespace='',
                executable='head_node',
                parameters=[
                    body_config,
                    head_config,
                    {
                        "camera_frame": "camera",
                        "base_link_frame": "base_link",
                        "odom_frame": "odom",
                        "map_frame": "map",
                        "ball_frame": "ball",
                        "ball_approach_frame": "ball_approach_frame",
                        "base_footprint_frame": "base_footprint"
                    }],
                remappings=[("/head_motor_goals", "/DynamixelController/command")]
                )

        return LaunchDescription([node])


