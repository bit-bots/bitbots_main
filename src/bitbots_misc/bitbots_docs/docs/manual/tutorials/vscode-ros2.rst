======================
Setup VSCode with ROS2
======================

Open a terminal.
Navigate to your bitbots_main workspace.

Activate the pixi environment:

`pixi shell`

Open VSCode

`code .`

Install the `Robotics Developer Extension <https://marketplace.visualstudio.com/items?itemName=Ranch-Hand-Robotics.rde-pack>`_.
You should see a `ROS2.jazzy` in the lower left corner.

Now you should be able to build the code with `Ctrl+Shift+B`

You can use multiple commands with `Ctrl+Shift+P` and then type `ROS`.

Debugging
~~~~~~~~~
You can debug launch files but only ones that are written in Python.
These cannot include further xml launch files.
It makes sense to create a small test launch with only the node that you want to debug and start the rest independently.
An example vscode launch configuration and the corresponding python launch file can be seen below:

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
                "target": "$COLCON_WS/src/bitbots_head_behavior/share/bitbots_head_behavior/launch/test.py",
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
                        "base_footprint_frame": "base_footprint"
                    }],
                remappings=[("/head_motor_goals", "/DynamixelController/command")]
                )

        return LaunchDescription([node])
