# Testing the Robot

!!! note "TODO"

    Rework of the public documentation (see issue #1037): expand this page to cover all robot testing. It should:

    - Reference `starting_robot` (and `configure_launch`) for the warnings on when it is *not* safe to start the robot.
    - Describe starting motion standalone (see below).
    - Replace the `# TODO Start RL motions` placeholders with the actual RL motion commands (see `../development/rl_models`).
    - Describe starting teleop.
    - Link to `extrinsic_calibration` for the extrinsic calibration.

Make sure to test in order, since there are dependencies between some things.

## Test Motion in Visualization

1.  Test Animation:  
    ``` bash
    ros2 launch bitbots_animation_server test.launch sim:=true
    ros2 launch piplus_description standalone.launch js_pub:=false
    ros2 run bitbots_animation_server run_animation.py cheering
    ```

2.  Test Walk:  
    ``` bash
    ros2 launch bitbots_bringup motion_standalone.launch
    # TODO Start RL motions
    ros2 run bitbots_teleop teleop_keyboard.py
    ```

    Let the robot move around in all directions and also combinations.

## Test Motion on Robot

1.  Test Animation on Robot:  
    ``` bash
    ros2 launch bitbots_animation_server test.launch
    ros2 run bitbots_animation_server run_animation.py cheering
    ```

2.  Test Walk on Robot:  
    ``` bash
    ros2 launch bitbots_bringup motion_standalone.launch
    # TODO Start RL motions
    ros2 run bitbots_teleop teleop_keyboard.py
    ```

    Let the robot move around in all directions and also combinations.
