# Community Examples

> You've written some beautiful launch files and would like to share them with the community? This is the place to collect and find them! 


## A turtlesim with pixi
**What:** A working `better_launch` + turtlesim example based on pixi, which makes it super easy to set up an isolated ROS/Python workspace.
**Where:** https://github.com/kywch/turtlesim-pixi
**Who:** @kywch

### Instructions
The turtlesim and teleop work out of the box by running `pixi run turtlesim` and `pixi run teleop`.

You can run the examples with following commands:

```sh
# Solve and install all dependencies in the virtual environment
pixi shell

# If the install is successful, the venv is activated
# Go to the ROS workspace
cd src
pixi run build

# Test installation
pixi run bl

# Run the turtlesim example
pixi run bl better_launch 08_better_turtlesim.launch.py
```
