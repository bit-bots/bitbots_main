import os

import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, TextSubstitution, LaunchConfiguration


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    fake_walk = LaunchConfiguration("fake_walk")
    sim_ns = LaunchConfiguration("sim_ns")

    fake_walk_arg = DeclareLaunchArgument('fake_walk', default_value='False')

    sim_ns_arg = DeclareLaunchArgument('sim_ns', default_value='/')

    robot_description = ParameterValue(Command(['xacro ',
                                                os.path.join(
                                                    get_package_share_directory("wolfgang_description"),
                                                    "urdf",
                                                    "robot.urdf",
                                                ), " use_fake_walk:=", fake_walk, " sim_ns:=", sim_ns]),
                                       value_type=str)

    robot_description_semantic_config = load_file("wolfgang_moveit_config", "config/wolfgang.srdf")
    kinematics_yaml = load_yaml("wolfgang_moveit_config", "config/kinematics.yaml")

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        "wolfgang_moveit_config", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        "wolfgang_moveit_config", "config/fake_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    rsp_node = Node(package='robot_state_publisher',
                    executable='robot_state_publisher',
                    respawn=True,
                    #output='screen',
                    parameters=[{
                        'robot_description': robot_description,
                        'publish_frequency': 1000.0
                    }],
                    arguments=['--ros-args', '--log-level', 'WARN']
                    )

    move_group_node = Node(package='moveit_ros_move_group',
                           executable='move_group',
                           #output='screen',
                           # hacky merging dicts
                           parameters=[{
                               'robot_description': robot_description,
                               'robot_description_semantic': robot_description_semantic_config,
                               'robot_description_kinematics': kinematics_yaml,
                               'publish_robot_description_semantic': True,
                           },
                               ompl_planning_pipeline_config,
                               trajectory_execution,
                               moveit_controllers,
                               planning_scene_monitor_parameters, ],
                           arguments=['--ros-args', '--log-level', 'WARN']
                           ) #todo joint limits

    return LaunchDescription([fake_walk_arg, sim_ns_arg, move_group_node, rsp_node])
