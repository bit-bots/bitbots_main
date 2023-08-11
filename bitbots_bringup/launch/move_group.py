import os

import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.utilities import normalize_to_list_of_substitutions
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, TextSubstitution, LaunchConfiguration
from launch.substitutions import TextSubstitution


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


def launch_setup(context, *args, **kwargs):
    sim = LaunchConfiguration("sim")
    fake_walk = LaunchConfiguration("fake_walk")
    sim_ns = LaunchConfiguration("sim_ns")
    robot_type = LaunchConfiguration("robot_type")

    robot_description = ParameterValue(Command(['xacro ',
                                                os.path.join(
                                                    get_package_share_directory(f"{robot_type.perform(context)}_description"),
                                                    "urdf",
                                                    "robot.urdf",
                                                ), " use_fake_walk:=", fake_walk, " sim_ns:=", sim_ns]),
                                       value_type=str)

    robot_description_semantic_config = load_file(f"{robot_type.perform(context)}_moveit_config",
                                                  f"config/{robot_type.perform(context)}.srdf")
    kinematics_yaml = load_yaml(f"{robot_type.perform(context)}_moveit_config", "config/kinematics.yaml")

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        f"{robot_type.perform(context)}_moveit_config", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)
    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        f"{robot_type.perform(context)}_moveit_config", "config/fake_controllers.yaml"
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

    # do some checks to provide better debug in case of errors in the configs
    if robot_description is None:
        print("### WARNING: robot_description is None")
    if robot_description_semantic_config is None:
        print("### WARNING: robot_description_semantic_config is None")
    if kinematics_yaml is None:
        print("### WARNING: kinematics_yaml is None")
    if ompl_planning_pipeline_config is None:
        print("### WARNING: ompl_planning_pipeline_config is None")
    if moveit_controllers is None:
        print("### WARNING: moveit_controllers is None")
    if trajectory_execution is None:
        print("### WARNING: trajectory_execution is None")
    if planning_scene_monitor_parameters is None:
        print("### WARNING: planning_scene_monitor_parameters is None")
    

    rsp_node = Node(package='robot_state_publisher',
                    executable='robot_state_publisher',
                    respawn=True,
                    # output='screen',
                    parameters=[{
                        'robot_description': robot_description,
                        'publish_frequency': 100.0,
                        'use_sim_time': sim
                    }],
                    arguments=['--ros-args', '--log-level', 'WARN']
                    )

    move_group_node = Node(package='moveit_ros_move_group',
                           executable='move_group',
                           # output='screen',
                           # hacky merging dicts
                           parameters=[{
                               'robot_description': robot_description,
                               'robot_description_semantic': robot_description_semantic_config,
                               'robot_description_kinematics': kinematics_yaml,
                               'publish_robot_description_semantic': True,
                               'use_sim_time': sim
                           },
                               ompl_planning_pipeline_config,
                               trajectory_execution,
                               moveit_controllers,
                               planning_scene_monitor_parameters, ],
                           arguments=['--ros-args', '--log-level', 'WARN']
                           )  # todo joint limits
    return [move_group_node, rsp_node]


def generate_launch_description():
    sim_arg = DeclareLaunchArgument('sim', default_value='False')
    fake_walk_arg = DeclareLaunchArgument('fake_walk', default_value='False')
    sim_ns_arg = DeclareLaunchArgument('sim_ns', default_value=TextSubstitution(text='/'))
    robot_type_arg = DeclareLaunchArgument('robot_type', default_value=TextSubstitution(text='wolfgang'))

    return LaunchDescription([sim_arg, fake_walk_arg, sim_ns_arg, robot_type_arg,
                              OpaqueFunction(function=launch_setup)])
