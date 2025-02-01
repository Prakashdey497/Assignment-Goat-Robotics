import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    amr_description = get_package_share_directory("amr_description")
    amr_description_prefix = get_package_prefix("amr_description")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")

    # Declare model argument for the URDF file
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(
            amr_description, 
            "urdf", 
            "amr.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )

    # world_name = LaunchConfiguration("world_name")

    world_name_arg = DeclareLaunchArgument(name="world_name", 
                                           default_value="empty")


    world_path = PathJoinSubstitution([
            amr_description,
            "worlds",
            PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
        ]
    )

    # Set GAZEBO_MODEL_PATH environment variable
    model_path = os.path.join(amr_description, "models")
    model_path += pathsep + os.path.join(amr_description_prefix, "share")
    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    # Set robot description parameter
    robot_description = ParameterValue(Command(["xacro ", 
                                                LaunchConfiguration("model")]),
                                       value_type=str)

    # Define the robot_state_publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    # Include the Gazebo server with the specified world file
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": [world_path]}.items(),
    )

    # Include the Gazebo client
    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzclient.launch.py")
        )
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "softbot",
            "-topic", "robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.0",
            "-R", "0.0",
            "-P", "0.0",
            "-Y", "0.0"
        ],
        output="screen"
    )

    # Return the launch description with all launch actions
    return LaunchDescription([
        env_var,
        model_arg,
        world_name_arg,
        start_gazebo_server,
        start_gazebo_client,
        robot_state_publisher_node,
        spawn_robot
    ])