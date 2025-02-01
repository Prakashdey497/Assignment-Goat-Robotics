import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():

    amr_navigation_dir = get_package_share_directory("amr_navigation")

    # softbot Navigation parameter file
    default_nav2_param_file = os.path.join(amr_navigation_dir,"config","nav2_dwb_params.yaml")

    # softbot default map yaml file
    default_map_file = os.path.join(get_package_share_directory("amr_mapping"),"maps","small_house","map.yaml")

    use_rviz = LaunchConfiguration("use_rviz")
    map_path = LaunchConfiguration("map_path")
    use_sim_time = LaunchConfiguration('use_sim_time')
    navigation_param_file = LaunchConfiguration('navigation_param_file')



    declare_use_sim_time_cmd = DeclareLaunchArgument(name='use_sim_time',
                                                     default_value='true',
                                                     description='Use simulation (Gazebo) clock if true')
    
    declare_use_rviz_arg = DeclareLaunchArgument(name="use_rviz",
                                                default_value="true",
                                                description = "use rviz2 or not!")
    
    declare_map_file_path_arg = DeclareLaunchArgument(name="map_path",
                                                default_value=default_map_file,
                                                description = "map yaml file path!")
    
    declare_nav_param_file_path_arg = DeclareLaunchArgument(name="navigation_param_file",
                                                default_value=default_nav2_param_file,
                                                description = "Nav2 Param File!")
    

    robot_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'),'launch', 'bringup_launch.py')),
        launch_arguments={
                'map': map_path , #'src/softbot_mapping/maps/office_map.yaml',
                'use_sim_time': use_sim_time,
                'params_file': navigation_param_file}.items()
    )

    rviz_node_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(amr_navigation_dir, "rviz", "navigation.rviz")],
        condition=IfCondition(use_rviz)
    )


    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_arg)
    ld.add_action(declare_map_file_path_arg)
    ld.add_action(declare_nav_param_file_path_arg)
    ld.add_action(robot_navigation_cmd)
    ld.add_action(rviz_node_cmd)

    return ld