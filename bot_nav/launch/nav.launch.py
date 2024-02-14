#! /usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node


def generate_launch_description():

    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    bot_desc_dir = get_package_share_directory("bot_desc")
    bot_nav_dir = get_package_share_directory("bot_nav")

    launch_dir = os.path.join(bot_desc_dir, 'launch')
    params_dir = os.path.join(bot_nav_dir, "config")
    nav2_params = os.path.join(params_dir, "nav2_no_map_params.yaml")

    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )

    use_rviz = LaunchConfiguration('use_rviz')
    

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Whether to start RVIZ')


    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'view.launch.py'))
    )

    server_conv = Node(
        package="bot_nav",
        executable="server_conv.py",
    )

    # robot_localization_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(launch_dir, 'dual_ekf_navsat.launch.py'))
    # )

    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "True",
            "params_file": configured_params,
            "autostart": "True",
        }.items(),
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", 'rviz_launch.py')),
        condition=IfCondition(use_rviz)
    )

    

    # Create the launch description and populate
    ld = LaunchDescription()

    # simulator launch
    ld.add_action(gazebo_cmd)

    

    # navigation2 launch
    ld.add_action(navigation2_cmd)

    # rviz launch
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(server_conv)


    return ld
