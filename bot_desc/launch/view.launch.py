import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
#import launch.actions 
import launch

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    

    package_name = 'bot_desc'

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('bot_desc'))
    xacro_file = os.path.join(pkg_path,'description_gps','robot.urdf.xacro')

    robot_description_config = Command(['xacro ', xacro_file])


    default_rviz_config_path = os.path.join(get_package_share_directory(package_name), 'config', 'bot1.rviz')


    world_path = os.path.join(get_package_share_directory('bot_desc'), 'worlds' , 'tree.sdf')
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
   
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
        remappings=remappings,
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[params]

    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        arguments=['-entity', 'rover', '-topic', 'robot_description'],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[LaunchConfiguration('use_sim_time')],
        arguments=['-d', LaunchConfiguration('rvizconfig')])


    # Launch!
    return LaunchDescription([

        launch.actions.ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
            output='screen'),

        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                             description='Absolute path to rviz config file'),


        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        

        node_robot_state_publisher,
        joint_state_publisher_node,
        spawn_entity,
        #rviz_node
    ])
