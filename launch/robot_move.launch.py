import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'gps_robot'
    file_subpath = 'description/robot.urdf.xacro'

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()


    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] 

    )


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )


    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'my_robot'],
                    output='screen')
    
    rviz_node = Node(package='rviz2' , executable='rviz2',output='screen')

    #cmd_vel_node = Node(package ='gps_robot',executable='cmd_vel',name='cmd_vel_node')

    position_drive_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['position_controller'],
    )


    velocity_drive_node=Node(
        package = 'controller_manager',
        executable='spawner',
        arguments=['velocity_controller']
    )
    joint_broad_spawner = Node(
        package = 'controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster']
    )




    # Run the node
    return LaunchDescription([
        node_robot_state_publisher,
    
        joint_state_publisher_node,
        joint_broad_spawner,
        rviz_node,
        gazebo,
        spawn_entity,
        #cmd_vel_node,
        
        velocity_drive_node,
        position_drive_node,
        
    ])

