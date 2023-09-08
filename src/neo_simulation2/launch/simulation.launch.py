# Neobotix GmbH
# Author: Pradheep Padmanabhan

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration, Command
from launch_ros.actions import Node
import os
from pathlib import Path
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch_ros.parameter_descriptions import ParameterValue

import xacro

MY_NEO_ENVIRONMENT = 'neo_track1' # or 'neo_track1' or 'neo_track2'

def generate_launch_description():
    default_world_path = os.path.join(get_package_share_directory('neo_simulation2'), 'worlds', MY_NEO_ENVIRONMENT + '.world')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # robot_dir = LaunchConfiguration(
    #     'robot_dir',
    #     default=os.path.join(get_package_share_directory('neo_simulation2'),
    #         'robots/mpo_700/meshes/descriptions/awcombo_description/urdf/',
    #         'awcombo_converted'+'.urdf'))
    
    xacro_file = os.path.join(get_package_share_directory('awcombo_description'), 'urdf/', 'awcombo'+ '.xacro')
    
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')

    # urdf = os.path.join(get_package_share_directory('awcombo_description'), 'urdf/', 'awcombo_converted'+ '.xacro.urdf')

    # spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',arguments=['-entity', "awcombo", '-file', urdf], output='screen')
    # controller_file = os.path.join(get_package_share_directory('neo_simulation2'), 'configs/mpo_700/awcombo.ros2_controllers.yaml')
    
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',arguments=['-entity', "awcombo", '-topic', 'robot_description'], output='screen')

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': doc.toxml()}])
        # arguments=[urdf]) 
        
        # arguments=[urdf, controller_file])
    # )

    teleop =  Node(package='teleop_twist_keyboard',executable="teleop_twist_keyboard",
    output='screen',
    prefix = 'xterm -e',
    name='teleop')

    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': default_world_path,
                'verbose': 'true',
                
            }.items()
        )
    
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'CSP'],
        output='screen'
    )

    return LaunchDescription([
                            #   teleop,
                               RegisterEventHandler(
                                event_handler=OnProcessExit(
                                    target_action=spawn_entity,
                                    on_exit=[load_joint_state_controller],
                                )
                            ),
                            RegisterEventHandler(
                                event_handler=OnProcessExit(
                                    target_action=load_joint_state_controller,
                                    on_exit=[load_joint_trajectory_controller],
                                )
                            ),
                              gazebo,
                              start_robot_state_publisher_cmd,
                              spawn_entity
                              ])