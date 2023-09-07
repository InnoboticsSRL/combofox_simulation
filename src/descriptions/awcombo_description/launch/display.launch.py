import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    xacro_dir = os.path.join(get_package_share_directory('awcombo_description'), 'urdf', 'awcombo.xacro')
    rviz_config_dir = os.path.join(get_package_share_directory('awcombo_description'), 'config', 'configuration.rviz')

    rviz_on = DeclareLaunchArgument(name='rviz_on', default_value='true', choices=['true', 'false'],
                                        description='Flag to launch rviz')
    joint_states = DeclareLaunchArgument(name='joint_states', default_value='false', choices=['true', 'false'],
                                        description='Flag to see the joint state publisher pop-up')

    # Xacro reading
    robot_description = ParameterValue(Command(['xacro', ' ', xacro_dir]),
                                       value_type=str)
    
    # Nodes
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir],
        condition=IfCondition(LaunchConfiguration('rviz_on'))
    )

    robot_state_pub = Node(  
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('joint_states'))
    )
    
    joint_state_pub_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('joint_states'))
    )

    return LaunchDescription([
        rviz_on,
        joint_states,
        rviz2,
        robot_state_pub,
        joint_state_pub,
        joint_state_pub_gui
    ])
