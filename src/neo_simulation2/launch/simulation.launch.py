
import yaml
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
from launch.conditions import IfCondition

from launch_ros.parameter_descriptions import ParameterValue

import xacro


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


# ------ Robot semantic ------
robot_description_semantic_config = load_file(
    'awcombo_moveit_config', 'config/awcombo.srdf')
robot_description_semantic = {
    'robot_description_semantic': robot_description_semantic_config}

# ------ Kinematics and limits ------
kinematics_yaml = load_yaml('awcombo_moveit_config', 'config/kinematics.yaml')
joint_limits_yaml = load_yaml(
    'awcombo_moveit_config', 'config/joint_limits.yaml')
robot_description_kinematics = {
    'robot_description_kinematics': kinematics_yaml}
joint_limits = {'robot_description_planning': joint_limits_yaml}

# ------ Rviz ------ì

# ------ Moveit planning pipeline -------
# ompl_planning_pipeline_config = {
#     'move_group': {
#         # 'planning_plugin': 'ompl_interface/OMPLPlanner',
#         # 'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
#         #                    'default_planner_request_adapters/ResolveConstraintFrames '
#         #                    'default_planner_request_adapters/FixWorkspaceBounds '
#         #                    'default_planner_request_adapters/FixStartStateBounds '
#         #                    'default_planner_request_adapters/FixStartStateCollision '
#         #                    'default_planner_request_adapters/FixStartStatePathConstraints',
#         # 'start_state_max_bounds_error': 0.1
#         'planning_plugin': 'pilz_industrial_motion_planner::CommandPlanner',
#         'request_adapters': 'pilz_industrial_motion_planner::CommandPlanner',
#         'start_state_max_bounds_error': 0.1
#     }
# }
# ompl_planning_yaml = load_yaml(
#     'awcombo_moveit_config', 'config/ompl_planning.yaml'
# )
# ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)
# Alternatively: Pilz motion planner
pilz_planning_pipeline_config = {
    'move_group': {
        'planning_plugin': 'pilz_industrial_motion_planner/CommandPlanner',
        'capabilities': 'pilz_industrial_motion_planner/MoveGroupSequenceAction '
                        'pilz_industrial_motion_planner/MoveGroupSequenceService',
        'start_state_max_bounds_error': 0.1
    }
}
simple_controllers_yaml = load_yaml(
    'awcombo_description', 'config/awcombo.ros2_controllers.yaml'
)
moveit_controllers = {
    'moveit_controller_manager': 'moveit_simple_controller_manager'
    '/MoveItSimpleControllerManager',
    'moveit_simple_controller_manager': simple_controllers_yaml,
}
trajectory_execution = {
    # 'moveit_manage_controllers': True,
    'trajectory_execution.allowed_execution_duration_scaling': 1.2,
    'trajectory_execution.allowed_goal_duration_margin': 0.5,
    'trajectory_execution.allowed_start_tolerance': 0.01,
}
planning_scene_monitor_parameters = {
    'publish_planning_scene': True,
    'publish_geometry_updates': True,
    'publish_state_updates': True,
    'publish_transforms_updates': True,
}

MY_NEO_ENVIRONMENT = 'neo_track1'  # or 'neo_track1' or 'neo_track2'


def generate_launch_description():
    default_world_path = os.path.join(get_package_share_directory(
        'neo_simulation2'), 'worlds', MY_NEO_ENVIRONMENT + '.world')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    xacro_file = os.path.join(get_package_share_directory(
        'awcombo_description'), 'urdf/', 'awcombo' + '.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=[
                        '-entity', "awcombo", '-topic', 'robot_description'], output='screen')

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    robot_description])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                'gazebo_ros'), 'launch', 'gazebo.launch.py')
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

    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits,
            # ompl_planning_pipeline_config, # Alternatively: pilz_planning_pipeline_config
            pilz_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters
        ]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output={'both': 'log'},
        # arguments=['-d', rviz_full_config],
        # condition=IfCondition(LaunchConfiguration('rviz_on')),
        parameters=[
            robot_description,
            robot_description_semantic,
            # ompl_planning_pipeline_config, # Alternatively: pilz_planning_pipeline_config
            pilz_planning_pipeline_config,
            kinematics_yaml,
        ],
        emulate_tty=True
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
        spawn_entity,
        run_move_group_node,
        rviz_node,
        # rviz_on
    ])
