from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    parameter_file = PathJoinSubstitution([
        FindPackageShare('plan_env'), 'config',
        'gemini336_mapping_experiment.yaml'])
    planner_node = Node(
        package='plan_env',
        executable='observation_planner_node',
        name='plan_env_observation_planner',
        output='screen',
        parameters=[parameter_file],
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('plan_env'), 'config', 'gemini336_mapping.rviz'])),
        Node(
            package='plan_env',
            executable='gemini336_depth_scene.py',
            name='gemini336_depth_scene',
            output='screen',
            parameters=[parameter_file],
        ),
        Node(
            package='plan_env',
            executable='grid_map_standalone_test',
            name='grid_map_standalone_test',
            output='screen',
            parameters=[parameter_file],
        ),
        planner_node,
        Node(
            package='plan_env',
            executable='gemini336_mapping_evaluator.py',
            name='gemini336_mapping_evaluator',
            output='screen',
            parameters=[parameter_file],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            output='screen',
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=planner_node,
                on_exit=[Shutdown(reason='observation planner completed')],
            )
        ),
    ])
