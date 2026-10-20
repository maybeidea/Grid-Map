"""Shared synthetic/real mapping, observation and export test entry point."""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    share = get_package_share_directory('plan_env')
    mode = value('mode')
    if mode not in ('sim', 'real'):
        raise ValueError('mode must be sim or real')
    simulated = mode == 'sim'
    params = value('params_file') or os.path.join(
        share, 'config', 'gemini336_mapping_experiment.yaml' if simulated
        else 'real_mapping_test.yaml')
    with open(params, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)

    def node_parameters(name, overrides):
        # Resolve the shared and named sections before adding launch overrides.
        # Humble otherwise lets a named YAML section beat a later /** override.
        merged = {}

        def flatten(values, prefix=''):
            for key, item in values.items():
                path = prefix + key
                if isinstance(item, dict):
                    flatten(item, path + '.')
                else:
                    merged[path] = item

        for scope in ('/**', name, '/' + name):
            flatten(config.get(scope, {}).get('ros__parameters', {}))
        merged.update(overrides)
        return [merged]
    output = value('output_dir') or (
        '/tmp/plan_env_gemini336' if simulated else '/tmp/plan_env_real')
    shared = {'use_sim_time': value('use_sim_time').lower() == 'true'}
    for argument, parameter in (
            ('cloud_topic', 'grid_map.cloud_topic'),
            ('map_frame', 'grid_map.frame_id'),
            ('motion_frame', 'grid_map.motion_frame')):
        if value(argument):
            shared[parameter] = value(argument)
    if value('cloud_reliable'):
        shared['grid_map.cloud_reliable'] = value('cloud_reliable').lower() == 'true'
    exporter = dict(shared, visualization_dir=output)
    if value('task_topic'):
        exporter['task_topic'] = value('task_topic')
    if value('record_bag'):
        exporter['record_bag'] = value('record_bag').lower() == 'true'
    exporter['bag_directory'] = os.path.join(output, 'bag')
    planner = dict(shared)
    planner['observation.navigation_enabled'] = not simulated
    planner['observation.exit_on_complete'] = simulated
    if value('task_topic'):
        planner['observation.task_topic'] = value('task_topic')
    if value('navigate_action'):
        planner['navigate_action'] = value('navigate_action')
    planner_node = Node(
        package='plan_env', executable='observation_planner_node',
        name='plan_env_observation_planner', output='screen',
        condition=IfCondition(LaunchConfiguration('use_observation_planner')),
        parameters=node_parameters('plan_env_observation_planner', planner))
    nodes = [
        Node(package='plan_env', executable='grid_map_standalone_test',
             name='grid_map_standalone_test', output='screen',
             parameters=node_parameters('grid_map_standalone_test', exporter)),
        planner_node,
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', value('rviz_config')],
             parameters=[{'use_sim_time': shared['use_sim_time']}],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ]
    if simulated:
        scene = {
            'use_sim_time': shared['use_sim_time'],
            'observation_planner_enabled': value('use_observation_planner').lower() == 'true',
        }
        if value('map_frame'):
            scene['map_frame'] = value('map_frame')
        remappings = []
        if value('cloud_topic'):
            remappings.append(('/camera/depth/points', value('cloud_topic')))
        if value('task_topic'):
            remappings.append(('/area_designation', value('task_topic')))
        nodes.extend([
            Node(package='plan_env', executable='gemini336_depth_scene.py',
                 name='gemini336_depth_scene', output='screen',
                 parameters=node_parameters('gemini336_depth_scene', scene),
                 remappings=remappings),
            Node(package='plan_env', executable='gemini336_mapping_evaluator.py',
                 name='gemini336_mapping_evaluator', output='screen',
                 parameters=node_parameters('gemini336_mapping_evaluator', {
                     'output_dir': output, 'use_sim_time': shared['use_sim_time']})),
            RegisterEventHandler(OnProcessExit(
                target_action=planner_node,
                on_exit=[Shutdown(reason='observation planner completed')])),
        ])
    return nodes


def generate_launch_description():
    share = get_package_share_directory('plan_env')
    arguments = [
        DeclareLaunchArgument('mode', default_value='sim', choices=['sim', 'real']),
        DeclareLaunchArgument('params_file', default_value='',
                              description='Optional mode-specific ROS parameter YAML'),
        DeclareLaunchArgument('output_dir', default_value=''),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_observation_planner', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Enable only when an external /clock exists'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(
            share, 'config', 'gemini336_mapping.rviz')),
    ]
    for name in ('cloud_topic', 'cloud_reliable', 'map_frame', 'motion_frame',
                 'task_topic', 'navigate_action', 'record_bag'):
        arguments.append(DeclareLaunchArgument(
            name, default_value='', description='Override YAML when nonempty'))
    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_nodes)])
