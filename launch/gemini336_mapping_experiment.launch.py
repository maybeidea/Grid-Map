from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_run_id = datetime.now().strftime('%Y%m%d_%H%M%S')
    output_dir = PathJoinSubstitution([
        LaunchConfiguration('output_root'), LaunchConfiguration('run_id')])
    planner_node = Node(
        package='plan_env',
        executable='observation_planner_node',
        name='plan_env_observation_planner',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_observation_planner')),
        parameters=[{
            'grid_map.frame_id': 'map',
            'grid_map.motion_frame': 'gemini336_depth_optical_frame',
            'grid_map.cloud_topic': '/camera/depth/points',
            'grid_map.map_size_z': 1.8,
            'grid_map.high_resolution': LaunchConfiguration('high_resolution'),
            'grid_map.low_resolution': LaunchConfiguration('low_resolution'),
            'grid_map.expansion_margin': 0.8,
            'grid_map.cloud_leaf_size': LaunchConfiguration('cloud_leaf_size'),
            'grid_map.cloud_low_z_clear_rays': True,
            'grid_map.stationary_frames_min': 1,
            'grid_map.stationary_frames': 2,
            'grid_map.motion_translation_threshold': 0.03,
            'grid_map.motion_rotation_threshold_rad': 0.02,
            'grid_map.registration.enabled': False,
            'grid_map.esdf_recompute_change_voxels': 128,
            'observation.max_stations_per_round': 5,
            'observation.max_rounds': 3,
            'observation.finish_on_complete': True,
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('plan_env'), 'config', 'gemini336_mapping.rviz'])),
        DeclareLaunchArgument('publish_rate', default_value='5.0'),
        DeclareLaunchArgument('pixel_stride', default_value='2'),
        DeclareLaunchArgument('scenario', default_value='front_opening',
                              description='Initial view: front_opening or side_obstacle'),
        DeclareLaunchArgument('use_observation_planner', default_value='true'),
        DeclareLaunchArgument('frames_per_observation', default_value='10'),
        DeclareLaunchArgument('wait_for_fusion', default_value='true'),
        DeclareLaunchArgument('high_resolution', default_value='0.02'),
        DeclareLaunchArgument('low_resolution', default_value='0.04'),
        DeclareLaunchArgument('cloud_leaf_size', default_value='0.02'),
        DeclareLaunchArgument('output_root', default_value='/tmp/plan_env_gemini336'),
        DeclareLaunchArgument('run_id', default_value=default_run_id),
        Node(
            package='plan_env',
            executable='gemini336_depth_scene.py',
            name='gemini336_depth_scene',
            output='screen',
            parameters=[{
                'publish_rate': LaunchConfiguration('publish_rate'),
                'pixel_stride': LaunchConfiguration('pixel_stride'),
                'scenario': LaunchConfiguration('scenario'),
                'observation_planner_enabled': LaunchConfiguration('use_observation_planner'),
                'frames_per_observation': LaunchConfiguration('frames_per_observation'),
                'wait_for_fusion': LaunchConfiguration('wait_for_fusion'),
                'camera.width': 640,
                'camera.height': 480,
                'camera.fx': 387.229248,
                'camera.fy': 387.229248,
                'camera.cx': 321.046387,
                'camera.cy': 243.449692,
                'camera.frame_id': 'gemini336_depth_optical_frame',
                'map_frame': 'map',
                'depth.min_range': 0.2,
                'depth.max_range': 5.0,
            }],
        ),
        Node(
            package='plan_env',
            executable='grid_map_standalone_test',
            name='grid_map_standalone_test',
            output='screen',
            parameters=[{
                'task_topic': '/area_designation',
                'heightmap_topic': '/grid_map/heightmap',
                'esdf_topic': '/grid_map/esdf_slice',
                'occupancy_3d_topic': '/grid_map/occupied_voxels',
                'visualization_dir': output_dir,
                'record_bag': False,
                'grid_map.frame_id': 'map',
                'grid_map.motion_frame': 'gemini336_depth_optical_frame',
                'grid_map.cloud_topic': '/camera/depth/points',
                'grid_map.map_size_z': 1.8,
                'grid_map.high_resolution': LaunchConfiguration('high_resolution'),
                'grid_map.low_resolution': LaunchConfiguration('low_resolution'),
                'grid_map.expansion_margin': 0.8,
                'grid_map.cloud_leaf_size': LaunchConfiguration('cloud_leaf_size'),
                # Keep floor returns as clearing rays while excluding the
                # ground surface itself from obstacle height classification.
                'grid_map.cloud_low_z_clear_rays': True,
                'grid_map.stationary_frames_min': 1,
                'grid_map.stationary_frames': 2,
                'grid_map.motion_translation_threshold': 0.03,
                'grid_map.motion_rotation_threshold_rad': 0.02,
                'grid_map.registration.enabled': False,
                'grid_map.esdf_recompute_change_voxels': 128,
                'grid_map.height_band_z_max': 2.0,
            }],
        ),
        planner_node,
        Node(
            package='plan_env',
            executable='gemini336_mapping_evaluator.py',
            name='gemini336_mapping_evaluator',
            output='screen',
            parameters=[{'output_dir': output_dir}],
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
