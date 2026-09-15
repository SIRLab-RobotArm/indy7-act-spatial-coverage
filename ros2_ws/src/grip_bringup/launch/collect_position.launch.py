#!/usr/bin/env python3
"""Start one RGB-only camera and a recorder fixed to one physical position."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_config = PathJoinSubstitution([
        FindPackageShare('grip_bringup'), 'config', 'camera_rgb_only.yaml'])

    camera = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='overhead_camera',
        name='camera',
        output='screen',
        parameters=[camera_config, {
            'serial_no': LaunchConfiguration('camera_serial'),
        }])

    recorder = Node(
        package='grip_collect',
        executable='episode_recorder_node',
        name='episode_recorder',
        output='screen',
        parameters=[{
            'output_dir': LaunchConfiguration('output_dir'),
            'data_block': LaunchConfiguration('data_block'),
            'position_id': LaunchConfiguration('position_id'),
            'accept_context_updates': False,
            'session_id': LaunchConfiguration('session_id'),
            'operator_id': LaunchConfiguration('operator_id'),
            'camera_serial': LaunchConfiguration('camera_serial'),
            'exposure_us': 5000.0,
            'gain': 16.0,
            'white_balance': 4600.0,
            'auto_exposure_locked': True,
            'home_joint_deg': [
                0.005221, 40.003174, -129.996640,
                90.000870, 0.000435, 0.001577,
            ],
            'ref_frame': 'indy_base',
            'image_height': 480,
            'image_width': 640,
        }])

    rgb_preview = Node(
        package='image_tools',
        executable='showimage',
        name='collection_rgb_preview',
        output='screen',
        condition=IfCondition(LaunchConfiguration('show_camera')),
        remappings=[
            ('image', '/overhead_camera/camera/color/image_raw'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('position_id'),
        DeclareLaunchArgument('output_dir'),
        DeclareLaunchArgument('data_block', default_value='main'),
        DeclareLaunchArgument('session_id', default_value='main_session'),
        DeclareLaunchArgument('operator_id', default_value='operator'),
        DeclareLaunchArgument(
            'camera_serial',
            default_value=EnvironmentVariable(
                'GRIP_CAMERA_SERIAL', default_value='_000000000000')),
        DeclareLaunchArgument('show_camera', default_value='true'),
        SetEnvironmentVariable(
            'PYTHONPATH', [
                os.environ.get('GRIP_ROS_PYDEPS', '') + os.pathsep,
                EnvironmentVariable('PYTHONPATH', default_value='')]),
        camera,
        recorder,
        rgb_preview,
    ])
