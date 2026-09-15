#!/usr/bin/env python3
"""One-overhead-camera collection graph with an explicit mock path."""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, GroupAction,
    IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mock = LaunchConfiguration('mock')
    camera_config = PathJoinSubstitution([
        FindPackageShare('grip_bringup'), 'config', 'camera_rgb_only.yaml'])

    indy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('indy_driver'), 'launch', 'indy_bringup.launch.py'])),
        launch_arguments={
            'indy_ip': LaunchConfiguration('indy_ip'),
            'indy_type': 'indy7',
            'launch_rviz': 'false',
            'mock': mock,
        }.items())

    mark7 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('pipet_hand_mark7_driver'),
            'launch', 'mark7_hardware.launch.py'])),
        condition=UnlessCondition(mock),
        launch_arguments={
            'port': LaunchConfiguration('mark7_port'),
            'use_mock_hardware': 'false',
            'use_rviz': 'false',
        }.items())

    gripper_services = Node(
        package='pipet_hand_mark7_teleop',
        executable='grip_preset_node',
        condition=UnlessCondition(mock),
        parameters=[PathJoinSubstitution([
            FindPackageShare('pipet_hand_mark7_driver'),
            'config', 'grip_presets.yaml'])])

    camera = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='overhead_camera',
        name='camera',
        output='screen',
        condition=UnlessCondition(mock),
        parameters=[camera_config, {
            'serial_no': LaunchConfiguration('camera_serial'),
        }])

    recorder = Node(
        package='grip_collect',
        executable='episode_recorder_node',
        name='episode_recorder',
        output='screen',
        parameters=[{
            'mock': ParameterValue(mock, value_type=bool),
            'output_dir': LaunchConfiguration('output_dir'),
            'data_block': LaunchConfiguration('data_block'),
            'sync_slop': 0.03,
            'camera_serial': LaunchConfiguration('camera_serial'),
            'exposure_us': 5000.0,
            'gain': 16.0,
            'white_balance': 4600.0,
            'auto_exposure_locked': True,
        }])

    rosbag = GroupAction(
        condition=UnlessCondition(mock),
        actions=[ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('record_bag')),
            cmd=['ros2', 'bag', 'record', '-a'],
            output='screen',
        )])

    return LaunchDescription([
        DeclareLaunchArgument('mock', default_value='false'),
        DeclareLaunchArgument(
            'indy_ip',
            default_value=EnvironmentVariable(
                'GRIP_INDY_IP', default_value='192.168.0.10')),
        DeclareLaunchArgument(
            'mark7_port',
            default_value=EnvironmentVariable(
                'GRIP_MARK7_PORT', default_value='/dev/ttyACM0')),
        DeclareLaunchArgument(
            'camera_serial',
            default_value=EnvironmentVariable(
                'GRIP_CAMERA_SERIAL', default_value='_000000000000')),
        DeclareLaunchArgument('output_dir', default_value='episodes/development_episodes'),
        DeclareLaunchArgument('data_block', default_value='dev'),
        DeclareLaunchArgument('record_bag', default_value='true'),
        SetEnvironmentVariable(
            'PYTHONPATH', [
                os.environ.get('GRIP_ROS_PYDEPS', '') + os.pathsep,
                EnvironmentVariable('PYTHONPATH', default_value='')]),
        indy,
        mark7,
        gripper_services,
        camera,
        recorder,
        rosbag,
    ])
