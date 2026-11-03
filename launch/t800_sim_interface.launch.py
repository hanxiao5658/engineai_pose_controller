#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Start the four EngineAI ROS 2 simulation interface nodes together.

Run:
    ros2 launch engineai_pose_controller t800_sim_interface.launch.py

Ctrl+C in the ros2 launch terminal requests shutdown of all four nodes.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package = 'engineai_pose_controller'

    return LaunchDescription([
        # Standard ROS2 Twist -> EngineAI custom BodyVelCmd
        Node(
            package=package,
            executable='body_vel_twist_to_vendor',
            name='body_vel_twist_to_vendor',
            output='screen',
            emulate_tty=True,
            respawn=False,
        ),

        # EngineAI BodyVelCmd -> LCM virtual gamepad (MuJoCo)
        Node(
            package=package,
            executable='global_pose_gamepad_controller',
            name='global_pose_gamepad_controller',
            output='screen',
            emulate_tty=True,
            respawn=False,
        ),

        # MuJoCo base pose -> geometry_msgs/PoseStamped
        Node(
            package=package,
            executable='engineai_base_pose_to_standard',
            name='engineai_base_pose_to_standard',
            output='screen',
            emulate_tty=True,
            respawn=False,
        ),

        # MuJoCo joint states -> sensor_msgs/JointState
        Node(
            package=package,
            executable='engineai_joint_state_to_standard',
            name='engineai_joint_state_to_standard',
            output='screen',
            emulate_tty=True,
            respawn=False,
        ),
    ])
