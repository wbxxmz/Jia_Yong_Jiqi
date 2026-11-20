#!/usr/bin/env python3
"""启动 robot_state_publisher 与 RViz，供题一驱动发布 /joint_states 显示。"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.dirname(HERE)
sys.path.insert(0, SRC)

from robot_urdf import expand_urdf  # noqa: E402


def generate_launch_description():
    urdf_root = os.environ.get("BAJIE_URDF_ROOT", "/home/yjh/bajie-competition/urdf")
    rviz_config = LaunchConfiguration("rviz_config")

    description = expand_urdf(urdf_root)

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": ParameterValue(description, value_type=str)}],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(SRC, "config", "q1.rviz"),
            ),
            rsp,
            rviz,
        ]
    )
