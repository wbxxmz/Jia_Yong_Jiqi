"""启动 robot_state_publisher 与 RViz，显示八界机器人。"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import sys

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from robot_urdf import expand_urdf  # noqa: E402


def launch_setup(context):
    urdf_root = LaunchConfiguration("urdf_root").perform(context)
    rviz_config = LaunchConfiguration("rviz_config").perform(context)

    description = expand_urdf(urdf_root)

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": ParameterValue(description, value_type=str)}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ]
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "urdf_root",
                default_value="/home/yjh/bajie-competition/urdf",
                description="官方 URDF 仓库根目录",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value="/home/yjh/bajie-competition/competition/bajie-entry/src/question1_project/config/q1.rviz",
                description="RViz 配置文件（官方配置副本，已关闭 TF 坐标轴显示）",
            ),
            DeclareLaunchArgument("rviz", default_value="true", description="是否启动 RViz"),
            OpaqueFunction(function=launch_setup),
        ]
    )
