from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("rs03_current_torque_control"),
        "config", "rs03_current_torque.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("auto_enable", default_value="false",
                              choices=["true", "false"]),
        DeclareLaunchArgument("control_mode", default_value="current",
                              choices=["current", "torque"]),
        DeclareLaunchArgument("max_current_a", default_value="0.5"),
        DeclareLaunchArgument("max_torque_nm", default_value="1.0"),
        DeclareLaunchArgument("max_velocity_rad_s", default_value="2.0"),
        Node(
            package="rs03_current_torque_control",
            executable="rs03_current_torque_node",
            name="rs03_current_torque",
            parameters=[
                config,
                {
                    "auto_enable": ParameterValue(
                        LaunchConfiguration("auto_enable"), value_type=bool),
                    "control_mode": LaunchConfiguration("control_mode"),
                    "max_current_a": ParameterValue(
                        LaunchConfiguration("max_current_a"), value_type=float),
                    "max_torque_nm": ParameterValue(
                        LaunchConfiguration("max_torque_nm"), value_type=float),
                    "max_velocity_rad_s": ParameterValue(
                        LaunchConfiguration("max_velocity_rad_s"), value_type=float),
                },
            ],
            output="screen",
        )
    ])
