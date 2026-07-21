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
                              choices=["current", "torque", "velocity", "position_pp"]),
        DeclareLaunchArgument("command_timeout_s", default_value="0.30"),
        DeclareLaunchArgument("max_current_a", default_value="0.5"),
        DeclareLaunchArgument("max_torque_nm", default_value="1.0"),
        DeclareLaunchArgument("torque_demo_duration_s", default_value="0.0"),
        DeclareLaunchArgument("torque_soft_velocity_start_rad_s", default_value="0.0"),
        DeclareLaunchArgument("torque_soft_velocity_limit_rad_s", default_value="0.0"),
        DeclareLaunchArgument("torque_soft_brake_gain_nm_per_rad_s", default_value="0.0"),
        DeclareLaunchArgument("torque_soft_brake_max_nm", default_value="0.0"),
        DeclareLaunchArgument("torque_speed_feedforward_nm", default_value="0.0"),
        DeclareLaunchArgument("torque_velocity_filter_alpha", default_value="0.2"),
        DeclareLaunchArgument("max_velocity_command_rad_s", default_value="0.5"),
        DeclareLaunchArgument("position_max_offset_rad", default_value="0.2"),
        DeclareLaunchArgument("position_current_limit_a", default_value="0.5"),
        DeclareLaunchArgument("position_speed_limit_rad_s", default_value="0.2"),
        DeclareLaunchArgument("position_acceleration_rad_s2", default_value="0.5"),
        DeclareLaunchArgument("position_slew_rate_rad_s", default_value="0.05"),
        DeclareLaunchArgument("position_ramp_max_error_rad", default_value="0.03"),
        DeclareLaunchArgument("position_tracking_error_rad", default_value="0.5"),
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
                    "command_timeout_s": ParameterValue(
                        LaunchConfiguration("command_timeout_s"), value_type=float),
                    "max_current_a": ParameterValue(
                        LaunchConfiguration("max_current_a"), value_type=float),
                    "max_torque_nm": ParameterValue(
                        LaunchConfiguration("max_torque_nm"), value_type=float),
                    "torque_demo_duration_s": ParameterValue(
                        LaunchConfiguration("torque_demo_duration_s"), value_type=float),
                    "torque_soft_velocity_start_rad_s": ParameterValue(
                        LaunchConfiguration("torque_soft_velocity_start_rad_s"), value_type=float),
                    "torque_soft_velocity_limit_rad_s": ParameterValue(
                        LaunchConfiguration("torque_soft_velocity_limit_rad_s"), value_type=float),
                    "torque_soft_brake_gain_nm_per_rad_s": ParameterValue(
                        LaunchConfiguration("torque_soft_brake_gain_nm_per_rad_s"), value_type=float),
                    "torque_soft_brake_max_nm": ParameterValue(
                        LaunchConfiguration("torque_soft_brake_max_nm"), value_type=float),
                    "torque_speed_feedforward_nm": ParameterValue(
                        LaunchConfiguration("torque_speed_feedforward_nm"), value_type=float),
                    "torque_velocity_filter_alpha": ParameterValue(
                        LaunchConfiguration("torque_velocity_filter_alpha"), value_type=float),
                    "max_velocity_command_rad_s": ParameterValue(
                        LaunchConfiguration("max_velocity_command_rad_s"), value_type=float),
                    "position_max_offset_rad": ParameterValue(
                        LaunchConfiguration("position_max_offset_rad"), value_type=float),
                    "position_current_limit_a": ParameterValue(
                        LaunchConfiguration("position_current_limit_a"), value_type=float),
                    "position_speed_limit_rad_s": ParameterValue(
                        LaunchConfiguration("position_speed_limit_rad_s"), value_type=float),
                    "position_acceleration_rad_s2": ParameterValue(
                        LaunchConfiguration("position_acceleration_rad_s2"), value_type=float),
                    "position_slew_rate_rad_s": ParameterValue(
                        LaunchConfiguration("position_slew_rate_rad_s"), value_type=float),
                    "position_ramp_max_error_rad": ParameterValue(
                        LaunchConfiguration("position_ramp_max_error_rad"), value_type=float),
                    "position_tracking_error_rad": ParameterValue(
                        LaunchConfiguration("position_tracking_error_rad"), value_type=float),
                    "max_velocity_rad_s": ParameterValue(
                        LaunchConfiguration("max_velocity_rad_s"), value_type=float),
                },
            ],
            output="screen",
        )
    ])
