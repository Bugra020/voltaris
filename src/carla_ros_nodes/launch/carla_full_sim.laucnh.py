import os

import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    carla_server_path = DeclareLaunchArgument(
        "carla_server_path",
        default_value="~/carla/Dist/CARLA_Shipping_0.9.15-262-g8f7e40f3c-dirty/LinuxNoEditor/CarlaUE4.sh",
        description="Path to the CARLAUE4.sh executable.",
    )

    render_offscreen = DeclareLaunchArgument(
        "render_offscreen",
        default_value="true",
        description="Run CARLA with rendering offscreen.",
    )

    carla_ros_bridge_nodes_share_dir = get_package_share_directory(
        "carla_ros_bridge_nodes"
    )

    # Path to the official carla_ros_bridge launch file
    # This assumes you have the carla_ros_bridge package installed.
    carla_ros_bridge_launch_path = PathJoinSubstitution(
        [get_package_share_directory("carla_ros_bridge"), "carla_ros_bridge.launch.py"]
    )

    # 1. Start CARLA Simulator (as a separate process)
    # Using ExecuteProcess to run shell commands.
    # The `output='log'` helps in debugging if CARLA doesn't start.
    # `emulate_tty=True` can help with process control.
    carla_server_process = ExecuteProcess(
        cmd=[
            LaunchConfiguration("carla_server_path"),
            (
                "-RenderOffScreen"
                if LaunchConfiguration("render_offscreen") == "true"
                else ""
            ),  # Conditional argument
        ],
        output="screen",
        emulate_tty=True,
        shell=True,
        name="carla_simulator",
        on_exit=[launch.actions.TimerAction(period=15.0)],

    )

    carla_ros_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(carla_ros_bridge_launch_path),
    )

    lidar_publisher_node = launch_ros.actions.Node(
        package="carla_ros_nodes",
        executable="lidar_publisher",
        name="carla_lidar_publisher",
        output="screen",
    )

    static_tf_publisher_node = launch_ros.actions.Node(
        package="carla_ros_nodes",
        executable="static_tf_publisher",
        name="carla_static_tf_publisher",
        output="screen",
    )

    dynamic_tf_publisher_node = launch_ros.actions.Node(
        package="carla_ros_nodes",
        executable="dynamic_tf_publisher",
        name="carla_dynamic_tf_publisher",
        output="screen",
    )

    return launch.LaunchDescription(
        [
            carla_server_path,
            render_offscreen,
            carla_server_process,
            launch.actions.TimerAction(
                period=5.0,
                actions=[
                    carla_ros_bridge_launch,
                    lidar_publisher_node,
                    static_tf_publisher_node,
                    dynamic_tf_publisher_node,
                ],
            ),
        ]
    )
