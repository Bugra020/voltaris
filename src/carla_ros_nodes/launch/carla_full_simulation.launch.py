import os

from launch_ros import actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression, TextSubstitution)


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

    carla_ros_bridge_nodes_share_dir = get_package_share_directory("carla_ros_nodes")

    # Path to the official carla_ros_bridge launch file
    # This assumes you have the carla_ros_bridge package installed.
    carla_ros_bridge_launch_path = PathJoinSubstitution(
        [get_package_share_directory("carla_ros_bridge"), "carla_ros_bridge.launch.py"]
    )

    manual_controls_script_path = PathJoinSubstitution(
        [
            carla_ros_bridge_nodes_share_dir,
            "carla_ros_nodes/utils",
            "manual_controls.py",
        ]
    )

    carla_render_arg = PythonExpression(
        [
            (
                "'-RenderOffScreen'"
                if LaunchConfiguration("render_offscreen") == "true"
                else "''"
            )
        ]
    )

    carla_server_process = ExecuteProcess(
        cmd=[
            LaunchConfiguration(
                "carla_server_path"
            ),  # The path to the CARLA executable
            carla_render_arg,  # Evaluates to '-RenderOffScreen' or ''
            TextSubstitution(
                text=" > /dev/null 2>&1"
            ),  # This is the shell redirection part
        ],
        output="log",  # Directs CARLA output to the launch log, making it truly headless
        shell=True,  # ESSENTIAL for the shell redirection (> /dev/null 2>&1) to work
        name="carla_simulator",
        on_exit=[TimerAction(period=15.0, actions=[])],  # Wait 15 seconds for CARLA to initialize
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

    manual_controls_script = ExecuteProcess(
        cmd=["python3", manual_controls_script_path],
        output="screen",
        emulate_tty=True,
        name="manual_caral_controls",
    )

    return launch.LaunchDescription(
        [
            carla_server_path,
            render_offscreen,
            carla_server_process,
            TimerAction(
                period=5.0,
                actions=[
                    carla_ros_bridge_launch,
                    lidar_publisher_node,
                    static_tf_publisher_node,
                    dynamic_tf_publisher_node,
                    manual_controls_script,
                ],
            ),
        ]
    )
