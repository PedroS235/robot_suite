import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.conditions import IfCondition


def generate_launch_description():
    pkg_dir = get_package_share_directory("hand_gestures")

    default_param_file = os.path.join(pkg_dir, "config", "params.yaml")
    params_file_arg = DeclareLaunchArgument(
        "params_file", default_value=str(default_param_file)
    )

    annotator_flag = DeclareLaunchArgument(
        "run_annotator",
        default_value="false",
        description="If true, the annotator node will run, drawing the landmarks into an image",
    )

    param_file = LaunchConfiguration("params_file")

    detector_node = Node(
        package="hand_gestures",
        executable="landmark_detector_node",
        parameters=[param_file],
        output="screen",
    )

    annotator_node = Node(
        package="hand_gestures",
        executable="landmark_annotator_node",
        parameters=[param_file],
        output="screen",
        condition=IfCondition(LaunchConfiguration("run_annotator")),
    )

    ld = LaunchDescription()
    ld.add_action(annotator_flag)
    ld.add_action(params_file_arg)
    ld.add_action(detector_node)
    ld.add_action(annotator_node)

    return ld
