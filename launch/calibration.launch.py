import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("multi_lidar_calibrator")
    parameter_file = os.path.join(pkg_share, "config", "params.yaml")
    output = os.path.join(pkg_share, "output")

    # Declare launch arguments
    output_dir_arg = DeclareLaunchArgument(
        "output_dir", default_value=output, description="Path to the output directory."
    )

    params_declare = DeclareLaunchArgument(
        "parameter_file",
        default_value=parameter_file,
        description="Path to the ROS2 parameters file to use.",
    )

    lidar_topics_arg = DeclareLaunchArgument(
        "lidar_topics",
        default_value='["/center_lidar/lidar_points", "/left_lidar/lidar_points"]',
        description="List of LiDAR topics."
    )

    # Use LaunchConfiguration to fetch the values of the arguments
    parameter_file_launch_config = LaunchConfiguration("parameter_file")
    output_dir_launch_config = LaunchConfiguration("output_dir")
    lidar_topics_launch_config = LaunchConfiguration("lidar_topics")

    return LaunchDescription(
        [
            params_declare,
            output_dir_arg,
            lidar_topics_arg,
            Node(
                package="multi_lidar_calibrator",
                executable="multi_lidar_calibrator",
                name="multi_lidar_calibration_node",
                parameters=[
                    parameter_file_launch_config,
                    {
                        'output_dir': output_dir_launch_config,
                        'lidar_topics': lidar_topics_launch_config
                    }
                ],
                output="screen",
            ),
        ]
    )
