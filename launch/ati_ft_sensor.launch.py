import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory("ati_ft_sensor"), "config", "ft_publisher.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="ati_ft_sensor",
                name="ati_wrench_publisher",
                executable = "ati_ft_sensor_wrench_pub",
                output="screen",
                parameters=[config],
            )
        ]
    )
