import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    static_tf_pub_node = Node(
        name="static_tf_publisher",
        package="object_detection",
        executable="static_tf_publisher.py",
        output="screen",    
    )

    return LaunchDescription(
        [
            static_tf_pub_node,
        ]
    )