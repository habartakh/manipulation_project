import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_description = "object_detection"
    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'object_detection_config.rviz')

    
    static_tf_pub_node = Node(
        name="static_tf_publisher",
        package="object_detection",
        executable="static_tf_publisher.py",
        output="screen",    
    )

    object_detection_node = Node(
        name="objects_detection",
        package="object_detection",
        executable="objects_detection.py",
        output="screen",    
    )

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])

    return LaunchDescription(
        [
            rviz_node,
            static_tf_pub_node,
            object_detection_node,
            
        ]
    )