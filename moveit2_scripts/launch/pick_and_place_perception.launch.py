import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import TimerAction


def generate_launch_description():

    static_tf_pub_node = Node(
        name="static_tf_publisher",
        package="object_detection",
        executable="static_tf_publisher.py",
        output="screen",    
    )

    # This node will publish the cube's coordinates 
    # so the robot arm can pick and place it
    object_detection_node = Node(
        name="objects_detection",
        package="object_detection",
        executable="objects_detection.py",
        output="screen", 
    )

    moveit_config = MoveItConfigsBuilder("name", package_name="my_moveit_config").to_moveit_configs()

    # MoveItCpp executable
    moveit_cpp_node = Node(
        name="pick_and_place",
        package="moveit2_scripts",
        executable="pick_and_place_perception",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],
        emulate_tty = True,
    )

    # wait for 5 seconds to make sure the detected objects position
    # is being published then launch the subscriber node
    delayed_moveit_node = TimerAction(
        period=5.0,  
        actions=[
        moveit_cpp_node,
        ])

    return LaunchDescription(
        [
            static_tf_pub_node,
            object_detection_node,
            # moveit_cpp_node,
            delayed_moveit_node,
        ]
    )


