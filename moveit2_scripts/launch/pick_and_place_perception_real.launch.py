import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    
    # Launch the object detection nodes first to start publishing the object position
    object_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('object_detection'),
                'launch',
                'object_detection_real.launch.py'
            )
        )
    )

    moveit_config = MoveItConfigsBuilder("name", package_name="real_moveit_config").to_moveit_configs()

    # MoveItCpp executable
    moveit_cpp_node = Node(
        name="pick_and_place_real",
        package="moveit2_scripts",
        executable="pick_and_place_perception_real",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': False},
        ],
        emulate_tty = True,
    )

    # wait for 10 seconds to make sure the detected objects position
    # is being published then launch the subscriber node and the moveit node
    delayed_moveit_node = TimerAction(
        period=10.0,  
        actions=[
        moveit_cpp_node,
        ])

    return LaunchDescription(
        [
            object_detection_launch,
            delayed_moveit_node,
        ]
    )


