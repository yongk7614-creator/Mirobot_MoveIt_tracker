from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="mirobot_moveit_tracker",
                executable="wheel_stop_to_goal_node",
                name="wheel_stop_to_goal_node",
                output="screen",
            ),
            Node(
                package="mirobot_moveit_tracker",
                executable="moveit_goal_node",
                name="moveit_goal_node",
                output="screen",
            ),
        ]
    )
