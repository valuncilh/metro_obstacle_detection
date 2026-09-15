from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="metro_obstacle_detection",
            executable="preprocessor",
            name="preprocessor",
            output="screen",
        ),
        Node(
            package="metro_obstacle_detection",
            executable="gauge_filter",
            name="gauge_filter",
            output="screen",
        ),
        Node(
            package="metro_obstacle_detection",
            executable="anomaly_detector",
            name="anomaly_detector",
            output="screen",
        ),
        Node(
            package="metro_obstacle_detection",
            executable="decision_maker",
            name="decision_maker",
            output="screen",
        ),
    ])
