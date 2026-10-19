from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    publisher = Node(
        package="hip_topic_demo",
        executable="hip_angle_publisher",
        parameters=[
            {"amplitude": 0.8, "frequency": 0.5}
        ],
        output="screen"
    )

    subscriber = Node(
        package = "hip_topic_demo",
        executable = "hip_angle_subscriber",
        output = "screen"
    )

    return LaunchDescription([
        publisher,
        subscriber
    ])