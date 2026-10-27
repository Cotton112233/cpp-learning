from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    plant = Node(
        package="mujoco_bridge",
        executable="mujoco_plant_node",
        output="screen"
    )

    controller = Node(
        package="mujoco_bridge",
        executable="joint_pd_controller",
        parameters=[{
            "hip_kp": ParameterValue(
                LaunchConfiguration("hip_kp"), value_type=float),
            "hip_kd": ParameterValue(
                LaunchConfiguration("hip_kd"), value_type=float),
            "knee_kp": ParameterValue(
                LaunchConfiguration("knee_kp"), value_type=float),
            "knee_kd": ParameterValue(
                LaunchConfiguration("knee_kd"), value_type=float)
        }],
        output="screen"
    )

    hip_publisher = Node(
        package="hip_topic_demo",
        executable="hip_angle_publisher",
        parameters=[{
            "amplitude": ParameterValue(
                LaunchConfiguration("hip_amplitude"), value_type=float),
            "frequency": ParameterValue(
                LaunchConfiguration("hip_frequency"), value_type=float)
        }],
        output="screen"
    )

    knee_publisher = Node(
        package="hip_topic_demo",
        executable="hip_angle_publisher",
        name="knee_angle_publisher",
        remappings=[
            ("/hip/target_angle", "/knee/target_angle")
        ],
        parameters=[{
            "amplitude": ParameterValue(
                LaunchConfiguration("knee_amplitude"), value_type=float),
            "frequency": ParameterValue(
                LaunchConfiguration("knee_frequency"), value_type=float)
        }],
        output="screen"
    )

    return LaunchDescription([
        DeclareLaunchArgument("hip_amplitude", default_value="0.4"),
        DeclareLaunchArgument("hip_frequency", default_value="0.1"),
        DeclareLaunchArgument("knee_amplitude", default_value="0.3"),
        DeclareLaunchArgument("knee_frequency", default_value="0.2"),
        DeclareLaunchArgument("hip_kp", default_value="50.0"),
        DeclareLaunchArgument("hip_kd", default_value="5.0"),
        DeclareLaunchArgument("knee_kp", default_value="50.0"),
        DeclareLaunchArgument("knee_kd", default_value="5.0"),
        plant,
        controller,
        hip_publisher,
        knee_publisher
    ])