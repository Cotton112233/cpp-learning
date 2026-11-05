from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    stamp = datetime.now(ZoneInfo("Asia/Tokyo")).strftime("%Y%m%d_%H%M%S_%f")
    default_csv = str(Path.home() / "ros2_experiment_logs" / "07_4" / (stamp + ".csv"))
    defaults = {
        "algorithm": "quintic",
        "start_delay": "1.0",
        "hold_time": "2.0",
        "output_file": default_csv,
        "hip_q0": "0.0", "hip_qf": "0.4", "hip_T": "4.0",
        "hip_vmax": "0.5", "hip_amax": "1.0",
        "knee_q0": "0.0", "knee_qf": "-0.3", "knee_T": "4.0",
        "knee_vmax": "0.5", "knee_amax": "1.0",
        "hip_kp": "50.0", "hip_kd": "5.0",
        "knee_kp": "50.0", "knee_kd": "5.0",
    }

    def number(name):
        return ParameterValue(LaunchConfiguration(name), value_type=float)

    trajectory_parameters = {
        "algorithm": ParameterValue(LaunchConfiguration("algorithm"), value_type=str),
    }
    for joint in ("hip", "knee"):
        for field in ("q0", "qf", "T", "vmax", "amax"):
            key = joint + "_" + field
            trajectory_parameters[key] = number(key)

    # Resolve start_delay at execution time and give both nodes the same origin.
    from launch.actions import OpaqueFunction

    def start_system(context):
        delay = float(LaunchConfiguration("start_delay").perform(context))
        epoch = time.time() + max(0.0, delay)
        parameters = dict(trajectory_parameters)
        parameters["start_epoch"] = epoch
        return [
            Node(package="mujoco_bridge", executable="mujoco_plant_node", output="screen"),
            Node(package="mujoco_bridge", executable="joint_pd_controller",
                 parameters=[{key: number(key) for key in
                              ("hip_kp", "hip_kd", "knee_kp", "knee_kd")}],
                 output="screen"),
            Node(package="trajectory_planning", executable="trajectory_recorder",
                 parameters=[parameters, {
                     "hold_time": number("hold_time"),
                     "output_file": ParameterValue(LaunchConfiguration("output_file"), value_type=str),
                 }], output="screen"),
            Node(package="trajectory_planning", executable="trajectory_node",
                 parameters=[parameters], output="screen"),
        ]

    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]
        + [OpaqueFunction(function=start_system)]
    )
