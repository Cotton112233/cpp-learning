import csv
import io
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # x,z are hip-relative coordinates; the model hip is 1.5m above world origin.
    defaults = {
        "foot_x": "0.2", "foot_z": "-0.8",
        "algorithm": "quintic", "duration": "4.0",
        "hip_vmax": "0.5", "hip_amax": "1.0",
        "knee_vmax": "0.5", "knee_amax": "1.0",
        "hip_kp": "50.0", "hip_kd": "5.0",
        "knee_kp": "50.0", "knee_kd": "5.0",
        "start_delay": "1.0", "hold_time": "2.0",
    }

    def start_system(context):
        values = {name: LaunchConfiguration(name).perform(context) for name in defaults}
        executable = Path(get_package_prefix("trajectory_planning")) / "lib/trajectory_planning/ik_cli"
        result = subprocess.run(
            [str(executable), values["foot_x"], values["foot_z"]],
            capture_output=True, text=True
        )
        if result.returncode != 0:
            raise RuntimeError(result.stderr.strip())

        angles = next(csv.DictReader(io.StringIO(result.stdout)))
        print("IK: hip=" + angles["hip"] + " rad, knee=" + angles["knee"] + " rad")
        arguments = {
            "algorithm": values["algorithm"],
            "hip_q0": "0.0", "knee_q0": "0.0",
            "hip_qf": angles["hip"], "knee_qf": angles["knee"],
            "hip_T": values["duration"], "knee_T": values["duration"],
        }
        for name in ("hip_vmax", "hip_amax", "knee_vmax", "knee_amax",
                     "hip_kp", "hip_kd", "knee_kp", "knee_kd", "start_delay", "hold_time"):
            arguments[name] = values[name]
        existing_launch = Path(get_package_share_directory("trajectory_planning")) / "launch/trajectory_system.launch.py"
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(existing_launch)),
            launch_arguments=arguments.items()
        )]

    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]
        + [OpaqueFunction(function=start_system)]
    )
