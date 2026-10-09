"""Refuse to launch Autoware without a writable data_path.

TensorRT nodes build their engine from the ONNX on first use and write it next to that
file. A missing or read-only data_path (the packaged /opt/autoware/<ver>/data is
root-owned) makes that write fail after a successful build; the component constructor
throws, the composable-node loader swallows it, and the node never appears while the rest
of the pipeline comes up and publishes empty results. Failing here, before anything
starts, names the fix instead.

Included first by carla_simulator.launch.xml. Silent when the directory is fine.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

FIX = "ros2 run acb_launch setup_autoware_data"


def problem(path):
    """Why `path` cannot serve as data_path, or None when it can."""
    if not os.path.isdir(path):
        return f"data_path {path} does not exist"
    if not os.access(path, os.W_OK | os.X_OK):
        return f"data_path {path} is not writable"
    return None


def _check(context):
    path = os.path.expanduser(LaunchConfiguration("data_path").perform(context))
    why = problem(path)
    if why:
        raise RuntimeError(
            f"{why}. TensorRT nodes write their engines there and silently never load "
            f"without it. Populate it with:  {FIX}  (or pass data_path:=<writable dir>)"
        )
    return []


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("data_path", description="directory to check"),
            OpaqueFunction(function=_check),
        ]
    )
