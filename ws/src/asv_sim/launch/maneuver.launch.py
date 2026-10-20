"""Open-loop maneuver in one command: Gazebo + bridge + static TFs + player.

Usage (from host):
  docker compose exec -T sim bash -ic \
      "ros2 launch asv_sim maneuver.launch.py maneuver:=circle gui:=false"

Launch arguments:
  maneuver  (default 'straight') - name of a file in config/maneuvers/
                                   (straight, circle, zigzag)
  gui       (default 'true')     - 'false' for headless Gazebo

The thruster limits come from config/asv.yaml through boat_params() of
task1.launch.py (one place for the asv.yaml -> parameter mapping).
After DONE everything keeps running (stop with Ctrl-C); the player keeps
publishing zero thrust (M1.7a decision P3). Never start it together with
task1.launch.py: both drive the thrusters.
"""

import importlib.util
import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

LIMIT_KEYS = ('max_thrust', 'min_thrust', 'steer_limit')


def _task1_launch():
    """task1.launch.py as a module (launch files are not an importable package)."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'task1.launch.py')
    spec = importlib.util.spec_from_file_location('task1_launch', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def limit_params(asv):
    """The player's thruster limits from the parsed asv.yaml."""
    boat = _task1_launch().boat_params(asv)
    return {k: boat[k] for k in LIMIT_KEYS}


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('asv_sim')
    with open(os.path.join(pkg_share, 'config', 'asv.yaml'), 'r') as f:
        asv = yaml.safe_load(f)
    name = LaunchConfiguration('maneuver').perform(context)
    maneuver_file = os.path.join(pkg_share, 'config', 'maneuvers', name + '.yaml')
    if not os.path.isfile(maneuver_file):
        raise FileNotFoundError(f'no maneuver file {maneuver_file}')

    return [
        Node(
            package='asv_sim',
            executable='maneuver_player',
            name='maneuver_player',
            parameters=[maneuver_file, limit_params(asv)],
            output='screen',
        )
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('asv_sim')
    return LaunchDescription([
        DeclareLaunchArgument(
            'maneuver',
            default_value='straight',
            description='Maneuver file name in config/maneuvers/ (without .yaml)',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Set to "false" for headless (server-only) Gazebo',
        ),
        # Every ROS node runs on Gazebo's clock (docs/design.md §3).
        SetParameter(name='use_sim_time', value=True),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'sim.launch.py')),
            launch_arguments={'gui': LaunchConfiguration('gui')}.items(),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
