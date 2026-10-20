"""Task 1 in one command: Gazebo + bridge + static TFs + waypoint follower.

Usage (from host):
  docker compose exec -T sim bash -ic \
      "ros2 launch asv_sim task1.launch.py gui:=false"

Launch arguments:
  gui          (default 'true')  – 'false' for headless Gazebo (passed to
                                   sim.launch.py)
  params_file  (default config/task1_route.yaml) – route, gains, gates

The boat's own values (drag, thrust and steering limits, size) are not in the
params file: they are read from config/asv.yaml here, so asv.yaml stays the
single source of truth (milestone1_plan.md section 6, 10.8 a).
After FINISHED everything keeps running (stop with Ctrl-C); the follower keeps
publishing zero thrust (M1.4 decision L2 a).
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def boat_params(asv):
    """Follower parameters that describe the boat, from the parsed asv.yaml.

    Same formulas as test/boat_model.hpp (load_boat_params). A missing key
    raises KeyError: there is no default, the node must not run on a guess.
    """
    hyd = asv['hydrodynamics']
    thr = asv['thrusters']
    hull = asv['hull']
    return {
        'drag_linear': float(hyd['xU']),         # X_u, N/(m/s)
        'drag_quadratic': float(hyd['xUU']),     # X_uu, N/(m/s)^2
        'max_thrust': float(thr['max_thrust']),  # N
        'min_thrust': float(thr['min_thrust']),  # N (reverse, negative)
        'steer_limit': float(thr['steer_limit']),  # rad
        'boat_length': float(hull['length']),    # L, m
        # b_w: centre line to the outer side of a hull, m
        'boat_half_width': float(hull['spacing']) / 2.0 + float(hull['radius']),
    }


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('asv_sim')
    with open(os.path.join(pkg_share, 'config', 'asv.yaml'), 'r') as f:
        asv = yaml.safe_load(f)
    params_file = LaunchConfiguration('params_file').perform(context)

    return [
        Node(
            package='asv_sim',
            executable='waypoint_follower',
            name='waypoint_follower',
            parameters=[params_file, boat_params(asv)],
            output='screen',
        )
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('asv_sim')
    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Set to "false" for headless (server-only) Gazebo',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'task1_route.yaml'),
            description='Follower parameter file (route, gains, gates)',
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
