"""Launch Gazebo Harmonic sim with ros_gz_bridge, static TF publishers and
the corruptor (realistic sensor data under /asv/sensors/..., M2.6d).

Usage (from host):
  docker compose exec -T sim bash -ic \
      "ros2 launch asv_sim sim.launch.py gui:=false"

Launch arguments:
  gui            (default 'true')     – 'false' for headless (server-only) mode
  corruptor      (default 'true')     – 'false' to leave the corruptor out
  noise_profile  (default 'nominal')  – config/noise_profiles/<name>.yaml
task1.launch.py and maneuver.launch.py include this file, so the same
arguments work there (e.g. noise_profile:=mode_check).
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def _launch_setup(context, *args, **kwargs):
    """Build the list of actions at launch time so we can read YAML files and
    inspect the 'gui' argument as a plain Python string."""

    pkg_share = get_package_share_directory('asv_sim')
    gui_str = LaunchConfiguration('gui').perform(context)
    headless = gui_str.lower() in ('false', '0', 'no')

    # Every ROS node below runs on Gazebo's clock (/clock), not the computer's
    # clock (docs/design.md §3). SetParameter applies to all nodes launched
    # after it in this description. Gazebo itself is not a ROS node.
    actions = [SetParameter(name='use_sim_time', value=True)]

    # ── 1. Gazebo Harmonic ──────────────────────────────────────────────
    world_file = os.path.join(pkg_share, 'worlds', 'roboboat_course.sdf')

    # GZ_SIM_RESOURCE_PATH lets Gazebo find our model SDFs.
    models_dir = os.path.join(pkg_share, 'models')
    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if models_dir not in gz_resource_path:
        os.environ['GZ_SIM_RESOURCE_PATH'] = (
            models_dir + (':' + gz_resource_path if gz_resource_path else '')
        )

    gz_args = f'-v 4 -r {world_file}'
    if headless:
        gz_args = f'-v 4 -r -s {world_file}'

    gz_sim_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
    )
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch),
            launch_arguments={'gz_args': gz_args}.items(),
        )
    )

    # ── 2. ros_gz_bridge ────────────────────────────────────────────────
    bridge_config = os.path.join(pkg_share, 'config', 'asv_bridge.yaml')
    actions.append(
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='asv_bridge',
            parameters=[{'config_file': bridge_config}],
            output='screen',
        )
    )

    # ── 3. Static TF publishers (one per frame in asv_frames.yaml) ─────
    frames_file = os.path.join(pkg_share, 'config', 'asv_frames.yaml')
    with open(frames_file, 'r') as f:
        frames_data = yaml.safe_load(f)

    for i, frame in enumerate(frames_data['frames']):
        xyz = frame['xyz']  # [x, y, z]
        rpy = frame['rpy']  # [roll, pitch, yaw]
        actions.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'static_tf_{frame["child"]}',
                arguments=[
                    '--x', str(xyz[0]),
                    '--y', str(xyz[1]),
                    '--z', str(xyz[2]),
                    '--roll', str(rpy[0]),
                    '--pitch', str(rpy[1]),
                    '--yaw', str(rpy[2]),
                    '--frame-id', frame['parent'],
                    '--child-frame-id', frame['child'],
                ],
                output='screen',
            )
        )

    # ── 4. Corruptor: clean sensor topics -> /asv/sensors/... (D1 a, D3 b) ──
    # On by default (M2.6d Q1 a), so every run has the realistic topics the
    # filter reads. It reads asv.yaml and the profile itself (corruptor_node.cpp).
    if LaunchConfiguration('corruptor').perform(context).lower() not in ('false', '0', 'no'):
        actions.append(
            Node(
                package='asv_sim',
                executable='corruptor',
                name='corruptor',
                parameters=[{'noise_profile': LaunchConfiguration('noise_profile')}],
                output='screen',
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Set to "false" for headless (server-only) Gazebo',
        ),
        DeclareLaunchArgument(
            'corruptor',
            default_value='true',
            description='Set to "false" to run without the corruptor (no /asv/sensors/...)',
        ),
        DeclareLaunchArgument(
            'noise_profile',
            default_value='nominal',
            description='Corruptor noise profile: config/noise_profiles/<name>.yaml',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
