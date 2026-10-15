"""Launch Gazebo Harmonic sim with ros_gz_bridge and static TF publishers.

Usage (from host):
  docker compose exec -T sim bash -ic \
      "ros2 launch asv_sim sim.launch.py gui:=false"

Launch arguments:
  gui  (default 'true')  – set to 'false' for headless (server-only) mode
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

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Set to "false" for headless (server-only) Gazebo',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
