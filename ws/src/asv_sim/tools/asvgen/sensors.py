"""Sensor models.

Each entry of `sensors:` in the YAML is expanded into a SensorPart that holds
everything the rest of the pipeline needs:

    sdf      <sensor> elements that go into base_link
    bridge   topics the ROS <-> Gazebo bridge must translate
    frames   static TF frames (parent, child, xyz, rpy)
    visuals  what the sensor looks like (and how tall it is, for its mount)
    masses   MassItems for the mass budget

All sensors are attached to base_link with a pose; that pose is also written
as the sensor's TF frame, so the simulated data and the frames can never
disagree. Topic and frame names are derived from the sensor's `name`.

Supported types and what they become in Gazebo:
    lidar          gpu_lidar   (a regular grid of beams)
    camera         camera
    stereo_camera  camera (left) + camera (right) [+ depth_camera]
    imu            imu         (noise from datasheet noise densities)
    gnss           navsat
    gnss_dual      navsat (primary antenna) + navsat (secondary antenna)
"""
import math
from dataclasses import dataclass, field

from . import geometry as geo
from .mass import MassItem
from .xml_utils import E, V, pose, box, cylinder, gaussian_noise

# ROS optical frame (z forward, x right, y down) relative to a camera body frame
OPTICAL_RPY = [-math.pi / 2, 0.0, -math.pi / 2]


@dataclass
class Frame:
    parent: str
    child: str
    xyz: list
    rpy: list


@dataclass
class BridgeTopic:
    topic: str
    ros_type: str
    gz_type: str
    direction: str = 'GZ_TO_ROS'


@dataclass
class VisualSpec:
    name: str
    xyz: list
    rpy: list
    geometry: object          # xml Element
    color: str
    size: tuple               # body size along the sensor's own x, y, z axes (m)

    @property
    def half_extents(self):
        """Half sizes of the axis-aligned box enclosing the rotated body:
        h_i = sum_j |R_ij| s_j / 2."""
        R = geo.rot(self.rpy)
        return [sum(abs(R[i][j]) * self.size[j] / 2 for j in range(3)) for i in range(3)]


@dataclass
class SensorPart:
    name: str
    type: str
    spec_name: str
    spec: dict
    cfg: dict
    sdf: list = field(default_factory=list)
    bridge: list = field(default_factory=list)
    frames: list = field(default_factory=list)
    visuals: list = field(default_factory=list)
    masses: list = field(default_factory=list)


# Physical size of the sensor bodies (x, y, z in the sensor's own frame), from
# datasheets where known. The geometry is what Gazebo draws (and sensors see).
BODY = {
    'livox_box': (box(0.065, 0.065, 0.060), (0.065, 0.065, 0.060)),   # Mid-360: 65 x 65 x 60 mm
    'stereo': (box(0.030, 0.175, 0.043), (0.030, 0.175, 0.043)),      # ZED 2i: 175 x 30 x 43 mm
    'camera': (box(0.030, 0.040, 0.040), (0.030, 0.040, 0.040)),
    'antenna': (cylinder(0.045, 0.020), (0.090, 0.090, 0.020)),
}


# --------------------------------------------------------------------------- SDF
def _sensor(name, stype, xyz, rpy, frame, topic, rate, *inner):
    return E('sensor',
             pose(xyz, rpy),
             V('gz_frame_id', frame),
             V('always_on', True),
             V('update_rate', float(rate)),
             V('topic', topic),
             *inner,
             name=name, type=stype)


def _camera_block(hfov, width, height, image_format='R8G8B8', clip=(0.1, 300.0), noise=None):
    return E('camera',
             V('horizontal_fov', float(hfov)),
             E('image', V('width', int(width)), V('height', int(height)), V('format', image_format)),
             E('clip', V('near', float(clip[0])), V('far', float(clip[1]))),
             E('noise', V('type', 'gaussian'), V('mean', 0.0), V('stddev', float(noise))) if noise else None)


def _info_topic(image_topic):
    """Gazebo publishes camera_info next to the image topic: .../x/image -> .../x/camera_info"""
    return image_topic.rsplit('/', 1)[0] + '/camera_info'


def _image_bridge(topic):
    return [BridgeTopic(topic, 'sensor_msgs/msg/Image', 'gz.msgs.Image'),
            BridgeTopic(_info_topic(topic), 'sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo')]


# ------------------------------------------------------------------------ builders
def build_lidar(s, spec, ns):
    """3D lidar as a regular grid: `channels` beams spread over vfov, `h_samples`
    points per beam over 360 deg. The horizontal range stops one step short of
    +180 deg so the first and last column do not overlap."""
    n, frame, topic = s['name'], f"{s['name']}_link", f"/{ns}/{s['name']}"
    lo, hi = spec['vfov']
    h = spec['h_samples']
    lidar = E('lidar',
              E('scan',
                E('horizontal', V('samples', h), V('resolution', 1),
                  V('min_angle', -math.pi), V('max_angle', math.pi - 2 * math.pi / h)),
                E('vertical', V('samples', spec['channels']), V('resolution', 1),
                  V('min_angle', float(lo)), V('max_angle', float(hi)))),
              E('range', V('min', float(spec['range'][0])), V('max', float(spec['range'][1])),
                V('resolution', 0.01)),
              E('noise', V('type', 'gaussian'), V('mean', 0.0), V('stddev', float(spec['noise']))),
              V('visibility_mask', 7))       # bit 3 off: VRX water (flag 8) is invisible to it
    geom, size = BODY['livox_box']
    return dict(
        sdf=[_sensor(n, 'gpu_lidar', s['xyz'], s['rpy'], frame, topic, spec['rate'], lidar)],
        bridge=[BridgeTopic(f'{topic}/points', 'sensor_msgs/msg/PointCloud2', 'gz.msgs.PointCloudPacked')],
        frames=[Frame('base_link', frame, s['xyz'], s['rpy'])],
        visuals=[VisualSpec(f'{n}_body', s['xyz'], s['rpy'], geom, 'sensor', size)],
        masses=[MassItem(n, spec['mass'], s['xyz'], group='sensors')])


def build_camera(s, spec, ns):
    n, topic = s['name'], f"/{ns}/{s['name']}/image_raw"
    geom, size = BODY['camera']
    cam = _camera_block(spec['hfov'], spec['width'], spec['height'], noise=spec.get('noise'))
    return dict(
        sdf=[_sensor(n, 'camera', s['xyz'], s['rpy'], f'{n}_optical', topic, spec['rate'], cam)],
        bridge=_image_bridge(topic),
        frames=[Frame('base_link', f'{n}_link', s['xyz'], s['rpy']),
                Frame(f'{n}_link', f'{n}_optical', [0.0, 0.0, 0.0], OPTICAL_RPY)],
        visuals=[VisualSpec(f'{n}_body', s['xyz'], s['rpy'], geom, 'sensor', size)],
        masses=[MassItem(n, spec['mass'], s['xyz'], group='sensors')])


def build_stereo(s, spec, ns):
    """Two cameras `baseline` apart along the body's y axis (left = +y), each
    with its own optical frame. Optionally an ideal depth image from the left
    camera's viewpoint, standing in for the depth the vendor SDK computes."""
    n, b = s['name'], spec['baseline']
    out = dict(sdf=[], bridge=[], frames=[Frame('base_link', f'{n}_link', s['xyz'], s['rpy'])])
    for side, dy in (('left', b / 2), ('right', -b / 2)):
        xyz = geo.to_parent(s['xyz'], s['rpy'], [0.0, dy, 0.0])
        topic = f'/{ns}/{n}/{side}/image_raw'
        cam = _camera_block(spec['hfov'], spec['width'], spec['height'], noise=spec.get('noise'))
        out['sdf'].append(_sensor(f'{n}_{side}', 'camera', xyz, s['rpy'], f'{n}_{side}_optical',
                                  topic, spec['rate'], cam))
        out['bridge'] += _image_bridge(topic)
        out['frames'].append(Frame(f'{n}_link', f'{n}_{side}_optical', [0.0, dy, 0.0], OPTICAL_RPY))
    if spec.get('depth'):
        xyz = geo.to_parent(s['xyz'], s['rpy'], [0.0, b / 2, 0.0])
        topic = f'/{ns}/{n}/depth/image'
        cam = _camera_block(spec['hfov'], spec['depth_width'], spec['depth_height'],
                            image_format='R_FLOAT32', clip=spec['depth_range'])
        out['sdf'].append(_sensor(f'{n}_depth', 'depth_camera', xyz, s['rpy'], f'{n}_left_optical',
                                  topic, spec['depth_rate'], cam))
        out['bridge'] += _image_bridge(topic)
        out['bridge'].append(BridgeTopic(f'{topic}/points', 'sensor_msgs/msg/PointCloud2',
                                         'gz.msgs.PointCloudPacked'))
    geom, size = BODY['stereo']
    out['visuals'] = [VisualSpec(f'{n}_body', s['xyz'], s['rpy'], geom, 'sensor', size)]
    out['masses'] = [MassItem(n, spec['mass'], s['xyz'], group='sensors')]
    return out


def imu_white_noise(spec):
    """Per-sample white noise from datasheet noise densities:
         sigma = noise_density * sqrt(sample_rate)
       (the noise power spread over the bandwidth that one sample covers)."""
    root = math.sqrt(spec['rate'])
    return spec['gyro_noise_density'] * root, spec['accel_noise_density'] * root


def build_imu(s, spec, ns):
    n, topic = s['name'], f"/{ns}/{s['name']}/data"
    g, a = imu_white_noise(spec)
    tau = spec.get('bias_drift_time', 0.0)

    def axes(white, bias, drift):
        # Gazebo's drifting bias is a first-order Gauss-Markov process driven by
        # dynamic_bias_stddev = sigma_b; its steady-state standard deviation is
        # sigma_b * sqrt(tau / 2). The YAML gives the steady-state value, so convert.
        sigma_b = drift / math.sqrt(tau / 2) if drift and tau else 0.0
        return [E(ax, gaussian_noise(white, bias, sigma_b, tau)) for ax in 'xyz']

    imu = E('imu',
            E('orientation_reference_frame', V('localization', 'ENU')),
            E('angular_velocity', *axes(g, spec.get('gyro_bias', 0.0), spec.get('gyro_bias_drift', 0.0))),
            E('linear_acceleration', *axes(a, spec.get('accel_bias', 0.0), spec.get('accel_bias_drift', 0.0))))
    return dict(
        sdf=[_sensor(n, 'imu', s['xyz'], s['rpy'], f'{n}_link', topic, spec['rate'], imu)],
        bridge=[BridgeTopic(topic, 'sensor_msgs/msg/Imu', 'gz.msgs.IMU')],
        frames=[Frame('base_link', f'{n}_link', s['xyz'], s['rpy'])],
        visuals=[],                                   # sits inside the electronics box
        masses=[MassItem(n, spec['mass'], s['xyz'], group='sensors')] if spec['mass'] > 0 else [])


def _navsat(name, xyz, rpy, frame, topic, spec):
    return _sensor(name, 'navsat', xyz, rpy, frame, topic, spec['rate'],
                   E('navsat', E('position_sensing',
                                 E('horizontal', gaussian_noise(spec['horizontal_noise'])),
                                 E('vertical', gaussian_noise(spec['vertical_noise'])))))


def build_gnss(s, spec, ns):
    n, topic = s['name'], f"/{ns}/{s['name']}/fix"
    geom, size = BODY['antenna']
    return dict(
        sdf=[_navsat(n, s['xyz'], s['rpy'], f'{n}_link', topic, spec)],
        bridge=[BridgeTopic(topic, 'sensor_msgs/msg/NavSatFix', 'gz.msgs.NavSat')],
        frames=[Frame('base_link', f'{n}_link', s['xyz'], s['rpy'])],
        visuals=[VisualSpec(f'{n}_antenna', s['xyz'], s['rpy'], geom, 'gnss', size)],
        masses=[MassItem(n, spec['mass'], s['xyz'], group='sensors')])


def build_gnss_dual(s, spec, ns):
    """Dual-antenna receiver: the primary antenna gives the position, the
    primary -> secondary baseline gives the heading. Both antennas are
    simulated as NavSat sensors; the heading itself is NOT computed from them
    (independent noise per antenna would be far worse than a real receiver,
    which solves the baseline jointly) - see analysis.py."""
    n, rpy = s['name'], s['rpy']
    p1, p2 = s['xyz'], s['xyz_secondary']
    t1, t2 = f'/{ns}/{n}/fix', f'/{ns}/{n}/secondary/fix'
    geom, size = BODY['antenna']
    return dict(
        sdf=[_navsat(f'{n}_primary', p1, rpy, f'{n}_primary_link', t1, spec),
             _navsat(f'{n}_secondary', p2, rpy, f'{n}_secondary_link', t2, spec)],
        bridge=[BridgeTopic(t1, 'sensor_msgs/msg/NavSatFix', 'gz.msgs.NavSat'),
                BridgeTopic(t2, 'sensor_msgs/msg/NavSatFix', 'gz.msgs.NavSat')],
        frames=[Frame('base_link', f'{n}_primary_link', p1, rpy),
                Frame('base_link', f'{n}_secondary_link', p2, rpy)],
        visuals=[VisualSpec(f'{n}_primary_antenna', p1, rpy, geom, 'gnss', size),
                 VisualSpec(f'{n}_secondary_antenna', p2, rpy, geom, 'gnss', size)],
        masses=[MassItem(f'{n}_primary', spec['mass'], p1, group='sensors'),
                MassItem(f'{n}_secondary', spec['mass'], p2, group='sensors')])


BUILDERS = {'lidar': build_lidar, 'camera': build_camera, 'stereo_camera': build_stereo,
            'imu': build_imu, 'gnss': build_gnss, 'gnss_dual': build_gnss_dual}


def expand(cfg):
    """All sensors of the configuration as SensorParts (in YAML order)."""
    parts = []
    for s in cfg['sensors']:
        spec = cfg['sensor_specs'][s['spec']]
        data = BUILDERS[s['type']](s, spec, cfg['namespace'])
        parts.append(SensorPart(name=s['name'], type=s['type'], spec_name=s['spec'], spec=spec, cfg=s, **data))
    return parts
