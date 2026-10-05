"""Assemble the whole boat from the validated configuration.

The boat is one rigid body (base_link) plus two moving thruster pods:

    base_link            hulls, deck, electronics, battery, all sensors
    <side>_engine_link   steering pod, revolute joint about z  (steering)
    <side>_prop_link     propeller, revolute joint about x     (thrust axis)

Everything rigidly attached is lumped into base_link's single inertial
(mass, centre of gravity, inertia tensor from mass.combine). The pods keep
their own small inertials because they move relative to the hull.

Plugins (Gazebo / VRX systems) attached to the model:
    vrx::Surface x2                        buoyancy of each hull, follows the waves
    vrx::SimpleHydrodynamics               linear + quadratic water drag
    gz Thruster x2                         force along the propeller axis
    gz JointPositionController x2          steering angle of each pod
    gz OdometryPublisher                   ground truth pose/twist (topic only)
"""
import math
from dataclasses import dataclass, field

from . import geometry as geo
from . import hydrostatics as hydro
from . import mass as mass_mod
from . import sensors as sensors_mod
from .mass import MassItem
from .xml_utils import E, V, pose, box, cylinder, sphere, material, comment

PROP_RADIUS = 0.045          # visual only
ENGINE_SHARE = 0.85          # share of a thruster unit's mass in the pod (rest: propeller)


@dataclass
class Solid:
    """Axis-aligned box used for overlap checks and line-of-sight tests."""
    owner: str        # sensor name, component name, or 'boat'
    label: str
    kind: str         # 'structure' (hull, deck), 'component', 'sensor', 'mount'
    lo: list
    hi: list


@dataclass
class Boat:
    cfg: dict
    fingerprint: str
    parts: list                                  # SensorParts
    visuals: list = field(default_factory=list)  # base_link visual elements
    collisions: list = field(default_factory=list)
    mass_items: list = field(default_factory=list)
    pod_items: list = field(default_factory=list)
    base: object = None                          # MassProperties of base_link
    total: object = None                         # MassProperties incl. pods
    hydro: object = None                         # Hydrostatics
    links: list = field(default_factory=list)    # pod link/joint elements
    plugins: list = field(default_factory=list)
    occluders: list = field(default_factory=list)   # (owner, label, xyz): points sensors may see
    extents: list = field(default_factory=list)     # points bounding the boat (size check)
    solids: list = field(default_factory=list)      # Solids: layout check + line of sight
    mounts: list = field(default_factory=list)      # sensor posts/arms, for the layout figure
    deck_top: float = 0.0
    z_prop: float = 0.0


def _visual(name, xyz, rpy, geometry, color):
    return E('visual', pose(xyz, rpy), E('geometry', geometry), material(color), name=name)


def _collision(name, xyz, rpy, geometry):
    return E('collision', pose(xyz, rpy), E('geometry', geometry), name=name)


def build(cfg, fingerprint):
    h, d, t = cfg['hull'], cfg['deck'], cfg['thrusters']
    r, hl, hy = h['radius'], h['length'], h['spacing'] / 2
    boat = Boat(cfg=cfg, fingerprint=fingerprint, parts=sensors_mod.expand(cfg))
    boat.deck_top = d['height'] + d['thickness'] / 2
    boat.z_prop = -r - t['depth']

    _structure(boat, h, d, r, hl, hy)
    _sensor_mounts(boat, d)
    _masses(boat, h, d, t, r, hl, hy)
    try:
        boat.hydro = hydro.solve(h, boat.total.mass, boat.total.cog, boat.total.tensor)
    except hydro.HydrostaticsError as e:
        raise SystemExit(f'ERROR: {e}')
    _pods(boat, t, r, hy)
    _plugins(boat, cfg, h, t, hl, hy)
    return boat


# ---------------------------------------------------------------- hull and deck
def _structure(boat, h, d, r, hl, hy):
    """Hulls are drawn as a cylinder with half-sphere ends, but collide (and
    float) as plain cylinders of the full length, matching vrx::Surface."""
    for side, y in (('left', hy), ('right', -hy)):
        boat.visuals.append(_visual(f'hull_{side}', [0, y, 0], [0, math.pi / 2, 0],
                                    cylinder(r, hl - 2 * r), 'hull'))
        for end, x in (('bow', hl / 2 - r), ('stern', -hl / 2 + r)):
            boat.visuals.append(_visual(f'hull_{side}_{end}', [x, y, 0], [0, 0, 0], sphere(r), 'hull'))
        boat.collisions.append(_collision(f'hull_{side}', [0, y, 0], [0, math.pi / 2, 0], cylinder(r, hl)))
        boat.solids.append(Solid('boat', f'hull {side}', 'structure', [-hl / 2, y - r, -r], [hl / 2, y + r, r]))
        for x in (hl / 2, -hl / 2):
            boat.extents.append([x, y + (r if y > 0 else -r), -r])
        boat.occluders += [('boat', f'hull {side} bow', [hl / 2 - r / 2, y, r * 0.8]),
                           ('boat', f'hull {side} stern', [-hl / 2 + r / 2, y, r * 0.8])]
    deck = box(d['length'], d['width'], d['thickness'])
    boat.visuals.append(_visual('deck', [0, 0, d['height']], [0, 0, 0], deck, 'deck'))
    boat.collisions.append(_collision('deck', [0, 0, d['height']], [0, 0, 0], deck))
    boat.solids.append(Solid('boat', 'deck', 'structure',
                             [-d['length'] / 2, -d['width'] / 2, d['height'] - d['thickness'] / 2],
                             [d['length'] / 2, d['width'] / 2, boat.deck_top]))
    for sx in (1, -1):
        for sy in (1, -1):
            corner = [sx * d['length'] / 2, sy * d['width'] / 2, boat.deck_top]
            boat.extents.append(corner)
            boat.occluders.append(('boat', f'deck {"front" if sx > 0 else "rear"} '
                                           f'{"left" if sy > 0 else "right"} corner', corner))
    bat = dict(boat.cfg['battery'], name='battery')
    for f, color in [(f, 'electronics') for f in boat.cfg['fixed_masses']] + [(bat, 'battery')]:
        boat.visuals.append(_visual(f['name'], f['xyz'], [0, 0, 0], box(*f['size']), color))
        boat.solids.append(Solid(f['name'], f['name'], 'component',
                                 [f['xyz'][k] - f['size'][k] / 2 for k in range(3)],
                                 [f['xyz'][k] + f['size'][k] / 2 for k in range(3)]))


# --------------------------------------------------------------- sensor mounts
def hull_top(cfg, x, y):
    """Height of the top of a hull at (x, y), or None if (x, y) is not above a hull.
    Hulls are cylinders with hemispherical ends (as drawn)."""
    h = cfg['hull']
    r, half, s = h['radius'], h['length'] / 2, h['spacing'] / 2
    for yh in (s, -s):
        dy = abs(y - yh)
        dx = max(0.0, abs(x) - (half - r))           # > 0 only on the rounded ends
        if abs(x) <= half and dx * dx + dy * dy <= r * r:
            return math.sqrt(r * r - dx * dx - dy * dy)
    return None


def on_deck(d, x, y):
    return abs(x) <= d['length'] / 2 and abs(y) <= d['width'] / 2


def _sensor_mounts(boat, d):
    """Every sensor stands on a post (or hangs from it, if upside down).

    Where the post stands:
      * at the sensor's `mount: [x, y]` if given (must be on the deck or a hull),
      * otherwise straight below the sensor if that is on the deck or on a hull,
      * otherwise at the nearest point of the deck, with an arm out to the sensor.
    Posts on a hull start at the hull's top surface.

    Posts, arms and sensor bodies are visuals, so simulated lidars and cameras
    see them - as on the real boat. They are also registered as Solids, for the
    layout overlap check and the line-of-sight analysis.
    """
    for part in boat.parts:
        mount = part.cfg.get('mount')
        for vs in part.visuals:
            x, y, z = vs.xyz
            hx, hy_, hz = vs.half_extents
            boat.visuals.append(_visual(vs.name, vs.xyz, vs.rpy, vs.geometry, vs.color))
            boat.occluders.append((part.name, vs.name, list(vs.xyz)))
            boat.solids.append(Solid(part.name, vs.name, 'sensor', [x - hx, y - hy_, z - hz], [x + hx, y + hy_, z + hz]))
            hanging = geo.is_upside_down(vs.rpy)
            arm_z = z + hz + 0.015 if hanging else z - hz - 0.005
            top = z + hz + (0.02 if hanging else 0.0)
            boat.extents += [[x + hx, y + hy_, top], [x - hx, y - hy_, z - hz]]

            if mount:
                mx, my = mount
            elif on_deck(d, x, y) or hull_top(boat.cfg, x, y) is not None:
                mx, my = x, y
            else:
                mx = max(-d['length'] / 2, min(d['length'] / 2, x))
                my = max(-d['width'] / 2, min(d['width'] / 2, y))
            if on_deck(d, mx, my):
                base = boat.deck_top
            else:
                base = hull_top(boat.cfg, mx, my)
                if base is None:
                    raise SystemExit(f'ERROR: sensor "{part.name}": mount point ({mx:.2f}, {my:.2f}) '
                                     'is neither on the deck nor on a hull')
            height = arm_z - base
            if height < 0.03:
                continue                     # sits directly on the deck / hull
            boat.visuals.append(_visual(f'{vs.name}_post', [mx, my, base + height / 2], [0, 0, 0],
                                        cylinder(0.015, height), 'mast'))
            boat.occluders.append((part.name, f'{vs.name} post', [mx, my, base + height * 0.6]))
            boat.solids.append(Solid(part.name, f'{vs.name} post', 'mount',
                                     [mx - 0.015, my - 0.015, base], [mx + 0.015, my + 0.015, arm_z]))
            entry = {'owner': part.name, 'base': [mx, my, base], 'top_z': arm_z, 'end': None}
            reach = math.hypot(x - mx, y - my)
            if reach > 0.02:
                boat.visuals.append(_visual(f'{vs.name}_arm', [(x + mx) / 2, (y + my) / 2, arm_z],
                                            [0, 0, math.atan2(y - my, x - mx)],
                                            box(reach + 0.03, 0.025, 0.01), 'mast'))
                boat.solids.append(Solid(part.name, f'{vs.name} arm', 'mount',
                                         [min(x, mx) - 0.0125, min(y, my) - 0.0125, arm_z - 0.005],
                                         [max(x, mx) + 0.0125, max(y, my) + 0.0125, arm_z + 0.005]))
                entry['end'] = [x, y, arm_z]
                boat.occluders.append((part.name, f'{vs.name} arm', [(x + mx) / 2, (y + my) / 2, arm_z]))
            boat.mounts.append(entry)


# ------------------------------------------------------------------ mass budget
def _masses(boat, h, d, t, r, hl, hy):
    items = [
        MassItem('hull left', h['mass'], [0, hy, 0], mass_mod.cylinder_x(h['mass'], r, hl), 'structure'),
        MassItem('hull right', h['mass'], [0, -hy, 0], mass_mod.cylinder_x(h['mass'], r, hl), 'structure'),
        MassItem('deck', d['mass'], [0, 0, d['height']],
                 mass_mod.box(d['mass'], d['length'], d['width'], d['thickness']), 'structure'),
    ]
    for f in boat.cfg['fixed_masses']:
        items.append(MassItem(f['name'], f['mass'], f['xyz'], mass_mod.box(f['mass'], *f['size']), 'components'))
    bat = boat.cfg['battery']
    items.append(MassItem('battery', bat['mass'], bat['xyz'], mass_mod.box(bat['mass'], *bat['size']), 'components'))
    for part in boat.parts:
        items += part.masses
    pods = []
    for side, y in (('left', hy), ('right', -hy)):
        pods.append(MassItem(f'{side} pod', ENGINE_SHARE * t['mass'], [t['x'], y, (-r + boat.z_prop) / 2],
                             group='propulsion'))
        pods.append(MassItem(f'{side} propeller', (1 - ENGINE_SHARE) * t['mass'],
                             [t['x'] - 0.06, y, boat.z_prop], group='propulsion'))
        boat.extents.append([t['x'] - 0.06, y, boat.z_prop - PROP_RADIUS])
    boat.mass_items, boat.pod_items = items, pods
    boat.base = mass_mod.combine(items)
    boat.total = mass_mod.combine(items + pods)


# ---------------------------------------------------------------- thruster pods
def _pods(boat, t, r, hy):
    """Steerable (azimuth) thruster = two joints, as on VRX's WAM-V:
    a steering joint (z axis) driven to an angle, and under it a propeller
    joint (x axis) along which the Thruster system pushes, so the thrust
    direction turns with the pod."""
    z_rel = boat.z_prop + r             # propeller axis relative to the engine link origin
    strut_top = 0.3 * r                 # strut reaches a bit into the hull
    m_eng, m_prop = ENGINE_SHARE * t['mass'], (1 - ENGINE_SHARE) * t['mass']
    for side, y in (('left', hy), ('right', -hy)):
        boat.links += [
            comment(f'{side} steerable thruster: steering joint (z) + propeller joint (x)'),
            E('link',
              pose([t['x'], y, -r]),
              E('inertial', pose([0, 0, z_rel / 2]), V('mass', m_eng),
                E('inertia', V('ixx', 0.002), V('iyy', 0.002), V('izz', 0.001),
                  V('ixy', 0.0), V('ixz', 0.0), V('iyz', 0.0))),
              _visual('strut', [0, 0, (strut_top + z_rel) / 2], [0, 0, 0],
                      box(0.04, 0.015, strut_top - z_rel), 'pod'),
              _visual('motor', [0, 0, z_rel], [0, math.pi / 2, 0], cylinder(0.03, 0.10), 'pod'),
              name=f'{side}_engine_link'),
            E('joint',
              V('parent', 'base_link'), V('child', f'{side}_engine_link'),
              E('axis', V('xyz', [0.0, 0.0, 1.0]),
                E('limit', V('lower', -float(t['steer_limit'])), V('upper', float(t['steer_limit'])))),
              name=f'{side}_steer_joint', type='revolute'),
            E('link',
              pose([t['x'] - 0.06, y, boat.z_prop]),
              E('inertial', V('mass', m_prop),
                E('inertia', V('ixx', 0.0001), V('iyy', 0.0001), V('izz', 0.0001),
                  V('ixy', 0.0), V('ixz', 0.0), V('iyz', 0.0))),
              _visual('propeller', [0, 0, 0], [0, math.pi / 2, 0], cylinder(PROP_RADIUS, 0.01), 'pod'),
              name=f'{side}_prop_link'),
            E('joint',
              V('parent', f'{side}_engine_link'), V('child', f'{side}_prop_link'),
              E('axis', V('xyz', [1.0, 0.0, 0.0]), E('limit', V('lower', -1e16), V('upper', 1e16))),
              name=f'{side}_prop_joint', type='revolute'),
        ]


# ---------------------------------------------------------------------- plugins
def _plugins(boat, cfg, h, t, hl, hy):
    ns = cfg['namespace']
    p = boat.plugins
    n, r = h['points'], h['radius']
    for side, y in (('left', hy), ('right', -hy)):
        # vrx::Surface samples each hull at N points. Each point stands for a
        # hull segment of length L/N and is the segment's KEEL (bottom): the
        # plugin measures the immersion from the point up to the (wavy) water
        # surface, caps it at the radius and applies rho*g*A_seg*L/N there.
        # Points sit at the middle of N equal segments.
        pts = [V('point', [-hl / 2 + (i + 0.5) * hl / n, y, -r]) for i in range(n)]
        p += [comment(f'buoyancy of the {side} hull (VRX): {n} keel points, reacts to the waves'),
              E('plugin',
                V('link_name', 'base_link'), V('hull_length', float(hl)), V('hull_radius', float(r)),
                V('fluid_level', 0.0),
                E('points', *pts),
                E('wavefield', V('topic', '/vrx/wavefield/parameters')),
                filename='libSurface.so', name='vrx::Surface')]
    hyd = cfg['hydrodynamics']
    p += [comment('water drag (VRX): linear + quadratic damping per axis, uncalibrated'),
          E('plugin', V('link_name', 'base_link'),
            V('xDotU', 0.0), V('yDotV', 0.0), V('nDotR', 0.0),
            *[V(k, float(v)) for k, v in hyd.items()],
            filename='libSimpleHydrodynamics.so', name='vrx::SimpleHydrodynamics')]
    for side in ('left', 'right'):
        p += [comment(f'{side}: thrust [N] on /{ns}/thrusters/{side}/thrust, '
                      f'steering angle [rad] on /{ns}/thrusters/{side}/pos (gz.msgs.Double)'),
              E('plugin',
                V('namespace', ns), V('topic', f'thrusters/{side}/thrust'),
                V('joint_name', f'{side}_prop_joint'),
                V('thrust_coefficient', 0.004422), V('fluid_density', hydro.RHO),
                V('propeller_diameter', 2 * PROP_RADIUS), V('velocity_control', True),
                V('max_thrust_cmd', float(t['max_thrust'])), V('min_thrust_cmd', float(t['min_thrust'])),
                filename='gz-sim-thruster-system', name='gz::sim::systems::Thruster'),
              E('plugin',
                V('joint_name', f'{side}_steer_joint'), V('topic', f'/{ns}/thrusters/{side}/pos'),
                V('use_velocity_commands', True),
                V('cmd_max', float(t['steer_rate'])), V('cmd_min', -float(t['steer_rate'])),
                filename='gz-sim-joint-position-controller-system',
                name='gz::sim::systems::JointPositionController')]
    p += [comment('ground truth for evaluating SLAM: topic only, NOT TF'),
          E('plugin',
            V('odom_frame', 'world'), V('robot_base_frame', 'base_link'), V('dimensions', 3),
            V('odom_publish_frequency', 50.0), V('odom_topic', f'/{ns}/ground_truth/odom'),
            V('tf_topic', f'/{ns}/ground_truth/pose_v'),
            filename='gz-sim-odometry-publisher-system', name='gz::sim::systems::OdometryPublisher')]
