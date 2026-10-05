"""Read, override and validate the boat description (config/asv.yaml).

Validation happens before anything is built, so a typo gives one clear
message ("sensors[2] (zed).xyz: expected 3 numbers") instead of a broken
model. All errors are collected and reported together; unknown keys only
produce warnings (they are probably typos, but they do no harm).
"""
import copy
import hashlib

import yaml


class ConfigError(Exception):
    pass


NUM, INT, BOOL, STR = 'number', 'integer', 'bool', 'string'
VEC2, VEC3 = 'vec2', 'vec3'

SECTIONS = {
    'name': STR,
    'namespace': STR,
    'hull': {'length': NUM, 'radius': NUM, 'spacing': NUM, 'mass': NUM},
    'deck': {'length': NUM, 'width': NUM, 'thickness': NUM, 'height': NUM, 'mass': NUM},
    'thrusters': {'x': NUM, 'depth': NUM, 'mass': NUM, 'max_thrust': NUM, 'min_thrust': NUM,
                  'steer_limit': NUM, 'steer_rate': NUM},
    'hydrodynamics': {k: NUM for k in ('xU', 'xUU', 'yV', 'yVV', 'zW', 'kP', 'kPP', 'mQ', 'mQQ', 'nR', 'nRR')},
    'battery': {'mass': NUM, 'size': VEC3, 'xyz': VEC3},
}
OPTIONAL_SECTIONS = ('fixed_masses',)
OPTIONAL_FIELDS = {'hull': {'points': INT}}     # buoyancy sample points per hull (default 4)
FIXED_MASS = {'name': STR, 'mass': NUM, 'size': VEC3, 'xyz': VEC3}

SENSOR_COMMON = {'name': STR, 'type': STR, 'spec': STR, 'xyz': VEC3}
SENSOR_OPTIONAL = {'rpy': VEC3, 'mount': VEC2, 'xyz_secondary': VEC3}
SENSOR_EXTRA_REQUIRED = {'gnss_dual': {'xyz_secondary': VEC3}}

_CAMERA = {'mass': NUM, 'hfov': NUM, 'width': INT, 'height': INT, 'rate': NUM}
_GNSS = {'mass': NUM, 'rate': NUM, 'horizontal_noise': NUM, 'vertical_noise': NUM}
SPEC_REQUIRED = {
    'lidar': {'mass': NUM, 'vfov': VEC2, 'channels': INT, 'h_samples': INT, 'range': VEC2,
              'rate': NUM, 'noise': NUM},
    'camera': _CAMERA,
    'stereo_camera': dict(_CAMERA, baseline=NUM),
    'imu': {'mass': NUM, 'rate': NUM, 'gyro_noise_density': NUM, 'accel_noise_density': NUM},
    'gnss': _GNSS,
    'gnss_dual': dict(_GNSS, heading_accuracy_1m=NUM),
}
SPEC_OPTIONAL = {
    'camera': {'noise': NUM},
    'stereo_camera': {'noise': NUM, 'depth': BOOL, 'depth_range': VEC2, 'depth_width': INT,
                      'depth_height': INT, 'depth_rate': NUM},
    'imu': {'gyro_bias': NUM, 'accel_bias': NUM, 'gyro_bias_drift': NUM, 'accel_bias_drift': NUM,
            'bias_drift_time': NUM},
}
DEPTH_KEYS = ('depth_range', 'depth_width', 'depth_height', 'depth_rate')
SENSOR_TYPES = tuple(SPEC_REQUIRED)


# --------------------------------------------------------------------------- load
def load(path):
    try:
        with open(path) as f:
            cfg = yaml.safe_load(f)
    except FileNotFoundError:
        raise ConfigError(f'config file not found: {path}')
    except yaml.YAMLError as e:
        raise ConfigError(f'{path} is not valid YAML:\n{e}')
    if not isinstance(cfg, dict):
        raise ConfigError(f'{path}: expected a mapping at the top level')
    return cfg


def fingerprint(cfg):
    """Short hash of the effective configuration, written into every output."""
    text = yaml.safe_dump(cfg, sort_keys=True)
    return hashlib.sha256(text.encode()).hexdigest()[:12]


# ------------------------------------------------------------------ --set overrides
def apply_override(cfg, assignment):
    """Apply 'a.b.c=value' to cfg. Sensors are addressed by name:
         battery.xyz=[-0.1,0,0.22]
         sensors.lidar.xyz=[0.6,0,0.5]
         sensor_specs.zed2i_4mm.depth=false
       The value is parsed as YAML (numbers, lists, true/false, strings)."""
    if '=' not in assignment:
        raise ConfigError(f'--set "{assignment}": expected key.path=value')
    path, raw = assignment.split('=', 1)
    keys = path.strip().split('.')
    value = yaml.safe_load(raw)
    node = cfg
    i = 0
    while i < len(keys) - 1:
        k = keys[i]
        if k == 'sensors' and isinstance(node.get('sensors'), list):
            name = keys[i + 1]
            match = [s for s in node['sensors'] if s.get('name') == name]
            if not match:
                raise ConfigError(f'--set {path}: no sensor named "{name}"')
            node = match[0]
            i += 2
            continue
        if k not in node or not isinstance(node[k], dict):
            raise ConfigError(f'--set {path}: "{k}" not found')
        node = node[k]
        i += 1
    if i != len(keys) - 1:
        raise ConfigError(f'--set {path}: incomplete key path')
    node[keys[-1]] = value


# ---------------------------------------------------------------------- validation
def _type_ok(value, kind):
    def num(v):
        return isinstance(v, (int, float)) and not isinstance(v, bool)
    if kind == NUM:
        return num(value)
    if kind == INT:
        return isinstance(value, int) and not isinstance(value, bool)
    if kind == BOOL:
        return isinstance(value, bool)
    if kind == STR:
        return isinstance(value, str)
    if kind in (VEC2, VEC3):
        n = 2 if kind == VEC2 else 3
        return isinstance(value, list) and len(value) == n and all(num(v) for v in value)
    return True


def _check_fields(obj, required, optional, where, errors, warnings):
    if not isinstance(obj, dict):
        errors.append(f'{where}: expected a mapping')
        return
    for key, kind in required.items():
        if key not in obj:
            errors.append(f'{where}.{key}: missing')
        elif not _type_ok(obj[key], kind):
            errors.append(f'{where}.{key}: expected {kind}, got {obj[key]!r}')
    for key, kind in optional.items():
        if key in obj and not _type_ok(obj[key], kind):
            errors.append(f'{where}.{key}: expected {kind}, got {obj[key]!r}')
    for key in obj:
        if key not in required and key not in optional:
            warnings.append(f'{where}.{key}: unknown key (typo?) - ignored')


def validate(cfg):
    """Check the configuration. Returns warnings; raises ConfigError on errors."""
    errors, warnings = [], []
    for section, schema in SECTIONS.items():
        if section not in cfg:
            errors.append(f'{section}: missing')
        elif isinstance(schema, dict):
            _check_fields(cfg[section], schema, OPTIONAL_FIELDS.get(section, {}), section, errors, warnings)
        elif not _type_ok(cfg[section], schema):
            errors.append(f'{section}: expected {schema}')
    for key in cfg:
        if key not in SECTIONS and key not in OPTIONAL_SECTIONS + ('sensors', 'sensor_specs'):
            warnings.append(f'{key}: unknown top-level key - ignored')
    for i, f in enumerate(cfg.get('fixed_masses', []) or []):
        _check_fields(f, FIXED_MASS, {}, f'fixed_masses[{i}]', errors, warnings)

    specs = cfg.get('sensor_specs')
    sensors = cfg.get('sensors')
    if not isinstance(specs, dict):
        errors.append('sensor_specs: missing or not a mapping')
        specs = {}
    if not isinstance(sensors, list) or not sensors:
        errors.append('sensors: missing or empty list')
        sensors = []

    names, used_as = set(), {}
    for i, s in enumerate(sensors):
        where = f'sensors[{i}]' + (f' ({s.get("name")})' if isinstance(s, dict) and 'name' in s else '')
        if not isinstance(s, dict):
            errors.append(f'{where}: expected a mapping')
            continue
        stype = s.get('type')
        extra = SENSOR_EXTRA_REQUIRED.get(stype, {})
        optional = {k: v for k, v in SENSOR_OPTIONAL.items() if k not in extra}
        _check_fields(s, dict(SENSOR_COMMON, **extra), optional, where, errors, warnings)
        if stype not in SENSOR_TYPES:
            errors.append(f'{where}.type: "{stype}" is not one of {", ".join(SENSOR_TYPES)}')
            continue
        if s.get('name') in names:
            errors.append(f'{where}.name: "{s["name"]}" used twice (names become topics and frames)')
        names.add(s.get('name'))
        spec = s.get('spec')
        if spec not in specs:
            errors.append(f'{where}.spec: no entry "{spec}" in sensor_specs')
            continue
        prev = used_as.setdefault(spec, stype)
        if prev != stype:
            errors.append(f'{where}: spec "{spec}" is used by a {prev} and a {stype}')

    for spec_name, stype in used_as.items():
        spec = specs[spec_name]
        where = f'sensor_specs.{spec_name}'
        _check_fields(spec, SPEC_REQUIRED[stype], SPEC_OPTIONAL.get(stype, {}), where, errors, warnings)
        if stype == 'stereo_camera' and isinstance(spec, dict) and spec.get('depth'):
            for k in DEPTH_KEYS:
                if k not in spec:
                    errors.append(f'{where}.{k}: required when depth: true')

    if errors:
        raise ConfigError('invalid configuration:\n  ' + '\n  '.join(errors))
    return warnings


def prepare(path, overrides=()):
    """load + overrides + validate + defaults. Returns (cfg, warnings, fingerprint)."""
    cfg = copy.deepcopy(load(path))
    for o in overrides:
        apply_override(cfg, o)
    warnings = validate(cfg)
    cfg.setdefault('fixed_masses', [])
    cfg['hull'].setdefault('points', 4)
    if cfg['hull']['points'] < 1:
        raise ConfigError('hull.points must be at least 1')
    for s in cfg['sensors']:
        s['rpy'] = [float(a) for a in s.get('rpy', [0.0, 0.0, 0.0])]
        s['xyz'] = [float(a) for a in s['xyz']]
        if 'xyz_secondary' in s:
            s['xyz_secondary'] = [float(a) for a in s['xyz_secondary']]
    return cfg, warnings, fingerprint(cfg)
