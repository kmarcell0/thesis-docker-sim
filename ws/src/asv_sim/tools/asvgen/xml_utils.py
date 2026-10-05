"""Tiny helpers to build SDF as an XML element tree.

Building a tree (instead of pasting strings) guarantees well-formed XML and
proper escaping; the tree is pretty-printed when written.

    E('link', V('mass', 2.0), name='base_link')
      -> <link name="base_link"><mass>2</mass></link>
"""
import xml.etree.ElementTree as ET

COLORS = {   # RGB, 0..1
    'hull': (0.92, 0.92, 0.92), 'deck': (0.10, 0.30, 0.70), 'battery': (0.95, 0.45, 0.05),
    'electronics': (0.25, 0.25, 0.25), 'sensor': (0.05, 0.05, 0.05), 'gnss': (0.90, 0.85, 0.20),
    'pod': (0.15, 0.15, 0.15), 'mast': (0.30, 0.30, 0.30),
}


def fmt_value(v):
    if isinstance(v, bool):
        return 'true' if v else 'false'
    if isinstance(v, float):
        s = f'{v:.6g}'
        return '0' if s == '-0' else s
    if isinstance(v, (list, tuple)):
        return ' '.join(fmt_value(x) for x in v)
    return str(v)


def E(tag, *children, text=None, **attrs):
    """Element with attributes and children (None children are skipped, lists are flattened)."""
    el = ET.Element(tag, {k: fmt_value(v) for k, v in attrs.items()})
    if text is not None:
        el.text = fmt_value(text)
    for c in children:
        if c is None:
            continue
        if isinstance(c, (list, tuple)):
            el.extend([x for x in c if x is not None])
        else:
            el.append(c)
    return el


def V(tag, value):
    """Leaf element holding a value: V('mass', 2.0) -> <mass>2</mass>."""
    return E(tag, text=value)


def comment(text):
    return ET.Comment(f' {text} ')


def pose(xyz, rpy=(0.0, 0.0, 0.0)):
    return V('pose', [float(x) for x in xyz] + [float(a) for a in rpy])


def box(sx, sy, sz):
    return E('box', V('size', [float(sx), float(sy), float(sz)]))


def cylinder(radius, length):
    return E('cylinder', V('radius', float(radius)), V('length', float(length)))


def sphere(radius):
    return E('sphere', V('radius', float(radius)))


def material(color):
    rgb = list(COLORS[color])
    return E('material', V('ambient', rgb + [1.0]), V('diffuse', rgb + [1.0]),
             V('specular', [0.2, 0.2, 0.2, 1.0]))


def gaussian_noise(stddev, bias=0.0, drift=0.0, drift_time=0.0):
    """<noise type="gaussian"> as used by IMU and NavSat sensors.

    stddev      white noise per sample
    bias        standard deviation of a constant bias, drawn once per run
    drift       standard deviation of a slowly varying (Gauss-Markov) bias
    drift_time  correlation time of that drift, seconds
    """
    return E('noise',
             V('mean', 0.0), V('stddev', float(stddev)),
             [V('bias_mean', 0.0), V('bias_stddev', float(bias))] if bias else None,
             [V('dynamic_bias_stddev', float(drift)),
              V('dynamic_bias_correlation_time', float(drift_time))] if drift and drift_time else None,
             type='gaussian')


def to_string(root):
    tree = ET.ElementTree(root)
    ET.indent(tree, space='  ')
    return '<?xml version="1.0"?>\n' + ET.tostring(root, encoding='unicode') + '\n'
