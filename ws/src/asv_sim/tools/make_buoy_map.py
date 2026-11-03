#!/usr/bin/env python3
"""True buoy map (M2.7, D7 a, D19 a, D20 a): writes config/buoys_gt.yaml.

Reads where the course puts every floating object and writes one list with
name, class, colour, radius and world position (x east, y north, m). Only
files are read, no simulation: the buoys are static (D7 a), so the positions
in the files are the true positions for the whole run.

    world file   worlds/roboboat_course.sdf   task pose  (x_T, y_T, psi_T)
    task model   models/rb26_task*/model.sdf  local pose (x_l, y_l) of each object
    element      models/rb26_<element>/model.sdf   collision shapes -> radius

    x_w = x_T + cos(psi_T) x_l - sin(psi_T) y_l
    y_w = y_T + sin(psi_T) x_l + cos(psi_T) y_l

Class and colour come from the element model name (rb26_a2_yellow -> A-2,
yellow). D19 a: Sur-Mark, A-0, A-2, indicator buoys and vessels (class
"other") are listed; dock piers and dock indicators are fixed structure and
left out. D20 a: radius = largest horizontal distance from the model origin to
the edge of its collision shapes (sphere and cylinder: their radius; boxes:
the circumscribed circle).

Usage (from ws/src/asv_sim):
    python3 tools/make_buoy_map.py            # writes config/buoys_gt.yaml
"""
import argparse
import math
import os
import xml.etree.ElementTree as ET

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
WORLD = os.path.join(PKG, 'worlds', 'roboboat_course.sdf')
MODELS = os.path.join(PKG, 'models')
OUT = os.path.join(PKG, 'config', 'buoys_gt.yaml')

# Element model name prefix -> class (D19 a). Order matters: longest first is
# not needed, the prefixes don't overlap.
CLASSES = {'rb26_surmark_': 'Sur-Mark', 'rb26_a0_': 'A-0', 'rb26_a2_': 'A-2',
           'rb26_indicator_buoy_': 'indicator', 'rb26_vessel_': 'other'}
SKIPPED = ('rb26_dock_',)  # fixed structure, not on the map
COLOURS = ('red', 'green', 'yellow', 'black')  # asv_msgs/BuoyColour


def parse_pose(el):
    """(x, y, z, roll, pitch, yaw) of a <pose> element; zeros if missing."""
    if el is None or not (el.text or '').strip():
        return (0.0,) * 6
    v = [float(t) for t in el.text.split()]
    if len(v) != 6:
        raise ValueError(f'pose needs 6 numbers, got {el.text!r}')
    return tuple(v)


def compose(task, local):
    """World (x, y) of a local (x, y) in a task placed at (x_T, y_T, psi_T)."""
    xt, yt, psi = task
    xl, yl = local
    c, s = math.cos(psi), math.sin(psi)
    return (xt + c * xl - s * yl, yt + s * xl + c * yl)


def uri_model(uri):
    prefix = 'model://'
    if not uri.startswith(prefix):
        raise ValueError(f'unexpected uri {uri!r}')
    return uri[len(prefix):]


def includes(sdf_path):
    """[(name, model, pose, static)] of the <include>s of the file's model or world."""
    root = ET.parse(sdf_path).getroot()
    parent = root.find('model')
    if parent is None:
        parent = root.find('world')
    out = []
    for inc in parent.findall('include'):
        name = inc.findtext('name', '').strip()
        model = uri_model(inc.findtext('uri', '').strip())
        static = inc.findtext('static', 'false').strip() == 'true'
        out.append((name, model, parse_pose(inc.find('pose')), static))
    return out


def classify(model):
    """(class, colour) of an element model, or None for skipped structure."""
    if model.startswith(SKIPPED):
        return None
    for prefix, cls in CLASSES.items():
        if model.startswith(prefix):
            colour = model[len(prefix):]
            if colour not in COLOURS:
                raise ValueError(f'{model}: unknown colour {colour!r}')
            return cls, colour
    raise ValueError(f'{model}: not a known course element')


def horizontal_radius(model_dir):
    """Largest horizontal distance from the model origin to its collision edge (D20 a)."""
    root = ET.parse(os.path.join(model_dir, 'model.sdf')).getroot()
    r_max = 0.0
    for col in root.iter('collision'):
        x, y, _, roll, pitch, yaw = parse_pose(col.find('pose'))
        if roll or pitch or yaw:
            raise ValueError(f'{model_dir}: rotated collision shapes are not handled')
        geom = col.find('geometry')
        sphere, cyl, box = geom.find('sphere'), geom.find('cylinder'), geom.find('box')
        if sphere is not None or cyl is not None:
            r = float((sphere if sphere is not None else cyl).findtext('radius'))
            r_max = max(r_max, math.hypot(x, y) + r)
        elif box is not None:
            sx, sy, _ = (float(t) for t in box.findtext('size').split())
            r_max = max(r_max, math.hypot(abs(x) + sx / 2, abs(y) + sy / 2))
        else:
            raise ValueError(f'{model_dir}: unhandled collision geometry')
    if r_max == 0.0:
        raise ValueError(f'{model_dir}: no collision shape')
    return r_max


def build_map(world=WORLD, models=MODELS):
    """List of dicts (id, name, class, colour, radius, x, y), in file order."""
    buoys = []
    for task_name, task_model, pose, _ in includes(world):
        if not task_model.startswith('rb26_task'):
            continue
        x, y, _, roll, pitch, psi = pose
        if roll or pitch:
            raise ValueError(f'{task_name}: tilted task pose')
        for name, model, local, _ in includes(os.path.join(models, task_model, 'model.sdf')):
            kind = classify(model)
            if kind is None:
                continue
            xw, yw = compose((x, y, psi), local[:2])
            buoys.append({'id': len(buoys), 'name': f'{task_name}::{name}',
                          'class': kind[0], 'colour': kind[1],
                          'radius': horizontal_radius(os.path.join(models, model)),
                          'x': xw, 'y': yw})
    return buoys


HEADER = """\
# True buoy map (M2.7, D7 a, D19 a, D20 a). GENERATED by
# tools/make_buoy_map.py from worlds/roboboat_course.sdf and models/rb26_*;
# never edit by hand. [REPO]
# Frame: map = world (ENU, x east, y north), metres. radius: largest
# horizontal radius of the collision shape (D20 a). class "other" = vessel:
# reported by the fake detector, never mapped by the filter (design.md 5).
"""


def to_yaml(buoys):
    return HEADER + yaml.safe_dump({'frame_id': 'map', 'buoys': buoys},
                                   sort_keys=False, default_flow_style=False)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out', default=OUT)
    args = ap.parse_args()
    buoys = build_map()
    with open(args.out, 'w') as f:
        f.write(to_yaml(buoys))
    counts = {}
    for b in buoys:
        counts[b['class']] = counts.get(b['class'], 0) + 1
    print(f'{args.out}: {len(buoys)} objects {counts}')


if __name__ == '__main__':
    main()
