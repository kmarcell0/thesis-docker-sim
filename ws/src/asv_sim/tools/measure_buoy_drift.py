#!/usr/bin/env python3
"""Buoy drift (M2.7, D7): how far do FLOATING course objects move on their own?

Run once with the floating (pre-M2.7) course models, the boat idle. Reads
Gazebo's pose message /world/<world>/pose/info (gz.msgs.Pose_V, text output of
`gz topic -e`) once at the start and once when the simulation time has
advanced by --duration. Every course object is a model nested in its task
model, and its pose there is relative to the task, so

    p_world = p_task + R(q_task) p_local

The task models have no links of their own; their pose is taken from the same
message. For every object on the true map (config/buoys_gt.yaml) it prints the
horizontal and vertical movement between the two samples, and the horizontal
distance at the end from its file position (= spawn position), and writes them
to a JSON file. Must run inside the container while the simulation is up.

Usage:
    python3 tools/measure_buoy_drift.py --duration 300 --out /ws/data/m27_drift/drift.json
"""
import argparse
import json
import math
import os
import re
import subprocess
import sys
import time

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
MAP = os.path.join(PKG, 'config', 'buoys_gt.yaml')


def read_pose_v(topic):
    """One Pose_V message: (stamp_s, [(name, (x, y, z), (w, x, y, z))]) in message order."""
    out = subprocess.run(['gz', 'topic', '-e', '-t', topic, '-n', '1'], capture_output=True,
                         text=True, timeout=30, check=True).stdout
    return parse_pose_v(out)


def parse_pose_v(text):
    """Parse the protobuf text output of gz.msgs.Pose_V (fields left at 0 are omitted)."""
    m = re.search(r'stamp \{\s*(?:sec: (\d+))?\s*(?:nsec: (\d+))?\s*\}', text)
    stamp = int(m.group(1) or 0) + int(m.group(2) or 0) * 1e-9
    poses = []
    for block in re.findall(r'\npose \{(.*?)\n\}', text, re.S):
        name = re.search(r'name: "([^"]*)"', block).group(1)

        def fields(sub, keys, default):
            mm = re.search(sub + r' \{([^}]*)\}', block)
            vals = dict(re.findall(r'(\w): (\S+)', mm.group(1))) if mm else {}
            return tuple(float(vals.get(k, default[i])) for i, k in enumerate(keys))
        p = fields('position', 'xyz', (0, 0, 0))
        q = fields('orientation', 'wxyz', (1, 0, 0, 0)) if 'orientation' in block else (1, 0, 0, 0)
        poses.append((name, p, q))
    return stamp, poses


def rotate(q, v):
    """v rotated by the unit quaternion q = (w, x, y, z)."""
    w, x, y, z = q
    tx, ty, tz = 2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0])
    return (v[0] + w * tx + y * tz - z * ty, v[1] + w * ty + z * tx - x * tz,
            v[2] + w * tz + x * ty - y * tx)


def world_positions(poses, tasks):
    """{task::name: world position} for the children of the task models.

    The message lists a task model followed by its nested models, so a child
    belongs to the last task name seen."""
    out, task = {}, None
    for name, p, q in poses:
        if name in tasks:
            task = (name, p, q)
        elif name in ('asv', 'link'):
            task = None  # the boat and the link entries come after the course
        elif task is not None:
            r = rotate(task[2], p)
            out[f'{task[0]}::{name}'] = tuple(task[1][i] + r[i] for i in range(3))
    return out


def drift(first, last, buoys):
    """Rows (name, class, horizontal m, vertical m) for every mapped object."""
    rows = []
    for b in buoys:
        a, c = first[b['name']], last[b['name']]
        rows.append({'name': b['name'], 'class': b['class'],
                     'horizontal_m': math.hypot(c[0] - a[0], c[1] - a[1]),
                     'vertical_m': c[2] - a[2],
                     'from_map_m': math.hypot(c[0] - b['x'], c[1] - b['y'])})
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--world', default='roboboat_course')
    ap.add_argument('--duration', type=float, default=300.0, help='simulated seconds')
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    topic = f'/world/{args.world}/pose/info'
    with open(MAP) as f:
        buoys = yaml.safe_load(f)['buoys']
    tasks = {b['name'].split('::')[0] for b in buoys}

    t0, poses0 = read_pose_v(topic)
    while True:
        time.sleep(10.0)
        t1, poses1 = read_pose_v(topic)
        if t1 - t0 >= args.duration:
            break
    rows = drift(world_positions(poses0, tasks), world_positions(poses1, tasks), buoys)
    # Check of the method: Gazebo moves the model poses (each relative to its
    # parent); the links must stay at the model origin, else the model poses
    # alone would miss motion.
    link_offset = max(math.hypot(math.hypot(*p[:2]), p[2]) for n, p, _ in poses1 if n == 'link')
    rows.sort(key=lambda r: -r['horizontal_m'])
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, 'w') as f:
        json.dump({'t0_s': t0, 't1_s': t1, 'largest_link_offset_m': link_offset, 'rows': rows},
                  f, indent=1)
    print(f'sim time {t0:.3f} s -> {t1:.3f} s ({t1 - t0:.1f} s)')
    print(f'largest link offset from its model origin: {link_offset:.3g} m (must be ~0)')
    print(f'{"object":48s} {"class":9s} {"horiz. m":>10s} {"vert. m":>10s} {"from map m":>10s}')
    for r in rows:
        print(f'{r["name"]:48s} {r["class"]:9s} {r["horizontal_m"]:10.4f} {r["vertical_m"]:10.4f}'
              f' {r["from_map_m"]:10.4f}')
    per_class = {}
    for r in rows:
        per_class[r['class']] = max(per_class.get(r['class'], 0.0), r['horizontal_m'])
    print('largest horizontal movement per class (m):',
          {k: f'{v:.3g}' for k, v in per_class.items()})
    return 0


if __name__ == '__main__':
    sys.exit(main())
