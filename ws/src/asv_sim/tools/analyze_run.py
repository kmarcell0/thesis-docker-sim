#!/usr/bin/env python3
"""Task 1 run analysis (M1.5): read a recorded bag, print the metrics.

The gate crossings and buoy clearances are recomputed here from the ground
truth (decision E1 a), independently of the follower's C++ GateMonitor, with
the equations of milestone1_plan.md section 5.6. The node's own `GATE ...`
status lines are read too, and each offset is compared with the recomputed
one.

Usage (from the host; the bag was recorded with --use-sim-time):
    docker compose exec -T sim bash -ic \
        "cd /ws/src/asv_sim && python3 tools/analyze_run.py /ws/data/bags/task1_1"

Topics read: /asv/ground_truth/odom, /asv/autonomy/status,
/asv/autonomy/cross_track_error, /asv/thrusters/{left,right}/thrust (optional).
Writes metrics.json and trajectory.png into the bag folder (or --out).
Exit code 0 if criteria 2-4 of milestone1_plan.md section 1 hold, else 1.
"""
import argparse
import importlib.util
import json
import math
import os
import re
import sys

import yaml

PKG = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

TOPIC_ODOM = '/asv/ground_truth/odom'
TOPIC_STATUS = '/asv/autonomy/status'
TOPIC_CTE = '/asv/autonomy/cross_track_error'
TOPIC_LEFT = '/asv/thrusters/left/thrust'
TOPIC_RIGHT = '/asv/thrusters/right/thrust'

# The four gate passes Task 1 needs, in order (criterion 2).
EXPECTED_PASSES = [('gate1', 'OUTBOUND'), ('gate2', 'OUTBOUND'),
                   ('gate2', 'INBOUND'), ('gate1', 'INBOUND')]

# Same text as gate_event_text() in src/follower.cpp.
GATE_LINE = re.compile(r'^GATE (\S+) (OUTBOUND|INBOUND) offset ([+-]?\d+\.\d+) m (\S+)$')


# ---- configuration ---------------------------------------------------------

def load_course(params_path):
    """Buoys, gates, margin d_m and capture from the follower's params file."""
    with open(params_path, 'r') as f:
        p = yaml.safe_load(f)['waypoint_follower']['ros__parameters']
    names, coords, radii = p['buoy_names'], p['buoy_coords'], p['buoy_radii']
    if len(coords) != 2 * len(names) or len(radii) != len(names):
        raise ValueError('buoy_names, buoy_coords and buoy_radii do not match')
    buoys = {n: (float(coords[2 * i]), float(coords[2 * i + 1]), float(radii[i]))
             for i, n in enumerate(names)}
    gates = [(g, p['gate_red'][i], p['gate_green'][i]) for i, g in enumerate(p['gate_names'])]
    return {
        'buoys': buoys,
        'gates': gates,
        'margin': float(p['gate_margin']),
        'capture': float(p['gate_capture']),
        'waypoints': list(zip(p['waypoints_x'], p['waypoints_y'])),
    }


def load_boat(asv_path):
    """(L, b_w) from asv.yaml, through boat_params() of task1.launch.py."""
    spec = importlib.util.spec_from_file_location(
        'task1_launch', os.path.join(PKG, 'launch', 'task1.launch.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    with open(asv_path, 'r') as f:
        bp = mod.boat_params(yaml.safe_load(f))
    return bp['boat_length'], bp['boat_half_width']


# ---- pure maths (unit tested) ----------------------------------------------

def yaw_from_quaternion(w, x, y, z):
    """psi = atan2(2(w z + x y), 1 - 2(y^2 + z^2)), as asv_sim::yaw_from_quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def clearance(buoy, p, heading, length, half_width):
    """c_j: distance from the boat rectangle L x 2 b_w to the buoy's surface, m.

    Section 5.6: d = R(-psi)(c_j - p), k = d clamped to the rectangle,
    c_j = |d - k| - r_j (-r_j if the centre is inside).
    buoy = (x, y, r); p = (x, y); heading = psi.
    """
    wx, wy = buoy[0] - p[0], buoy[1] - p[1]
    c, s = math.cos(heading), math.sin(heading)
    dx, dy = c * wx + s * wy, -s * wx + c * wy
    kx = min(max(dx, -0.5 * length), 0.5 * length)
    ky = min(max(dy, -half_width), half_width)
    return math.hypot(dx - kx, dy - ky) - buoy[2]


def gate_crossing(red, green, p0, p1, half_width, margin, capture):
    """One step p0 -> p1 against gate (red a, green b); section 5.6.

    Returns None if the step does not cross the gate line (or crosses it more
    than `capture` beyond a buoy), else (t, offset, direction, result) with t
    the fraction of the step at the crossing.
    """
    ax, ay = red[0], red[1]
    gx, gy = green[0] - ax, green[1] - ay           # b - a

    def side(q):                                     # f(q) = (b - a) x (q - a)
        return gx * (q[1] - ay) - gy * (q[0] - ax)

    f0, f1 = side(p0), side(p1)
    if (f0 >= 0.0) == (f1 >= 0.0):                   # half-open, as in C++
        return None
    t = f0 / (f0 - f1)
    q = (p0[0] + t * (p1[0] - p0[0]), p0[1] + t * (p1[1] - p0[1]))
    width = math.hypot(gx, gy)                        # w_g
    offset = 0.5 * width - ((q[0] - ax) * gx + (q[1] - ay) * gy) / width
    if abs(offset) > 0.5 * width + capture:
        return None
    step_cross = gx * (p1[1] - p0[1]) - gy * (p1[0] - p0[0])
    direction = 'OUTBOUND' if step_cross > 0.0 else 'INBOUND'
    room = 0.5 * width - half_width - margin
    if abs(offset) > 0.5 * width:
        result = 'FAIL_OUTSIDE'
    elif offset <= room - red[2] and -offset <= room - green[2]:
        result = 'PASS'
    else:
        result = 'FAIL_CLOSE'
    return t, offset, direction, result


def gate_events(poses, course, half_width):
    """All gate events along poses [(t, x, y, psi), ...], in time order."""
    events = []
    buoys = course['buoys']
    for k in range(1, len(poses)):
        t0, x0, y0, _ = poses[k - 1]
        t1, x1, y1, _ = poses[k]
        for name, red, green in course['gates']:
            hit = gate_crossing(buoys[red], buoys[green], (x0, y0), (x1, y1),
                                half_width, course['margin'], course['capture'])
            if hit is None:
                continue
            frac, offset, direction, result = hit
            events.append({
                'gate': name, 'direction': direction, 'offset': offset, 'result': result,
                't': t0 + frac * (t1 - t0),
                'x': x0 + frac * (x1 - x0), 'y': y0 + frac * (y1 - y0),
            })
    return events


def min_clearances(poses, buoys, length, half_width):
    """Smallest clearance per buoy over poses: {name: (c_min, t)}."""
    out = {}
    for name, b in buoys.items():
        best = (math.inf, None)
        for t, x, y, psi in poses:
            c = clearance(b, (x, y), psi, length, half_width)
            if c < best[0]:
                best = (c, t)
        out[name] = best
    return out


def rms(values):
    """sqrt(mean(v_k^2)); nan for an empty list."""
    if not values:
        return math.nan
    return math.sqrt(sum(v * v for v in values) / len(values))


def reverse_share(left, right):
    """Fraction of commands with min(F_L, F_R) < 0. left/right: thrusts, paired by index."""
    n = min(len(left), len(right))
    if n == 0:
        return math.nan
    return sum(1 for k in range(n) if min(left[k], right[k]) < 0.0) / n


def parse_status(status):
    """From [(t, text), ...]: run start (first RUNNING), finish, stop, gate lines."""
    start = finish = stopped = None
    node_events = []
    for t, text in status:
        if start is None and text.startswith('RUNNING'):
            start = t
        elif finish is None and text == 'FINISHED':
            finish = t
        elif stopped is None and text.startswith('STOPPED'):
            stopped = (t, text)
        m = GATE_LINE.match(text)
        if m:
            node_events.append({'gate': m.group(1), 'direction': m.group(2),
                                'offset': float(m.group(3)), 'result': m.group(4), 't': t})
    return {'start': start, 'finish': finish, 'stopped': stopped, 'node_events': node_events}


def passes_in_order(events):
    """True if the PASS events contain the four Task 1 passes in order."""
    seq = [(e['gate'], e['direction']) for e in events if e['result'] == 'PASS']
    it = iter(seq)
    return all(any(s == want for s in it) for want in EXPECTED_PASSES)


# ---- bag I/O ---------------------------------------------------------------

def read_bag(path, topics=None):
    """{topic: [(t_bag, msg), ...]}; t_bag in s (simulation time with --use-sim-time).

    topics: the topics to read (default: the Task 1 topics above); topics not
    in the bag are left out.
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topics is None:
        topics = (TOPIC_ODOM, TOPIC_STATUS, TOPIC_CTE, TOPIC_LEFT, TOPIC_RIGHT)
    wanted = [t for t in topics if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    classes = {t: get_message(types[t]) for t in wanted}
    data = {t: [] for t in wanted}
    while reader.has_next():
        topic, raw, stamp_ns = reader.read_next()
        data[topic].append((stamp_ns * 1e-9, deserialize_message(raw, classes[topic])))
    return data


def odom_poses(msgs):
    """[(t, x, y, psi)] from Odometry messages; t from header.stamp."""
    poses = []
    for _, m in msgs:
        q = m.pose.pose.orientation
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        poses.append((t, m.pose.pose.position.x, m.pose.pose.position.y,
                      yaw_from_quaternion(q.w, q.x, q.y, q.z)))
    return poses


# ---- analysis --------------------------------------------------------------

def analyze(data, course, length, half_width):
    poses = odom_poses(data.get(TOPIC_ODOM, []))
    if len(poses) < 2:
        raise RuntimeError('fewer than two odometry messages in the bag')
    st = parse_status([(t, m.data) for t, m in data.get(TOPIC_STATUS, [])])
    events = gate_events(poses, course, half_width)
    clear = min_clearances(poses, course['buoys'], length, half_width)
    closest = min(clear.items(), key=lambda kv: kv[1][0])

    # Recomputed vs. node offsets, matched by order of (gate, direction).
    diffs = []
    pending = list(st['node_events'])
    for e in events:
        for i, n in enumerate(pending):
            if (n['gate'], n['direction']) == (e['gate'], e['direction']):
                diffs.append(e['offset'] - n['offset'])
                pending.pop(i)
                break

    cte = [m.data for _, m in data.get(TOPIC_CTE, [])]
    left = data.get(TOPIC_LEFT, [])
    right = data.get(TOPIC_RIGHT, [])
    thrust = None
    if left and right:
        t_lo = st['start'] if st['start'] is not None else -math.inf
        t_hi = st['finish'] if st['finish'] is not None else math.inf

        def running(msgs):
            return [m.data for t, m in msgs if t_lo <= t < t_hi]

        after = [m.data for t, m in left + right
                 if st['finish'] is not None and t > st['finish']]
        thrust = {
            'reverse_share_running': reverse_share(running(left), running(right)),
            'reverse_share_all': reverse_share([m.data for _, m in left],
                                               [m.data for _, m in right]),
            'commands_running': min(len(running(left)), len(running(right))),
            'min_thrust': min(m.data for _, m in left + right),
            'max_abs_after_finish': max((abs(v) for v in after), default=None),
        }

    run_time = (st['finish'] - st['start']
                if st['start'] is not None and st['finish'] is not None else None)
    finished_still = (st['finish'] is not None and thrust is not None
                      and thrust['max_abs_after_finish'] == 0.0)
    criteria = {
        '2_gates_both_ways': passes_in_order(events)
        and all(e['result'] == 'PASS' for e in events),
        '2_clearance_ge_margin': closest[1][0] >= course['margin'],
        '3_no_collision': closest[1][0] > 0.0,
        '4_finished_thrust_zero': finished_still,
    }
    return {
        'run_time': run_time,
        'stopped': st['stopped'][1] if st['stopped'] else None,
        'gate_events': events,
        'node_gate_events': st['node_events'],
        'offset_diff_max_abs': max((abs(d) for d in diffs), default=None),
        'offset_diffs': diffs,
        'min_clearance': {n: c for n, (c, _) in clear.items()},
        'closest_buoy': {'name': closest[0], 'clearance': closest[1][0], 't': closest[1][1]},
        'cross_track_max_abs': max((abs(e) for e in cte), default=None),
        'cross_track_rms': rms(cte),
        'thrust': thrust,
        'criteria': criteria,
        'poses': poses,
    }


def fmt(v, spec='.3f'):
    return 'n/a' if v is None or (isinstance(v, float) and math.isnan(v)) else format(v, spec)


def print_report(r, bag):
    print(f'Run: {bag}')
    print(f'  run time (RUNNING -> FINISHED, sim time): {fmt(r["run_time"], ".2f")} s')
    if r['stopped']:
        print(f'  {r["stopped"]}')
    print('  gate events (recomputed from ground truth):')
    for e in r['gate_events']:
        print(f'    {e["gate"]} {e["direction"]:8s} offset {e["offset"]:+.3f} m '
              f'{e["result"]}  t = {e["t"]:.2f} s')
    print(f'  node gate events: {len(r["node_gate_events"])}; '
          f'largest |recomputed - node| offset: {fmt(r["offset_diff_max_abs"], ".4f")} m')
    cb = r['closest_buoy']
    print(f'  closest buoy: {cb["name"]} {cb["clearance"]:.3f} m at t = {fmt(cb["t"], ".2f")} s')
    print(f'  cross-track error: max |e| {fmt(r["cross_track_max_abs"])} m, '
          f'RMS {fmt(r["cross_track_rms"])} m')
    th = r['thrust']
    if th:
        print(f'  thrust: reverse share {100 * th["reverse_share_running"]:.1f} % of '
              f'{th["commands_running"]} RUNNING commands '
              f'({100 * th["reverse_share_all"]:.1f} % of all), '
              f'smallest {th["min_thrust"]:.1f} N, '
              f'max |F| after FINISHED {fmt(th["max_abs_after_finish"], ".1f")} N')
    else:
        print('  thrust: topics not in the bag')
    print('  criteria (milestone1_plan.md section 1):')
    for k, ok in r['criteria'].items():
        print(f'    {k}: {"OK" if ok else "FAIL"}')


def plot(r, course, out_png):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(10, 6))
    wx, wy = zip(*course['waypoints'])
    ax.plot(wx, wy, 'k--', lw=0.8, label='route')
    ax.plot(wx, wy, 'k.', ms=4)
    xs = [p[1] for p in r['poses']]
    ys = [p[2] for p in r['poses']]
    ax.plot(xs, ys, 'b-', lw=1.2, label='driven path (ground truth)')
    for name, (x, y, rad) in course['buoys'].items():
        colour = 'red' if 'red' in name else 'green' if 'green' in name else 'grey'
        ax.add_patch(plt.Circle((x, y), rad, color=colour))
    for e in r['gate_events']:
        ax.plot(e['x'], e['y'], 'x', color='k' if e['result'] == 'PASS' else 'm', ms=8)
    ax.set_aspect('equal')
    ax.set_xlabel('x (east) [m]')
    ax.set_ylabel('y (north) [m]')
    ax.grid(True, lw=0.3)
    ax.legend(loc='best', fontsize=8)
    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag', help='bag folder')
    ap.add_argument('--params', default=os.path.join(PKG, 'config', 'task1_route.yaml'))
    ap.add_argument('--asv', default=os.path.join(PKG, 'config', 'asv.yaml'))
    ap.add_argument('--out', default=None, help='output folder (default: the bag folder)')
    args = ap.parse_args(argv)

    course = load_course(args.params)
    length, half_width = load_boat(args.asv)
    r = analyze(read_bag(args.bag), course, length, half_width)
    print_report(r, args.bag)

    out = args.out or args.bag
    with open(os.path.join(out, 'metrics.json'), 'w') as f:
        json.dump({k: v for k, v in r.items() if k != 'poses'}, f, indent=2)
    plot(r, course, os.path.join(out, 'trajectory.png'))
    print(f'  wrote {os.path.join(out, "metrics.json")} and trajectory.png')
    return 0 if all(r['criteria'].values()) else 1


if __name__ == '__main__':
    sys.exit(main())
