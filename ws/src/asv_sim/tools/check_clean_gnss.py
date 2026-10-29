#!/usr/bin/env python3
"""Clean-GNSS check (M2.2): does each Gazebo NavSat fix equal the true
antenna position?

With gazebo_sensor_noise false, every fix on /asv/gnss/fix and
/asv/gnss/secondary/fix should be the antenna's true position. For each fix k
at its own stamp t_k:

    truth     p(t_k) linear between the two ground-truth samples around t_k,
              q(t_k) by slerp (shortest way round)
    expected  a_k = p(t_k) + R(q(t_k)) l      (l = lever arm of the antenna,
                                                from config/asv_frames.yaml)
    measured  m_k = ENU of (lat_k, lon_k, alt_k)   (tools/geodesy.py)
    error     e_k = m_k - a_k

The lever arm is looked up by the fix's header.frame_id, so no sensor
position is typed in here. Prints max and RMS of the horizontal and vertical
error per antenna.

Usage (from the host; the bag was recorded with --use-sim-time):
    docker compose exec -T sim bash -ic \\
        "cd /ws/src/asv_sim && python3 tools/check_clean_gnss.py /ws/data/bags/m22_clean_gnss"
"""
import argparse
import bisect
import math
import os
import sys

import yaml

import geodesy as gd

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOPIC_ODOM = '/asv/ground_truth/odom'
TOPICS_FIX = ('/asv/gnss/fix', '/asv/gnss/secondary/fix')


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def lever_arms(frames_path):
    """{child frame: xyz} for the frames whose parent is base_link."""
    with open(frames_path) as f:
        frames = yaml.safe_load(f)['frames']
    return {fr['child']: fr['xyz'] for fr in frames if fr['parent'] == 'base_link'}


def slerp(q0, q1, s):
    """Spherical interpolation between unit quaternions (w, x, y, z), 0 <= s <= 1,
    the shortest way round (q and -q are the same rotation)."""
    dot = sum(a * b for a, b in zip(q0, q1))
    if dot < 0.0:
        q1, dot = tuple(-c for c in q1), -dot
    if dot > 0.9995:                      # nearly equal: linear, then normalise
        q = [a + s * (b - a) for a, b in zip(q0, q1)]
    else:
        th = math.acos(dot)
        w0, w1 = math.sin((1 - s) * th) / math.sin(th), math.sin(s * th) / math.sin(th)
        q = [w0 * a + w1 * b for a, b in zip(q0, q1)]
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)


def rotate(q, v):
    """R(q) v for a unit quaternion q = (w, x, y, z): body vector -> world."""
    w, x, y, z = q
    R = ((1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
         (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
         (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)))
    return tuple(sum(R[i][j] * v[j] for j in range(3)) for i in range(3))


def pose_at(truth, t):
    """(p, q) at time t from truth = sorted [(t, p, q)]; None outside its span."""
    times = [row[0] for row in truth]
    i = bisect.bisect_left(times, t)
    if i < len(times) and times[i] == t:
        return truth[i][1], truth[i][2]
    if i == 0 or i == len(times):
        return None
    (ta, pa, qa), (tb, pb, qb) = truth[i - 1], truth[i]
    s = (t - ta) / (tb - ta)
    return tuple(a + s * (b - a) for a, b in zip(pa, pb)), slerp(qa, qb, s)


def antenna_errors(truth, fixes, ltp, arm):
    """[(t, e_east, e_north, e_up)] for fixes = [(t, lat, lon, alt)]; fixes outside
    the ground-truth span are skipped."""
    out = []
    for t, lat, lon, alt in fixes:
        pose = pose_at(truth, t)
        if pose is None:
            continue
        p, q = pose
        a = [pi + ri for pi, ri in zip(p, rotate(q, arm))]
        m = ltp.to_enu(lat, lon, alt)
        out.append((t, m[0] - a[0], m[1] - a[1], m[2] - a[2]))
    return out


def summary(errors):
    """Max and RMS of the horizontal and vertical error, in m."""
    h = [math.hypot(e, n) for _, e, n, _ in errors]
    v = [abs(u) for *_, u in errors]
    rms = lambda xs: math.sqrt(sum(x * x for x in xs) / len(xs))  # noqa: E731
    return {'n': len(errors), 'h_max': max(h), 'h_rms': rms(h), 'v_max': max(v), 'v_rms': rms(v)}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag')
    ap.add_argument('--world', default=os.path.join(PKG, 'worlds', 'roboboat_course.sdf'))
    ap.add_argument('--frames', default=os.path.join(PKG, 'config', 'asv_frames.yaml'))
    args = ap.parse_args(argv)

    import analyze_run as ar        # needs rosbag2_py: only inside the container
    data = ar.read_bag(args.bag, (TOPIC_ODOM,) + TOPICS_FIX)
    truth = sorted((stamp(m), (m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z),
                    (m.pose.pose.orientation.w, m.pose.pose.orientation.x,
                     m.pose.pose.orientation.y, m.pose.pose.orientation.z))
                   for _, m in data[TOPIC_ODOM])
    ltp = gd.LocalTangentPlane(*gd.read_geo_reference(args.world))
    arms = lever_arms(args.frames)
    times = {t for t, *_ in truth}
    print(f'ground truth: {len(truth)} samples, {truth[0][0]:.3f} .. {truth[-1][0]:.3f} s')
    worst = 0.0
    for topic in TOPICS_FIX:
        msgs = [m for _, m in data.get(topic, [])]
        if not msgs:
            print(f'{topic}: no messages')
            return 1
        frame = msgs[0].header.frame_id
        fixes = [(stamp(m), m.latitude, m.longitude, m.altitude) for m in msgs]
        r = summary(antenna_errors(truth, fixes, ltp, arms[frame]))
        on_sample = sum(t in times for t, *_ in fixes)
        print(f'{topic} ({frame}, lever arm {arms[frame]} m): {r["n"]} fixes, '
              f'{on_sample} stamped exactly on a truth sample')
        print(f'  horizontal max {r["h_max"]:.3e} m, RMS {r["h_rms"]:.3e} m; '
              f'vertical max {r["v_max"]:.3e} m, RMS {r["v_rms"]:.3e} m')
        worst = max(worst, r['h_max'], r['v_max'])
    print(f'largest difference: {worst:.3e} m')
    return 0


if __name__ == '__main__':
    sys.exit(main())
