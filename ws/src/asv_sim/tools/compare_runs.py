#!/usr/bin/env python3
"""Maneuver repeatability (M1.7b): compare recorded runs of one maneuver.

Every pair of bags is lined up on tau = t - t_s, where t_s is the bag time of
the first RUNNING message of the maneuver player in that bag. Run B's true
pose is linearly interpolated at run A's tau samples, then
    e(tau)     = |p_B(tau) - p_A(tau)|          (m)
    e_psi(tau) = wrap(psi_B(tau) - psi_A(tau))  (rad)
over 0 <= tau <= t_N + coast, t_N = the maneuver table's end time.
Also per run: closest Task 1 buoy (analyze_run.min_clearances); per pair:
whether the thruster commands changed at the same tau.

Usage (from the host; bags recorded with --use-sim-time):
    docker compose exec -T sim bash -ic "cd /ws/src/asv_sim && \
        python3 tools/compare_runs.py --maneuver circle \
        /ws/data/bags/maneuver_circle_1 /ws/data/bags/maneuver_circle_2 ..."

Writes compare.json and compare.png to --out
(default /ws/data/runs/maneuver_<name>/).
"""
import argparse
import bisect
import itertools
import json
import math
import os
import sys

import yaml

import analyze_run as ar

PKG = ar.PKG
TOPIC_STATUS = '/asv/maneuver/status'
TOPIC_ODOM = ar.TOPIC_ODOM
TOPICS_CMD = ['/asv/thrusters/left/thrust', '/asv/thrusters/right/thrust',
              '/asv/thrusters/left/pos', '/asv/thrusters/right/pos']
COAST = 10.0  # s compared after the end of the table (the boat still moves)


def wrap(a):
    """Angle into (-pi, pi] (same convention as asv_sim::wrap_angle)."""
    w = math.atan2(math.sin(a), math.cos(a))
    return math.pi if w == -math.pi else w


def unwrap(angles):
    """Continuous angle sequence: removes the 2 pi jumps between samples."""
    out = []
    for a in angles:
        if not out:
            out.append(a)
        else:
            out.append(out[-1] + wrap(a - out[-1]))
    return out


def start_time(status):
    """Bag time of the first RUNNING status; None if the maneuver never ran."""
    for t, text in status:
        if text.startswith('RUNNING'):
            return t
    return None


def track(poses, t_s):
    """[(t, x, y, psi)] -> sorted [(tau, x, y, psi_unwrapped)] with tau = t - t_s."""
    poses = sorted(poses)
    psi = unwrap([p[3] for p in poses])
    return [(p[0] - t_s, p[1], p[2], s) for p, s in zip(poses, psi)]


def interpolate(trk, tau):
    """(x, y, psi) of the track at tau, linear between samples; None outside."""
    taus = [s[0] for s in trk]
    if not taus or tau < taus[0] or tau > taus[-1]:
        return None
    k = bisect.bisect_left(taus, tau)
    if taus[k] == tau:
        return trk[k][1:]
    a, b = trk[k - 1], trk[k]
    w = (tau - a[0]) / (b[0] - a[0])
    return tuple(a[i] + w * (b[i] - a[i]) for i in (1, 2, 3))


def compare(trk_a, trk_b, tau_end, tau_max):
    """Path difference of B against A on A's samples with 0 <= tau <= tau_max.

    Returns max e, RMS e, e at tau_end (= t_N), max |e_psi|, the compared
    tau range and the sample series (for the plot).
    """
    series = []
    for tau, x, y, psi in trk_a:
        if tau < 0.0 or tau > tau_max:
            continue
        pb = interpolate(trk_b, tau)
        if pb is None:
            continue
        series.append((tau, math.hypot(pb[0] - x, pb[1] - y), wrap(pb[2] - psi)))
    if not series:
        return None
    e = [s[1] for s in series]
    at_end = interpolate([(s[0], s[1], 0.0, 0.0) for s in series], tau_end)
    return {
        'max_e': max(e),
        'rms_e': ar.rms(e),
        'e_at_end': at_end[0] if at_end else math.nan,
        'max_abs_e_psi': max(abs(s[2]) for s in series),
        'tau_range': [series[0][0], series[-1][0]],
        'samples': len(series),
        'series': series,
    }


def command_changes(msgs, t_s):
    """[(tau, value)] at every change of a command topic (and its first value)."""
    out = []
    for t, m in msgs:
        if not out or m.data != out[-1][1]:
            out.append((t - t_s, m.data))
    return out


def command_offset(ch_a, ch_b):
    """Largest |tau_A - tau_B| over matching command changes.

    Returns None if the two runs do not have the same sequence of values
    (then they did not get the same commands).
    """
    if [v for _, v in ch_a] != [v for _, v in ch_b]:
        return None
    return max((abs(a[0] - b[0]) for a, b in zip(ch_a, ch_b)), default=0.0)


def load_end_time(name):
    """t_N of config/maneuvers/<name>.yaml (the last table time)."""
    path = os.path.join(PKG, 'config', 'maneuvers', name + '.yaml')
    with open(path, 'r') as f:
        p = yaml.safe_load(f)['maneuver_player']['ros__parameters']
    return float(p['times'][-1])


def analyze_runs(bags, t_end, buoys, length, half_width):
    runs = []
    for bag in bags:
        data = ar.read_bag(bag, [TOPIC_ODOM, TOPIC_STATUS] + TOPICS_CMD)
        status = [(t, m.data) for t, m in data.get(TOPIC_STATUS, [])]
        t_s = start_time(status)
        if t_s is None:
            raise ValueError(f'{bag}: no RUNNING on {TOPIC_STATUS}')
        poses = ar.odom_poses(data.get(TOPIC_ODOM, []))
        trk = track(poses, t_s)
        used = [(t, x, y, psi) for t, x, y, psi in poses
                if 0.0 <= t - t_s <= t_end + COAST]
        clear = ar.min_clearances(used, buoys, length, half_width)
        closest = min(clear.items(), key=lambda kv: kv[1][0])
        runs.append({
            'bag': bag,
            'start': t_s,
            'track': trk,
            'commands': {c: command_changes(data.get(c, []), t_s) for c in TOPICS_CMD},
            'closest_buoy': closest[0],
            'closest_clearance': closest[1][0],
            'end_pose': interpolate(trk, t_end),
        })
    return runs


def compare_pairs(runs, t_end):
    pairs = []
    for i, j in itertools.combinations(range(len(runs)), 2):
        c = compare(runs[i]['track'], runs[j]['track'], t_end, t_end + COAST)
        offsets = [command_offset(runs[i]['commands'][k], runs[j]['commands'][k])
                   for k in TOPICS_CMD]
        c['pair'] = [i + 1, j + 1]
        c['start_difference'] = runs[j]['start'] - runs[i]['start']
        c['same_commands'] = all(o is not None for o in offsets)
        c['max_command_offset'] = (max(offsets) if c['same_commands'] else None)
        pairs.append(c)
    return pairs


def print_report(name, t_end, runs, pairs):
    print(f'maneuver {name}: t_N = {t_end:.1f} s, compared tau in [0, {t_end + COAST:.1f}] s')
    for k, r in enumerate(runs, 1):
        end = r['end_pose']
        end_txt = f'({end[0]:.3f}, {end[1]:.3f}) m' if end else 'n/a'
        print(f'  run {k}: start {r["start"]:.3f} s, pose at t_N {end_txt}, '
              f'closest {r["closest_buoy"]} {r["closest_clearance"]:.3f} m  [{r["bag"]}]')
    # Millimetres in the printout: the differences are mm to um (compare.json: m).
    print('  pair  max e (mm)  RMS e (mm)  e(t_N) (mm)  max|e_psi| (rad)  tau range (s)  '
          'start diff (s)  same cmds  max cmd offset (s)')
    for p in pairs:
        off = f'{p["max_command_offset"]:.3f}' if p['same_commands'] else '-'
        print(f'  {p["pair"][0]}-{p["pair"][1]}   {1e3 * p["max_e"]:10.4f}  '
              f'{1e3 * p["rms_e"]:10.4f}  {1e3 * p["e_at_end"]:11.4f}  '
              f'{p["max_abs_e_psi"]:16.2e}  '
              f'{p["tau_range"][0]:5.2f}-{p["tau_range"][1]:6.2f}  '
              f'{p["start_difference"]:14.3f}  {str(p["same_commands"]):9}  {off}')


def plot(name, runs, pairs, course, out_png):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    # Categorical slots 1-3, validated (dataviz validate_palette.js, light):
    # contrast WARN for slot 3 -> every line is direct-labelled.
    colors = ['#2a78d6', '#eb6834', '#1baf7a']
    ink, muted = '#333333', '#8a8a85'
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    for k, r in enumerate(runs):
        xs = [s[1] for s in r['track'] if s[0] >= 0.0]
        ys = [s[2] for s in r['track'] if s[0] >= 0.0]
        ax1.plot(xs, ys, color=colors[k % 3], lw=2, label=f'run {k + 1}')
        ax1.annotate(f'run {k + 1}', (xs[-1], ys[-1]), color=ink, fontsize=8,
                     xytext=(4, 4 * (k - 1)), textcoords='offset points')
    for n, (bx, by, br) in course['buoys'].items():
        ax1.add_patch(plt.Circle((bx, by), br, color=muted))
    ax1.set_aspect('equal')
    ax1.set_xlabel('x east (m)')
    ax1.set_ylabel('y north (m)')
    ax1.set_title(f'{name}: true paths', color=ink)
    ax1.legend(frameon=False)
    for k, p in enumerate(pairs):
        lbl = f'{p["pair"][0]}-{p["pair"][1]}'
        taus = [s[0] for s in p['series']]
        es = [1e3 * s[1] for s in p['series']]
        ax2.plot(taus, es, color=colors[k % 3], lw=2, label=f'pair {lbl}')
        ax2.annotate(lbl, (taus[-1], es[-1]), color=ink, fontsize=8,
                     xytext=(4, 0), textcoords='offset points')
    ax2.set_xlabel('tau (s)')
    ax2.set_ylabel('e (mm)')
    ax2.set_title('position difference between runs', color=ink)
    ax2.legend(frameon=False)
    for ax in (ax1, ax2):
        ax.grid(True, color='#e6e6e3', lw=0.8)
        for side in ('top', 'right'):
            ax.spines[side].set_visible(False)
    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bags', nargs='+', help='two or more bag folders of the same maneuver')
    ap.add_argument('--maneuver', required=True, help='name in config/maneuvers/')
    ap.add_argument('--params', default=os.path.join(PKG, 'config', 'task1_route.yaml'))
    ap.add_argument('--asv', default=os.path.join(PKG, 'config', 'asv.yaml'))
    ap.add_argument('--out', default=None)
    args = ap.parse_args(argv)
    if len(args.bags) < 2:
        ap.error('need at least two bags')

    t_end = load_end_time(args.maneuver)
    course = ar.load_course(args.params)
    length, half_width = ar.load_boat(args.asv)
    runs = analyze_runs(args.bags, t_end, course['buoys'], length, half_width)
    pairs = compare_pairs(runs, t_end)
    print_report(args.maneuver, t_end, runs, pairs)

    out = args.out or f'/ws/data/runs/maneuver_{args.maneuver}'
    os.makedirs(out, exist_ok=True)
    result = {
        'maneuver': args.maneuver, 't_end': t_end, 'coast': COAST,
        'runs': [{k: v for k, v in r.items() if k not in ('track', 'commands')}
                 for r in runs],
        'pairs': [{k: v for k, v in p.items() if k != 'series'} for p in pairs],
    }
    with open(os.path.join(out, 'compare.json'), 'w') as f:
        json.dump(result, f, indent=2)
    plot(args.maneuver, runs, pairs, course, os.path.join(out, 'compare.png'))
    print(f'  wrote {out}/compare.json and compare.png')
    return 0


if __name__ == '__main__':
    sys.exit(main())
