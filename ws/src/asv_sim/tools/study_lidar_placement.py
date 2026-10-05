#!/usr/bin/env python3
"""Lidar placement study: sweep the lidar position and mount, keep the
placements that satisfy the hard constraints, and rank the rest.

Hard constraints (a placement is rejected if any fails):
  * the boat fits the RoboBoat 6 x 3 x 3 ft box
  * no overlapping components, sensors or mounts
  * no part of the lidar (body, post, arm) appears in any camera image

Variables: lidar x and height, lidar mount (post straight below, or beside
the cameras with an arm), height of the two GNSS antennas.

Ranking (lower is better), the front first because that is where the boat goes.
All terms come from the line-of-sight water visibility map (analysis.py):
  score = gap_ahead                          blind water ahead of the bow (m)
        + W_LANE * (1 - coverage_front_lane) blind share of the lane ahead (3 m)
        + W_BAND * (1 - coverage_near_band)  blind share of the water within 2 m all around
        + W_SELF * self_hit_share            share of beams wasted on the own boat
With the weights below, losing 10 % of the lane ahead costs as much as a
0.3 m longer blind gap at the bow.

Usage (from ws/src/asv_sim):
  python3 tools/study_lidar_placement.py            # table + config/lidar_study.csv (+ .png)
"""
import copy
import csv
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from asvgen import analysis, boat as boat_mod, config  # noqa: E402

PKG = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
LIDAR = 'lidar'
XS = [round(0.05 * i, 2) for i in range(0, 18)]          # 0.00 .. 0.85 m along the centreline
ZS = [0.45, 0.50, 0.55, 0.60, 0.62]                       # height of the lidar (boat frame)
ANTENNA_ZS = [0.45, 0.60, 0.68]                           # height of both GNSS antennas
SIDE_POST_Y = 0.30                                        # 'side' mount: post beside the cameras
W_LANE, W_BAND, W_SELF = 3.0, 1.5, 1.0


def evaluate(base, x, z, mount, ant_z):
    cfg = copy.deepcopy(base)
    lidar = next(s for s in cfg['sensors'] if s['name'] == LIDAR)
    lidar['xyz'] = [x, 0.0, z]
    lidar.pop('mount', None)
    if mount == 'side':
        d = cfg['deck']
        lidar['mount'] = [min(x, d['length'] / 2 - 0.02), SIDE_POST_Y]
    for s in cfg['sensors']:
        if s['type'] == 'gnss_dual':
            s['xyz'][2] = s['xyz_secondary'][2] = ant_z
    row = {'x': x, 'z': z, 'mount': mount, 'ant_z': ant_z}
    try:
        boat = boat_mod.build(cfg, 'study')
    except SystemExit as e:                               # e.g. impossible mount point
        return dict(row, valid=False, why=str(e))
    m, _ = analysis.analyze(boat)
    lid = m['sensors'][LIDAR]
    nf = lid['near_field']
    seen_by_camera = sorted({cam for cam, cm in m['sensors'].items()
                             if cm['type'] in ('camera', 'stereo_camera')
                             and any(lbl.startswith(LIDAR) for lbl in cm['sees_own_boat'])})
    why = []
    if not m['roboboat']['fits_box']:
        why.append('too tall/big')
    if not m['layout']['ok']:
        why.append('overlap')
    if seen_by_camera:
        why.append('lidar in camera image')
    reach = lid['max_distance_2_beams_m']
    row.update({'valid': not why, 'why': '; '.join(why),
                'gap_ahead': nf['gap_ahead_m'], 'gap_behind': nf['gap_behind_m'], 'gap_sides': nf['gap_sides_m'],
                'lane': nf['coverage_front_lane'], 'band': nf['coverage_near_band'],
                'self_hit': nf['self_hit_share'], 'a0': reach['A-0 buoy'], 'a2': reach['A-2 buoy'],
                'surmark': reach['Sur-Mark buoy'], 'height': m['roboboat']['size_m'][2],
                'trim': m['hydrostatics']['trim_deg']})
    row['score'] = round(row['gap_ahead'] + W_LANE * (1 - row['lane']) + W_BAND * (1 - row['band'])
                         + W_SELF * row['self_hit'], 3)
    return row


def show(rows, title):
    head = (f'{"x":>5} {"z":>5} {"mount":>6} {"ant_z":>5} | {"ahead":>5} {"behind":>6} {"sides":>5} | '
            f'{"lane":>5} {"2m":>5} {"self":>5} | {"A-0":>4} {"A-2":>5} {"SurM":>5} | {"score":>5}')
    print(title)
    print(head)
    print('-' * len(head))
    for r in rows:
        print(f'{r["x"]:5.2f} {r["z"]:5.2f} {r["mount"]:>6} {r["ant_z"]:5.2f} | {r["gap_ahead"]:5.2f} '
              f'{r["gap_behind"]:6.2f} {r["gap_sides"]:5.2f} | {100 * r["lane"]:4.0f}% {100 * r["band"]:4.0f}% '
              f'{100 * r["self_hit"]:4.0f}% | {r["a0"]:4.1f} {r["a2"]:5.1f} {r["surmark"]:5.1f} | {r["score"]:5.3f}')
    print()


def main():
    base, _, _ = config.prepare(os.path.join(PKG, 'config', 'asv.yaml'))
    rows = [evaluate(base, x, z, mount, az) for x in XS for z in ZS for mount in ('below', 'side')
            for az in ANTENNA_ZS]
    valid = sorted((r for r in rows if r['valid']), key=lambda r: r['score'])
    print(f'{len(rows)} placements evaluated, {len(valid)} satisfy the constraints')
    rejected = {}
    for r in rows:
        if not r['valid']:
            for reason in r['why'].split('; '):
                rejected[reason] = rejected.get(reason, 0) + 1
    print('rejected:', ', '.join(f'{k} ({v})' for k, v in rejected.items()), '\n')
    print('gaps: blind water from the hull outline (m); lane: water ahead seen (3 m); 2m: water within 2 m '
          'seen; self: beams on own boat; A-0/A-2/SurM: detection range with >= 2 beams (m)\n')
    show(valid[:12], 'BEST PLACEMENTS')
    for x in (0.0, 0.3, 0.6, 0.85):                  # best per lidar position, for comparison
        best = [r for r in valid if r['x'] == x][:1]
        if best:
            show(best, f'best with the lidar at x = {x}')

    out = os.path.join(PKG, 'config', 'lidar_study.csv')
    with open(out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(valid[0].keys()) if valid else ['x'], extrasaction='ignore')
        w.writeheader()
        w.writerows(rows)
    print(f'all results: {os.path.relpath(out, PKG)}')
    _heatmap(rows, os.path.join(PKG, 'config', 'lidar_study.png'))


def _heatmap(rows, path):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return
    best_az = min((r for r in rows if r['valid']), key=lambda r: r['score'])['ant_z']
    fig, axes = plt.subplots(1, 4, figsize=(19, 4.2), sharey=True)
    for ax, key, title, better_low in zip(
            axes, ('gap_ahead', 'lane', 'band', 'score'),
            ('blind gap ahead of the bow [m]', 'lane ahead seen [-]', 'water within 2 m seen [-]', 'score (lower = better)'),
            (True, False, False, True)):
        grid = [[float('nan')] * len(XS) for _ in ZS]
        for r in rows:
            if r['valid'] and r['ant_z'] == best_az:
                i, j = ZS.index(r['z']), XS.index(r['x'])
                v = r[key]
                cur = grid[i][j]
                if cur != cur or (v < cur if better_low else v > cur):   # nan check, best of both mounts
                    grid[i][j] = v
        im = ax.imshow(grid, origin='lower', aspect='auto', cmap='viridis_r' if better_low else 'viridis',
                       extent=[XS[0] - 0.025, XS[-1] + 0.025, -0.5, len(ZS) - 0.5])
        ax.set_yticks(range(len(ZS)), [f'{z:.2f}' for z in ZS])
        ax.set_xlabel('lidar x (forward) [m]')
        ax.set_title(title)
        fig.colorbar(im, ax=ax)
    axes[0].set_ylabel('lidar height z [m]')
    fig.suptitle(f'Lidar placement study, GNSS antennas at z = {best_az} m (white = violates a constraint)')
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    print(f'heat maps: {os.path.relpath(path, PKG)}')


if __name__ == '__main__':
    main()
