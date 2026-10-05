"""Design checks and sensor coverage metrics.

Everything here is computed from the same numbers that go into the SDF, so
the report describes exactly the model that was generated. Results are
returned twice: as human-readable lines (report.txt) and as a nested dict
(report.json) that scripts can compare across design variants.

Course targets (RoboBoat 2026 handbook, section 3.2):
    A-0 buoy       0.5 ft above water, 20.3 cm wide   (Task 2 channel)
    A-2 buoy       1 ft above water,   36.8 cm wide   (Task 3)
    Sur-Mark buoy  39 in above water,  18 in wide     (Task 1 gates)
"""
import math

from . import geometry as geo
from .hydrostatics import G
from .sensors import imu_white_noise

LBS = 2.20462
FT, IN = 0.3048, 0.0254
BOX = (6 * FT, 3 * FT, 3 * FT)              # RoboBoat 2026: vehicle must fit a 6 x 3 x 3 ft box
TARGETS = {                                  # name: (height above water, width), metres
    'A-0 buoy': (0.5 * FT, 0.203),
    'A-2 buoy': (1.0 * FT, 0.368),
    'Sur-Mark buoy': (39 * IN, 18 * IN),
}
MIN_BEAMS = 2               # beams needed to call a lidar detection reliable
MIN_PIXELS = 20             # object width in pixels assumed necessary for classification
DISPARITY_NOISE_PX = 0.25   # assumed stereo matching accuracy, pixels


# ------------------------------------------------------------------ RoboBoat rules
def weight_score(lbs):
    """RoboBoat 2026 static judging, weight points."""
    if lbs > 140:
        return None                      # disqualified
    if lbs > 110:
        return -250 - 5 * (lbs - 110)
    if lbs > 70:
        return 2 * (110 - lbs)
    return 80 + (70 - lbs)


def roboboat(boat):
    xs, ys, zs = zip(*boat.extents)
    size = [max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)]
    lbs = boat.total.mass * LBS
    thrust_lbf = 2 * boat.cfg['thrusters']['max_thrust'] / G * LBS
    ws = weight_score(lbs)
    m = {'size_m': [round(v, 3) for v in size], 'fits_box': all(a <= b for a, b in zip(size, BOX)),
         'weight_lbs': round(lbs, 1), 'weight_score': None if ws is None else round(ws, 1),
         'thrust_lbf': round(thrust_lbf, 1), 'thrust_weight_score': round(100 * thrust_lbf / lbs, 1)}
    lines = ['RoboBoat 2026 checks',
             f'  size      {size[0]:.2f} x {size[1]:.2f} x {size[2]:.2f} m  '
             f'(limit {BOX[0]:.2f} x {BOX[1]:.2f} x {BOX[2]:.2f})'
             + ('' if m['fits_box'] else '   <-- EXCEEDS the 6 x 3 x 3 ft box'),
             f'  weight    {lbs:.1f} lbs -> weight score '
             + ('DISQUALIFIED (> 140 lbs)' if ws is None else f'{ws:.0f}'),
             f'  thrust    {thrust_lbf:.1f} lbf total -> thrust/weight score {m["thrust_weight_score"]:.0f}']
    return m, lines


# ------------------------------------------------------------------------ layout
OVERLAP_TOL = 0.002          # m; boxes may touch, but not interpenetrate more than this


def layout(boat):
    """Components, sensors and mounts must not overlap each other, and every
    component must sit on top of the deck, inside its footprint."""
    d = boat.cfg['deck']
    parts = [s for s in boat.solids if s.kind != 'structure']
    conflicts = []
    for i, a_ in enumerate(parts):
        for b_ in parts[i + 1:]:
            if a_.owner == b_.owner:
                continue                       # a sensor touches its own post and arm
            ov = [min(a_.hi[k], b_.hi[k]) - max(a_.lo[k], b_.lo[k]) for k in range(3)]
            if all(v > OVERLAP_TOL for v in ov):
                conflicts.append({'a': a_.label, 'b': b_.label, 'overlap_m': [round(v, 3) for v in ov]})
    placement = []
    for s in (s for s in parts if s.kind == 'component'):
        if s.lo[2] < boat.deck_top - OVERLAP_TOL:
            placement.append(f'{s.label} reaches {100 * (boat.deck_top - s.lo[2]):.1f} cm into the deck')
        if s.lo[0] < -d['length'] / 2 or s.hi[0] > d['length'] / 2 or \
                s.lo[1] < -d['width'] / 2 or s.hi[1] > d['width'] / 2:
            placement.append(f'{s.label} sticks out over the deck edge')
    m = {'conflicts': conflicts, 'placement': placement, 'ok': not conflicts and not placement}
    if m['ok']:
        lines = ['Layout     OK: no overlapping components, sensors or mounts']
    else:
        lines = ['Layout     <-- PROBLEMS:']
        lines += [f'  {c["a"]} overlaps {c["b"]} by ' + ' x '.join(f'{100 * v:.1f}' for v in c['overlap_m'])
                  + ' cm' for c in conflicts]
        lines += [f'  {p}' for p in placement]
    return m, lines


# --------------------------------------------------------------------- mass, hydro
def mass_and_hydro(boat):
    t, hs = boat.total, boat.hydro
    groups = {}
    for i in boat.mass_items + boat.pod_items:
        groups[i.group] = groups.get(i.group, 0.0) + i.mass
    m = {'total_kg': round(t.mass, 3), 'total_lbs': round(t.mass * LBS, 1),
         'cog': [round(c, 4) for c in t.cog],
         'groups_kg': {k: round(v, 3) for k, v in groups.items()},
         'items': [{'name': i.name, 'group': i.group, 'kg': i.mass, 'xyz': [round(c, 3) for c in i.xyz]}
                   for i in boat.mass_items + boat.pod_items]}
    h = {'draft_m': round(hs.draft, 4), 'waterline_z': round(hs.z_water, 4),
         'buoyancy_reserve': round(hs.reserve, 3), 'gm_roll_m': round(hs.gm_t, 3),
         'gm_pitch_m': round(hs.gm_l, 3), 'heel_deg': round(hs.heel_deg, 2), 'trim_deg': round(hs.trim_deg, 2),
         'roll_period_s': round(hs.roll_period, 2), 'pitch_period_s': round(hs.pitch_period, 2),
         'spawn_z': round(-hs.z_water + 0.02, 3)}
    r = boat.cfg['hull']['radius']
    n_pts = boat.cfg['hull']['points']
    pitch_ratio = 1 - 1 / n_pts ** 2
    h['sim_pitch_stiffness_ratio'] = round(pitch_ratio, 4)
    lines = [f'Mass       {t.mass:6.2f} kg ({t.mass * LBS:.1f} lbs): '
             + ', '.join(f'{k} {v:.1f}' for k, v in groups.items()),
             f'CoG        x {t.cog[0]:+.3f}  y {t.cog[1]:+.3f}  z {t.cog[2]:+.3f} m',
             f'Draft      {hs.draft * 100:5.1f} cm of {2 * r * 100:.0f} cm hull diameter '
             f'(buoyancy reserve {hs.reserve * 100:.0f} % before half-submerged)',
             f'Stability  GM roll {hs.gm_t:.2f} m, GM pitch {hs.gm_l:.2f} m'
             + ('' if hs.gm_t > 0.05 else '   <-- WARNING: low or negative, may capsize'),
             f'Attitude   heel {hs.heel_deg:+.2f} deg (+ = to port), trim {hs.trim_deg:+.2f} deg (+ = bow up)',
             f'Periods    roll {hs.roll_period:.2f} s, pitch {hs.pitch_period:.2f} s (no added mass: lower bounds)',
             f'Simulation VRX samples each hull at {n_pts} keel points -> pitch stiffness in Gazebo is '
             f'{100 * pitch_ratio:.1f} % of the value above (1 - 1/N^2)',
             f'Spawn at   z = {h["spawn_z"]:.3f} m (waterline at world z = 0)']
    return m, h, lines


# -------------------------------------------------------------------------- lidar
NEAR_FIELD_COLUMNS = 72      # azimuth samples for the self-hit tracing (every 5 deg)
WATER_GRID = 0.1             # m, raster of the water visibility map
WATER_MARGIN = 3.0           # m, how far around the boat the water is sampled
NEAR_BAND = 2.0              # m, coverage is reported for water within this distance of the hulls
FRONT_LANE_EXTRA = 0.5       # m, the 'front lane' is the boat's width + this on each side, out to WATER_MARGIN


def ray_box(o, d, lo, hi, t_max):
    """Slab test: distance along the unit ray o + t d at which it enters the
    axis-aligned box [lo, hi], or None if it misses within (0, t_max)."""
    t0, t1 = 1e-6, t_max
    for k in range(3):
        if abs(d[k]) < 1e-12:
            if o[k] < lo[k] or o[k] > hi[k]:
                return None
            continue
        a_, b_ = (lo[k] - o[k]) / d[k], (hi[k] - o[k]) / d[k]
        if a_ > b_:
            a_, b_ = b_, a_
        t0, t1 = max(t0, a_), min(t1, b_)
        if t0 > t1:
            return None
    return t0


def lidar_self_hits(part, boat):
    """Share of the lidar's beams that end on the own boat (all channels, every
    5 deg of azimuth, traced against the boat's solids): the part of each scan
    wasted on self-returns."""
    s, spec = part.cfg, part.spec
    o, R = s['xyz'], geo.rot(s['rpy'])
    lo, hi = spec['vfov']
    rmin, rmax = spec['range']
    n = spec['channels']
    boxes = [b for b in boat.solids if b.owner != part.name]
    z_w = boat.hydro.z_water
    hits = total = 0
    for j in range(NEAR_FIELD_COLUMNS):
        phi = -math.pi + 2 * math.pi * j / NEAR_FIELD_COLUMNS
        for k in range(n):
            d = geo.mat_vec(R, geo.ray(phi, lo + k * (hi - lo) / (n - 1)))
            t_end = min(rmax, (z_w - o[2]) / d[2]) if d[2] < -1e-9 else rmax
            ts = [t for t in (ray_box(o, d, bx.lo, bx.hi, t_end) for bx in boxes) if t is not None]
            total += 1
            if ts and min(ts) >= rmin:
                hits += 1
    return hits / total


def boat_outline(boat):
    """Half length and half width of the hulls' footprint (the boat outline)."""
    h = boat.cfg['hull']
    return h['length'] / 2, h['spacing'] / 2 + h['radius']


def water_visibility(part, boat):
    """Which water points around the boat the lidar can see (line of sight).

    The water plane around the boat is sampled on a WATER_GRID raster out to
    WATER_MARGIN beyond the hull outline. A point P is visible if
      * its distance is within the lidar's range,
      * its direction, expressed in the lidar's frame, lies inside the vertical
        field of view (the Mid-360 pattern fills the whole field over time),
      * the segment lidar -> P is not blocked by any of the boat's solids.
    Points under the boat are skipped. Returns the raster and gap metrics,
    all measured from the hull outline, so they compare across placements."""
    s, spec = part.cfg, part.spec
    o, R = s['xyz'], geo.rot(s['rpy'])
    lo, hi = spec['vfov']
    rmin, rmax = spec['range']
    boxes = [b for b in boat.solids if b.owner != part.name]
    z_w = boat.hydro.z_water
    hx, hy = boat_outline(boat)
    nx = int(round((2 * hx + 2 * WATER_MARGIN) / WATER_GRID)) + 1
    ny = int(round((2 * hy + 2 * WATER_MARGIN) / WATER_GRID)) + 1
    xs = [-hx - WATER_MARGIN + i * WATER_GRID for i in range(nx)]
    ys = [-hy - WATER_MARGIN + j * WATER_GRID for j in range(ny)]

    def visible(x, y):
        v = [x - o[0], y - o[1], z_w - o[2]]
        dist = geo.norm(v)
        if not rmin <= dist <= rmax:
            return False
        if not lo <= geo.elevation(geo.mat_t_vec(R, v)) <= hi:
            return False
        d = [c / dist for c in v]
        return all(ray_box(o, d, bx.lo, bx.hi, dist - 1e-3) is None for bx in boxes)

    grid = [[None if (abs(x) <= hx and abs(y) <= hy) else visible(x, y) for x in xs] for y in ys]

    def gap(cells):
        """Distance from the outline to the first visible cell along a line."""
        for dist, seen in cells:
            if seen:
                return dist
        return WATER_MARGIN                     # nothing visible within the margin

    corridor_y = [j for j, y in enumerate(ys) if abs(y) <= hy]
    corridor_x = [i for i, x in enumerate(xs) if abs(x) <= hx]
    ahead = [gap([(xs[i] - hx, grid[j][i]) for i in range(nx) if xs[i] > hx]) for j in corridor_y]
    behind = [gap([(-hx - xs[i], grid[j][i]) for i in reversed(range(nx)) if xs[i] < -hx]) for j in corridor_y]
    port = [gap([(ys[j] - hy, grid[j][i]) for j in range(ny) if ys[j] > hy]) for i in corridor_x]
    stbd = [gap([(-hy - ys[j], grid[j][i]) for j in reversed(range(ny)) if ys[j] < -hy]) for i in corridor_x]
    centre = min(corridor_y, key=lambda j: abs(ys[j]))
    band = [grid[j][i] for j, y in enumerate(ys) for i, x in enumerate(xs)
            if grid[j][i] is not None and max(abs(x) - hx, abs(y) - hy, 0) <= NEAR_BAND]
    lane = [grid[j][i] for j, y in enumerate(ys) for i, x in enumerate(xs)
            if x > hx and abs(y) <= hy + FRONT_LANE_EXTRA]
    return {
        'xs': xs, 'ys': ys, 'grid': grid,
        'gap_ahead_m': round(max(ahead), 2),
        'gap_ahead_centreline_m': round(ahead[corridor_y.index(centre)], 2),
        'gap_behind_m': round(max(behind), 2),
        'gap_sides_m': round(max(port + stbd), 2),
        'coverage_near_band': round(sum(1 for v in band if v) / len(band), 3),
        'coverage_front_lane': round(sum(1 for v in lane if v) / len(lane), 3),
    }


def lidar(part, boat):
    s, spec = part.cfg, part.spec
    xyz, rpy = s['xyz'], s['rpy']
    h_s = xyz[2] - boat.hydro.z_water
    lo, hi = spec['vfov']
    n = spec['channels']
    beams = [lo + k * (hi - lo) / (n - 1) for k in range(n)]
    # beams of the column that points straight ahead of the boat, in the boat frame
    fwd = geo.mat_t_vec(geo.rot(rpy), [1.0, 0.0, 0.0])
    az = math.atan2(fwd[1], fwd[0])
    elev = sorted(geo.elevation(geo.mat_vec(geo.rot(rpy), geo.ray(az, b))) for b in beams)
    rmin, rmax = spec['range']

    def hits(d, height):
        if d < rmin or d > rmax:
            return 0
        return sum(1 for e in elev if 0.0 <= h_s + d * math.tan(e) <= height)

    dists = [1, 2, 3, 5, 10, 20, 40]
    table = {name: {d: hits(d, th) for d in dists} for name, (th, _) in TARGETS.items()}
    reach = {}
    for name, (th, _) in TARGETS.items():
        ok = [d / 4 for d in range(1, int(rmax * 4) + 1) if hits(d / 4, th) >= MIN_BEAMS]
        reach[name] = max(ok) if ok else 0.0
    own = [label for owner, label, pt in boat.occluders
           if owner != part.name
           and rmin <= geo.norm(geo.to_child(xyz, rpy, pt)) <= rmax
           and lo <= geo.elevation(geo.to_child(xyz, rpy, pt)) <= hi]
    water_ring = h_s / math.tan(-elev[0]) if elev[0] < 0 else None
    vis = water_visibility(part, boat)
    near = {k: v for k, v in vis.items() if k not in ('xs', 'ys', 'grid')}
    near['self_hit_share'] = round(lidar_self_hits(part, boat), 3)
    m = {'height_above_water_m': round(h_s, 3), 'upside_down': geo.is_upside_down(rpy),
         'elevation_deg': [round(math.degrees(elev[0]), 1), round(math.degrees(elev[-1]), 1)],
         'beam_spacing_deg': round(math.degrees((hi - lo) / (n - 1)), 2),
         'points_per_scan': n * spec['h_samples'],
         'water_ring_m': None if water_ring is None else round(water_ring, 2),
         'beams_on_targets': table, f'max_distance_{MIN_BEAMS}_beams_m': reach, 'sees_own_boat': own,
         'near_field': near}
    lines = [f'Lidar "{part.name}"' + (' (upside down)' if m['upside_down'] else '')
             + f': {h_s:.2f} m above water, beams {m["elevation_deg"][0]:+.0f}..{m["elevation_deg"][1]:+.0f} deg '
             f'({m["beam_spacing_deg"]:.2f} deg apart), {m["points_per_scan"]} points per scan']
    if water_ring is not None:
        lines.append(f'  lowest beam meets the water {water_ring:.1f} m away (ignoring the boat)')
    lines.append(f'  blind water next to the boat (line of sight, from the hull outline): '
                 f'{near["gap_ahead_m"]:.2f} m ahead of the bow ({near["gap_ahead_centreline_m"]:.2f} m on the '
                 f'centreline), {near["gap_behind_m"]:.2f} m behind the stern, {near["gap_sides_m"]:.2f} m beside the hulls')
    lines.append(f'  sees {100 * near["coverage_front_lane"]:.0f} % of the water in the lane ahead of the boat '
                 f'({WATER_MARGIN:.0f} m), {100 * near["coverage_near_band"]:.0f} % of the water within {NEAR_BAND:.0f} m '
                 f'all around; '
                 f'{100 * near["self_hit_share"]:.0f} % of its beams hit the own boat')
    lines.append('  beams on a buoy straight ahead at ' + ''.join(f'{d:>5} m' for d in dists))
    for name, row in table.items():
        lines.append(f'    {name:<30}' + ''.join(f'{c:>7}' for c in row.values()))
    lines.append(f'  >= {MIN_BEAMS} beams up to: ' + ', '.join(f'{k} {v:.1f} m' for k, v in reach.items()))
    if own:
        lines.append(f'  sees its own boat ({len(own)} points): {", ".join(own[:6])}'
                     + (' ...' if len(own) > 6 else '') + ' -> crop a box around the boat')
    return m, lines


# ------------------------------------------------------------------------- cameras
def _camera_geometry(xyz, rpy, spec, z_water):
    w, h, hfov = spec['width'], spec['height'], spec['hfov']
    f = (w / 2) / math.tan(hfov / 2)                       # focal length in pixels
    vfov = 2 * math.atan((h / 2) / f)
    R = geo.rot(rpy)
    axis = geo.elevation(geo.mat_vec(R, [1.0, 0.0, 0.0]))
    bottom = geo.elevation(geo.mat_vec(R, geo.ray(0.0, -vfov / 2)))
    top = geo.elevation(geo.mat_vec(R, geo.ray(0.0, vfov / 2)))
    h_c = xyz[2] - z_water
    near_water = h_c / math.tan(-bottom) if bottom < 0 else None
    horizon_row = h / 2 + f * math.tan(axis)               # pinhole: rows grow downwards
    return dict(f=f, vfov=vfov, h_c=h_c, near_water=near_water, top=top,
                horizon_row=horizon_row if 0 <= horizon_row <= h else None)


def camera(part, boat):
    s, spec = part.cfg, part.spec
    stereo = part.type == 'stereo_camera'
    g = _camera_geometry(s['xyz'], s['rpy'], spec, boat.hydro.z_water)
    dists = [5, 10, 20, 40]
    pixels = {name: {d: round(g['f'] * w / d, 1) for d in dists} for name, (_, w) in TARGETS.items()}
    classify = {name: round(g['f'] * w / MIN_PIXELS, 1) for name, (_, w) in TARGETS.items()}
    own = []
    for owner, label, pt in boat.occluders:          # parts of the boat inside the image
        v = geo.to_child(s['xyz'], s['rpy'], pt)       # camera body frame, x = viewing direction
        if owner != part.name and v[0] > 0.05 and abs(math.atan2(v[1], v[0])) <= spec['hfov'] / 2 \
                and abs(math.atan2(v[2], v[0])) <= g['vfov'] / 2:
            own.append(label)
    m = {'stereo': stereo, 'resolution': [spec['width'], spec['height']],
         'fov_deg': [round(math.degrees(spec['hfov']), 1), round(math.degrees(g['vfov']), 1)],
         'focal_px': round(g['f'], 1), 'height_above_water_m': round(g['h_c'], 3),
         'water_visible_from_m': None if g['near_water'] is None else round(g['near_water'], 2),
         'horizon_row': None if g['horizon_row'] is None else round(g['horizon_row']),
         'target_width_px': pixels, f'max_distance_{MIN_PIXELS}_px_m': classify, 'sees_own_boat': own}
    lines = [f'{"Stereo camera" if stereo else "Camera"} "{part.name}": {spec["width"]}x{spec["height"]} '
             f'@ {spec["rate"]} Hz, {m["fov_deg"][0]:.0f} x {m["fov_deg"][1]:.0f} deg, '
             f'{g["h_c"]:.2f} m above water'
             + (f', baseline {spec["baseline"] * 100:.0f} cm' if stereo else '')]
    view = []
    if g['near_water'] is not None:
        view.append(f'sees the water from {g["near_water"]:.1f} m')
    view.append(f'horizon at image row {m["horizon_row"]} of {spec["height"]}' if m['horizon_row'] is not None
                else ('horizon above the image' if g['top'] < 0 else 'horizon below the image'))
    lines.append('  ' + ', '.join(view))
    lines.append('  buoy width in pixels at        ' + ''.join(f'{d:>6} m' for d in dists))
    for name, row in pixels.items():
        lines.append(f'    {name:<30}' + ''.join(f'{v:>8.0f}' for v in row.values()))
    lines.append(f'  >= {MIN_PIXELS} px wide up to: ' + ', '.join(f'{k} {v:.0f} m' for k, v in classify.items()))
    if own:
        lines.append(f'  parts of the boat in the image: {", ".join(own)}')
    if stereo:
        zs = [2, 5, 10, 20, 35]
        sig = {z: round(z * z * DISPARITY_NOISE_PX / (g['f'] * spec['baseline']), 3) for z in zs}
        z10 = 0.1 * g['f'] * spec['baseline'] / DISPARITY_NOISE_PX
        m['depth_error_m'] = sig
        m['depth_error_below_10pct_up_to_m'] = round(z10, 1)
        lines.append(f'  stereo depth error ({DISPARITY_NOISE_PX} px matching): '
                     + ', '.join(f'{z} m: {e:.2f} m' for z, e in sig.items()))
        lines.append(f'  depth error stays below 10 % of the distance up to {z10:.0f} m')
    return m, lines


# ----------------------------------------------------------------------- IMU, GNSS
def imu(part, boat):
    g, a = imu_white_noise(part.spec)
    arm = geo.sub(part.cfg['xyz'], boat.total.cog)
    r = geo.norm(arm)
    yaw_rate = 0.5                                   # rad/s, a brisk turn
    m = {'rate_hz': part.spec['rate'], 'gyro_noise_deg_s': round(math.degrees(g), 4),
         'accel_noise_mg': round(a / G * 1000, 3), 'lever_arm_from_cog_m': [round(c, 3) for c in arm],
         'centripetal_at_0.5rad_s_m_s2': round(yaw_rate ** 2 * r, 4)}
    lines = [f'IMU "{part.name}" @ {part.spec["rate"]} Hz: white noise per sample '
             f'{m["gyro_noise_deg_s"]:.3f} deg/s, {m["accel_noise_mg"]:.2f} mg; '
             f'{r:.2f} m from the CoG ({m["centripetal_at_0.5rad_s_m_s2"]:.3f} m/s^2 extra in a 0.5 rad/s turn)']
    return m, lines


def gnss(part, boat):
    s, spec = part.cfg, part.spec
    if part.type == 'gnss':
        m = {'lever_arm_m': [round(c, 3) for c in s['xyz']]}
        return m, [f'GNSS "{part.name}": antenna at {fmt_vec(s["xyz"])} m (lever arm from base_link)']
    p1, p2 = s['xyz'], s['xyz_secondary']
    base = geo.sub(p2, p1)
    length = geo.norm(base)
    offset = math.degrees(math.atan2(base[1], base[0]))
    acc = spec['heading_accuracy_1m'] / length
    naive = math.degrees(math.sqrt(2) * spec['horizontal_noise'] / length)
    m = {'baseline_m': round(length, 3), 'baseline_heading_offset_deg': round(offset, 2),
         'heading_accuracy_deg': round(acc, 3), 'two_independent_fixes_deg': round(naive, 3),
         'primary_lever_arm_m': [round(c, 3) for c in p1]}
    lines = [f'Dual GNSS "{part.name}": baseline {length:.2f} m at {offset:+.1f} deg to the bow -> '
             f'heading accuracy ~{acc:.2f} deg (datasheet {spec["heading_accuracy_1m"]} deg at 1 m, '
             'scaled with 1/baseline)',
             f'  two independently noisy fixes would give {naive:.2f} deg -> emulate the receiver heading '
             'from ground truth + noise',
             f'  primary antenna (position) at {fmt_vec(p1)} m from base_link']
    return m, lines


def fmt_vec(v):
    return '[' + ', '.join(f'{c:+.2f}' for c in v) + ']'


# ------------------------------------------------------------------------- report
ANALYSES = {'lidar': lidar, 'camera': camera, 'stereo_camera': camera, 'imu': imu,
            'gnss': gnss, 'gnss_dual': gnss}


def analyze(boat, warnings=()):
    """Returns (metrics dict, report text)."""
    m_mass, m_hydro, l_mass = mass_and_hydro(boat)
    m_rb, l_rb = roboboat(boat)
    m_lay, l_lay = layout(boat)
    metrics = {'name': boat.cfg['name'], 'fingerprint': boat.fingerprint,
               'mass': m_mass, 'hydrostatics': m_hydro, 'roboboat': m_rb, 'layout': m_lay, 'sensors': {}}
    lines = [f'=== {boat.cfg["name"]} design report (fingerprint {boat.fingerprint}) ===', '']
    if warnings:
        lines += ['Config warnings:'] + [f'  {w}' for w in warnings] + ['']
    lines += l_lay + [''] + l_mass + [''] + l_rb + ['']
    for part in boat.parts:
        m, l = ANALYSES[part.type](part, boat)
        metrics['sensors'][part.name] = dict(m, type=part.type, spec=part.spec_name)
        lines += l + ['']
    return metrics, '\n'.join(lines)
