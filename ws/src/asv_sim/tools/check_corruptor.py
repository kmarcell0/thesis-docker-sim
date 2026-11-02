#!/usr/bin/env python3
"""Corruptor recording check (M2.6d): do the errors on /asv/sensors/... have the
size that asv.yaml x the noise profile says?

For every sensor the known parts are taken away; what is left must be white
noise with mean 0 and the configured sigma (design.md section 4):

    IMU      e = w~ - w - b^         gyro: corrupted - clean - true bias
                                     (accel the same); only 4 ms steps, where
                                     sigma = eta / sqrt(0.004 s)
    GNSS     e = ENU(fix~) - beta_G  per ENU axis and receiver mode; ENU in the
                                     tangent plane at the clean fix of the same
                                     stamp (the corruptor adds the error there),
                                     beta_G = /asv/ground_truth/gnss_error;
                                     sigma = white_h (E, N), white_v (U)
    velocity e = v~ - v_ant          v_ant = central difference of the antenna
                                     position p + R(q) l from ground-truth
                                     odometry at t -+ 20 ms; sigma = sigma_v
    heading  e = wrap(a~ - (pi/2 - psi))   psi from ground-truth odometry;
                                     sigma = sigma_alpha

The expected sigmas are computed here from asv.yaml and the profile (a Python
twin of imu_error_params() and gnss_error_params() in asv_sim), and the truth
from the clean and ground-truth topics: nothing is taken from the corruptor's
own covariance fields (independent truth).

Pass rule per residual, n samples, sample mean m and sample std s:
    |m| <= 3 sigma / sqrt(n)           (standard error of the mean)
    |s / sigma - 1| <= 3 / sqrt(2 (n - 1))   (standard error of the std)
Count checks: one corrupted IMU sample and one bias per clean sample; one
status per clean fix; fix, gnss_error, velocity and heading exactly on the
epochs that are not an outage; the observed mode switches follow the
profile's schedule (same order, same spacing within one epoch).

Usage (from the host; the bag was recorded with --use-sim-time):
    docker compose exec -T sim bash -ic \\
        "cd /ws/src/asv_sim && python3 tools/check_corruptor.py /ws/data/bags/m26d_check \\
         --profile mode_check"
Exit code 0 if every check passes, 1 otherwise.
"""
import argparse
import math
import os
import sys

import yaml

import check_clean_gnss as cg
import geodesy as gd

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

TOPIC_IMU_CLEAN = '/asv/imu/data'
TOPIC_IMU = '/asv/sensors/imu/data'
TOPIC_IMU_BIAS = '/asv/ground_truth/imu_bias'
TOPIC_FIX_CLEAN = '/asv/gnss/fix'
TOPIC_FIX = '/asv/sensors/gnss/fix'
TOPIC_GNSS_ERROR = '/asv/ground_truth/gnss_error'
TOPIC_STATUS = '/asv/ground_truth/gnss_status'
TOPIC_VELOCITY = '/asv/sensors/gnss/velocity'
TOPIC_HEADING = '/asv/sensors/gnss/heading'
TOPIC_ODOM = '/asv/ground_truth/odom'
TOPICS = (TOPIC_IMU_CLEAN, TOPIC_IMU, TOPIC_IMU_BIAS, TOPIC_FIX_CLEAN, TOPIC_FIX,
          TOPIC_GNSS_ERROR, TOPIC_STATUS, TOPIC_VELOCITY, TOPIC_HEADING, TOPIC_ODOM)

# asv_msgs/GnssStatus MODE_* (= asv_core::GnssMode)
MODE_NAMES = {0: 'outage', 1: 'standalone', 2: 'rtk_float', 3: 'rtk_fixed'}

IMU_STEP = 0.004        # s, the IMU's nominal step (200 Hz [REPO asv.yaml])
STEP_TOL = 0.0005       # s, a step counts as 4 ms within this
EPOCH_TOL = 0.1 + 1e-6  # s, one GNSS epoch (10 Hz [REPO])
HALF_DIFF = 0.020       # s, half the central-difference span (one odometry step)


# ---- expected values (asv.yaml x profile) ----------------------------------

def _one_sensor(asv, types):
    found = [s for s in asv['sensors'] if s['type'] in types]
    if len(found) != 1:
        raise ValueError(f'asv.yaml must have exactly one sensor of type {types}, '
                         f'found {len(found)}')
    return found[0]


def expected_sigmas(asv, profile):
    """The configured noise sizes, the same rules as asv_sim's
    imu_error_params() and gnss_error_params():
      gyro_density, accel_density   eta (rad/s/sqrt(Hz), m/s^2/sqrt(Hz))
      modes[name] = (h, v)          GNSS white sigma per axis (m)
      velocity                      sigma_v per ENU axis (m/s)
      heading                       sigma_alpha (rad), None without gnss_dual
      schedule                      [(from_s, mode name)]"""
    imu_spec = asv['sensor_specs'][_one_sensor(asv, ('imu',))['spec']]
    imu = profile['imu']
    gnss = _one_sensor(asv, ('gnss', 'gnss_dual'))
    spec = asv['sensor_specs'][gnss['spec']]
    g = profile['gnss']
    table = spec['position_modes']
    heading = None
    if gnss['type'] == 'gnss_dual':
        # D12 a: the datasheet accuracy is for a 1 m baseline, scales with 1/B
        baseline = math.dist(gnss['xyz_secondary'], gnss['xyz'])
        heading = math.radians(spec['heading_accuracy_1m']) / baseline * g['heading']
    return {
        'gyro_density': imu_spec['gyro_noise_density'] * imu['gyro']['white'],
        'accel_density': imu_spec['accel_noise_density'] * imu['accel']['white'],
        'modes': {m: (table[m]['white_h'] * g['white'], table[m]['white_v'] * g['white'])
                  for m in ('rtk_fixed', 'rtk_float', 'standalone')},
        'velocity': spec['velocity_noise'] * g['velocity'],
        'heading': heading,
        'schedule': [(float(e['from']), e['mode']) for e in g['schedule']],
    }


# ---- statistics --------------------------------------------------------------

def residual_stats(values, sigma):
    """n, mean, sample std, and whether both agree with N(0, sigma^2) within
    three standard errors."""
    n = len(values)
    if n < 2:
        return {'n': n, 'mean': math.nan, 'std': math.nan, 'mean_ok': False, 'std_ok': False}
    mean = sum(values) / n
    std = math.sqrt(sum((v - mean) ** 2 for v in values) / (n - 1))
    return {
        'n': n, 'mean': mean, 'std': std,
        'mean_ok': abs(mean) <= 3.0 * sigma / math.sqrt(n),
        'std_ok': abs(std / sigma - 1.0) <= 3.0 / math.sqrt(2.0 * (n - 1)),
    }


def wrap(a):
    """Angle into (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


# ---- residuals ---------------------------------------------------------------

def ns(msg):
    """header.stamp in integer nanoseconds (exact keys for matching)."""
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def imu_residuals(clean, corrupted, bias, step=IMU_STEP, tol=STEP_TOL):
    """{'gyro': [ex], [ey], [ez]], 'accel': [...]} from dicts {stamp_ns: Imu},
    only for samples whose step from the previous clean sample is 4 ms."""
    out = {'gyro': ([], [], []), 'accel': ([], [], [])}
    stamps = sorted(clean)
    for prev, t in zip(stamps, stamps[1:]):
        if abs((t - prev) * 1e-9 - step) > tol or t not in corrupted or t not in bias:
            continue
        c, m, b = clean[t], corrupted[t], bias[t]
        for key, field in (('gyro', 'angular_velocity'), ('accel', 'linear_acceleration')):
            for j, ax in enumerate('xyz'):
                out[key][j].append(getattr(getattr(m, field), ax) - getattr(getattr(c, field), ax)
                                   - getattr(getattr(b, field), ax))
    return out


def gnss_residuals(clean, fixes, errors, modes):
    """{mode name: ([e], [n], [u])}: corrupted fix in the tangent plane at the
    clean fix, minus the true slow error; dicts {stamp_ns: msg}, modes
    {stamp_ns: mode name}."""
    out = {}
    for t, fix in fixes.items():
        if t not in clean or t not in errors or t not in modes:
            continue
        c = clean[t]
        here = gd.LocalTangentPlane(c.latitude, c.longitude, c.altitude)
        m = here.to_enu(fix.latitude, fix.longitude, fix.altitude)
        b = errors[t].vector
        res = out.setdefault(modes[t], ([], [], []))
        for j, bj in enumerate((b.x, b.y, b.z)):
            res[j].append(m[j] - bj)
    return out


def antenna_position(truth, t, arm):
    """p + R(q) l at time t (s) from truth = sorted [(t, p, q)]; None outside."""
    pose = cg.pose_at(truth, t)
    if pose is None:
        return None
    p, q = pose
    return [pi + ri for pi, ri in zip(p, cg.rotate(q, arm))]


def velocity_residuals(truth, velocities, arm, half=HALF_DIFF):
    """([e], [n], [u]): measured minus the central difference of the true
    antenna position at t -+ half; velocities = {stamp_ns: msg}."""
    out = ([], [], [])
    for t_ns, msg in velocities.items():
        t = t_ns * 1e-9
        a, b = antenna_position(truth, t - half, arm), antenna_position(truth, t + half, arm)
        if a is None or b is None:
            continue
        v = msg.twist.twist.linear
        for j, vj in enumerate((v.x, v.y, v.z)):
            out[j].append(vj - (b[j] - a[j]) / (2 * half))
    return out


def yaw_of(q):
    """Yaw of a unit quaternion (w, x, y, z)."""
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def heading_residuals(truth, headings):
    """[wrap(alpha - (pi/2 - psi))]; headings = {stamp_ns: GnssHeading}."""
    out = []
    for t_ns, msg in headings.items():
        pose = cg.pose_at(truth, t_ns * 1e-9)
        if pose is None:
            continue
        out.append(wrap(msg.azimuth - (math.pi / 2 - yaw_of(pose[1]))))
    return out


# ---- counts and schedule -----------------------------------------------------

def missing_and_extra(expected, got, lo, hi):
    """Stamps in [lo, hi] that are expected but missing, and got but not expected."""
    exp = {t for t in expected if lo <= t <= hi}
    have = {t for t in got if lo <= t <= hi}
    return sorted(exp - have), sorted(have - exp)


def mode_switches(modes):
    """[(t_s, mode)]: the first epoch and every epoch where the mode changes;
    modes = {stamp_ns: mode name}."""
    out = []
    for t in sorted(modes):
        if not out or out[-1][1] != modes[t]:
            out.append((t * 1e-9, modes[t]))
    return out


def schedule_matches(switches, schedule, tol=EPOCH_TOL):
    """Do the observed switches follow the schedule? The recording may start
    late, so the first observed mode may be any schedule entry; from there the
    modes must come in schedule order and the time between switches must equal
    the difference of the 'from' values within one epoch. Returns
    (ok, t0) with t0 = the implied time of the first clean fix (s), or None."""
    for start in range(len(schedule)):
        if schedule[start][1] != switches[0][1]:
            continue
        rest = schedule[start + 1:start + len(switches)]
        if len(rest) != len(switches) - 1:
            continue
        if any(s[1] != m for (_, m), s in zip(switches[1:], rest)):
            continue
        if len(switches) == 1:
            return True, None
        t0 = switches[1][0] - rest[0][0]
        if all(abs(t - (t0 + s[0])) <= tol for (t, _), s in zip(switches[1:], rest)):
            return True, t0
    return False, None


# ---- report ------------------------------------------------------------------

def _row(name, stats, sigma, unit):
    ok = stats['mean_ok'] and stats['std_ok']
    ratio = stats['std'] / sigma if sigma else math.nan
    print(f'  {name:<22} n {stats["n"]:6d}  mean {stats["mean"]:+.3e}  std {stats["std"]:.4e}'
          f'  expected {sigma:.4e} {unit}  ({ratio:6.3f})  {"PASS" if ok else "FAIL"}')
    return ok


def _count(name, missing, extra):
    ok = not missing and not extra
    print(f'  {name:<44} missing {len(missing):4d}, extra {len(extra):4d}  '
          f'{"PASS" if ok else "FAIL"}')
    return ok


def by_stamp(data, topic):
    return {ns(m): m for _, m in data.get(topic, [])}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag')
    ap.add_argument('--profile', default='nominal', help='config/noise_profiles/<name>.yaml')
    ap.add_argument('--asv', default=os.path.join(PKG, 'config', 'asv.yaml'))
    ap.add_argument('--frames', default=os.path.join(PKG, 'config', 'asv_frames.yaml'))
    args = ap.parse_args(argv)

    with open(args.asv) as f:
        asv = yaml.safe_load(f)
    with open(os.path.join(PKG, 'config', 'noise_profiles', args.profile + '.yaml')) as f:
        exp = expected_sigmas(asv, yaml.safe_load(f))

    import analyze_run as ar        # needs rosbag2_py: only inside the container
    data = ar.read_bag(args.bag, TOPICS)
    for topic in TOPICS:
        if not data.get(topic):
            print(f'{topic}: no messages')
            return 1
    d = {topic: by_stamp(data, topic) for topic in TOPICS}
    ok = True

    print(f'profile {args.profile}; bag {args.bag}')
    print('counts (inside the window where both topics were recorded):')
    clean_imu, imu, bias = d[TOPIC_IMU_CLEAN], d[TOPIC_IMU], d[TOPIC_IMU_BIAS]
    lo, hi = max(min(clean_imu), min(imu)), min(max(clean_imu), max(imu))
    ok &= _count('IMU: corrupted on every clean sample', *missing_and_extra(clean_imu, imu, lo, hi))
    ok &= _count('IMU: true bias on every corrupted sample', *missing_and_extra(imu, bias, lo, hi))

    status = d[TOPIC_STATUS]
    modes = {t: MODE_NAMES[m.position_mode] for t, m in status.items()}
    clean_fix = d[TOPIC_FIX_CLEAN]
    lo, hi = max(min(clean_fix), min(status)), min(max(clean_fix), max(status))
    ok &= _count('GNSS: status on every clean fix', *missing_and_extra(clean_fix, status, lo, hi))
    up = {t for t, m in modes.items() if m != 'outage'}
    for name, topic in (('fix', TOPIC_FIX), ('gnss_error', TOPIC_GNSS_ERROR),
                        ('velocity', TOPIC_VELOCITY), ('heading', TOPIC_HEADING)):
        got = d[topic]
        lo, hi = max(min(status), min(got)), min(max(status), max(got))
        ok &= _count(f'GNSS: {name} on every epoch outside an outage',
                     *missing_and_extra(up, got, lo, hi))
    flag_wrong = [t for t, m in status.items()
                  if m.heading_available != (t in d[TOPIC_HEADING])
                  and min(d[TOPIC_HEADING]) <= t <= max(d[TOPIC_HEADING])]
    ok &= _count('GNSS: heading_available = heading published', flag_wrong, [])

    switches = mode_switches(modes)
    sched_ok, t0 = schedule_matches(switches, exp['schedule'])
    ok &= sched_ok
    print('GNSS mode switches (s): ' + ', '.join(f'{m} {t:.1f}' for t, m in switches))
    print(f'  schedule {exp["schedule"]}: {"PASS" if sched_ok else "FAIL"}'
          + (f' (first clean fix at {t0:.1f} s)' if t0 is not None else ''))

    print('residuals (mean 0 and the expected std, within 3 standard errors):')
    res = imu_residuals(clean_imu, imu, bias)
    for key, unit in (('gyro', 'rad/s'), ('accel', 'm/s^2')):
        sigma = exp[key + '_density'] / math.sqrt(IMU_STEP)
        for j, ax in enumerate('xyz'):
            ok &= _row(f'IMU {key} {ax}', residual_stats(res[key][j], sigma), sigma, unit)

    res = gnss_residuals(clean_fix, d[TOPIC_FIX], d[TOPIC_GNSS_ERROR], modes)
    for mode in ('rtk_fixed', 'rtk_float', 'standalone'):
        if mode not in res:
            print(f'  GNSS {mode}: no fixes in this recording')
            continue
        h, v = exp['modes'][mode]
        for j, (ax, sigma) in enumerate(zip('ENU', (h, h, v))):
            ok &= _row(f'GNSS {mode} {ax}', residual_stats(res[mode][j], sigma), sigma, 'm')

    odom = data[TOPIC_ODOM]
    truth = sorted((cg.stamp(m), (m.pose.pose.position.x, m.pose.pose.position.y,
                                  m.pose.pose.position.z),
                    (m.pose.pose.orientation.w, m.pose.pose.orientation.x,
                     m.pose.pose.orientation.y, m.pose.pose.orientation.z))
                   for _, m in odom)
    arm = cg.lever_arms(args.frames)[next(iter(clean_fix.values())).header.frame_id]
    res = velocity_residuals(truth, d[TOPIC_VELOCITY], arm)
    for j, ax in enumerate('ENU'):
        ok &= _row(f'velocity {ax}', residual_stats(res[j], exp['velocity']),
                   exp['velocity'], 'm/s')
    if exp['heading'] is not None:
        ok &= _row('heading', residual_stats(heading_residuals(truth, d[TOPIC_HEADING]),
                                             exp['heading']), exp['heading'], 'rad')

    print('ALL PASS' if ok else 'SOME CHECKS FAILED')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
