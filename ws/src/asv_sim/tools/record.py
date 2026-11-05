#!/usr/bin/env python3
"""Record a bag with one of the recording profiles (M2.9, design.md section 8).

The profiles (topic lists) are in config/recording_profiles.yaml; a profile
may extend another one (nav < nav_lidar < perception). This script builds

    ros2 bag record --use-sim-time -o <out_dir>/<name> --topics <topics...>

runs it and stops it cleanly with SIGINT (Ctrl-C), so the bag is readable.

Usage (inside the container, the simulation already running):
    python3 tools/record.py --profile nav --name run1 [--duration 60]
    python3 tools/record.py --profile nav --name run1 --print   # only print
--duration is in wall-clock seconds (headless Gazebo runs at a real-time
factor of about 0.85, so the bag holds less simulated time); without it the
recording runs until Ctrl-C.
"""
import argparse
import os
import signal
import subprocess
import sys
import time

import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROFILES_FILE = os.path.join(PKG, 'config', 'recording_profiles.yaml')
OUT_DIR = '/ws/data/bags'


def load_profiles(path=PROFILES_FILE):
    with open(path) as f:
        return yaml.safe_load(f)['profiles']


def profile_topics(profiles, name):
    """{topic: rate in Hz or None} of a profile, with its `extends` chain
    resolved (the base profile's topics first)."""
    if name not in profiles:
        raise KeyError(f'unknown profile {name!r}; known: {", ".join(profiles)}')
    seen = []
    topics = {}
    while name is not None:
        if name in seen:
            raise ValueError(f'profile cycle: {" -> ".join(seen + [name])}')
        seen.append(name)
        name = profiles[name].get('extends')
    for p in reversed(seen):
        for topic, rate in profiles[p]['topics'].items():
            if topic in topics:
                raise ValueError(f'{topic} listed twice (profile {p})')
            topics[topic] = rate
    return topics


def record_command(topics, out):
    return ['ros2', 'bag', 'record', '--use-sim-time', '-o', out, '--topics', *topics]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--profile', required=True)
    ap.add_argument('--name', required=True, help='bag folder name')
    ap.add_argument('--out-dir', default=OUT_DIR)
    ap.add_argument('--duration', type=float, help='wall-clock seconds')
    ap.add_argument('--print', action='store_true', help='print the command only')
    args = ap.parse_args(argv)

    topics = profile_topics(load_profiles(), args.profile)
    cmd = record_command(topics, os.path.join(args.out_dir, args.name))
    if args.print:
        print(' '.join(cmd))
        return 0

    proc = subprocess.Popen(cmd)
    try:
        if args.duration is None:
            return proc.wait()
        time.sleep(args.duration)
    except KeyboardInterrupt:
        pass        # Ctrl-C also reached the recorder (same process group)
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
    return proc.wait()


if __name__ == '__main__':
    sys.exit(main())
