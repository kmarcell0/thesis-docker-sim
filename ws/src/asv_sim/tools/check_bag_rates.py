#!/usr/bin/env python3
"""Recording check (M2.9): did a bag keep every message of its profile?

For each topic of the profile with a rate f (config/recording_profiles.yaml):
    expected n = f * D,    D = bag duration (simulation time, --use-sim-time)
    PASS if |count - n| <= 1 % of n
Counts and D come from the bag's metadata.yaml (the numbers `ros2 bag info`
prints). Topics without a rate are only listed. Also printed: bag size and
MB per second of simulation time (design.md section 8).

Usage (inside the container, or on the host: needs only PyYAML):
    python3 tools/check_bag_rates.py /ws/data/bags/<bag> --profile nav
Exit code 0 if every check passes, 1 otherwise.
"""
import argparse
import os
import sys

import yaml

import record

TOLERANCE = 0.01    # relative, the M2.9 done-when


def read_metadata(bag):
    """(duration in s, {topic: message count}) from <bag>/metadata.yaml."""
    with open(os.path.join(bag, 'metadata.yaml')) as f:
        info = yaml.safe_load(f)['rosbag2_bagfile_information']
    duration = info['duration']['nanoseconds'] * 1e-9
    counts = {t['topic_metadata']['name']: t['message_count']
              for t in info['topics_with_message_count']}
    return duration, counts


def bag_size(bag):
    """Bytes of all files in the bag folder."""
    return sum(os.path.getsize(os.path.join(bag, f)) for f in os.listdir(bag))


def check_counts(topics, duration, counts, tol=TOLERANCE):
    """[(topic, rate, count, expected or None, ok or None)], one per topic.
    A topic missing from the bag has count 0."""
    rows = []
    for topic, rate in topics.items():
        count = counts.get(topic, 0)
        if rate is None:
            rows.append((topic, None, count, None, None))
            continue
        expected = rate * duration
        rows.append((topic, rate, count, expected, abs(count - expected) <= tol * expected))
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag')
    ap.add_argument('--profile', required=True)
    args = ap.parse_args(argv)

    topics = record.profile_topics(record.load_profiles(), args.profile)
    duration, counts = read_metadata(args.bag)
    rows = check_counts(topics, duration, counts)
    size = bag_size(args.bag)

    print(f'bag {args.bag}, profile {args.profile}, duration {duration:.2f} s (sim)')
    print(f'{"topic":40s} {"rate":>6s} {"count":>8s} {"expected":>9s} {"diff %":>7s}')
    ok_all = True
    for topic, rate, count, expected, ok in rows:
        if rate is None:
            print(f'{topic:40s} {"-":>6s} {count:8d} {"-":>9s} {"-":>7s}  (not checked)')
            continue
        diff = 100.0 * (count - expected) / expected if expected else float('nan')
        print(f'{topic:40s} {rate:6g} {count:8d} {expected:9.1f} {diff:+7.2f}  '
              f'{"PASS" if ok else "FAIL"}')
        ok_all &= ok
    extra = sorted(set(counts) - set(topics))
    if extra:
        print('topics not in the profile:', ', '.join(extra))
    print(f'size {size / 1e6:.1f} MB, {size / 1e6 / duration:.3f} MB per sim s, '
          f'{size / 1e6 / duration * 60:.1f} MB per sim min')
    print('ALL PASS' if ok_all else 'FAILED')
    return 0 if ok_all else 1


if __name__ == '__main__':
    sys.exit(main())
