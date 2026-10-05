"""Command line entry point: python3 tools/make_asv.py [options]"""
import argparse
import os
import sys

from . import __version__
from . import analysis, boat as boat_mod, config, outputs, plot

PKG = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))


def parse_args(argv):
    ap = argparse.ArgumentParser(
        prog='make_asv.py',
        description='Generate the ASV Gazebo model, bridge/frames configs and a design report '
                    'from a YAML boat description.',
        epilog='examples:\n'
               '  make_asv.py                                     # config/asv.yaml -> models/asv\n'
               '  make_asv.py --check                             # report only, write nothing\n'
               '  make_asv.py --set battery.xyz=[0.1,0,0.225] --check\n'
               '  make_asv.py --name asv_low_lidar --set sensors.lidar.xyz=[0.72,0,0.45] --plot',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('config', nargs='?', default=os.path.join(PKG, 'config', 'asv.yaml'),
                    help='boat description (default: config/asv.yaml)')
    ap.add_argument('--set', action='append', default=[], metavar='KEY=VALUE',
                    help='override a value, e.g. sensors.lidar.xyz=[0.6,0,0.5] (repeatable)')
    ap.add_argument('--name', help='model name for this variant (files: models/<name>, config/<name>_*)')
    ap.add_argument('--check', action='store_true', help='validate and print the report, write nothing')
    ap.add_argument('--plot', action='store_true', help='also draw config/<name>_layout.png (needs matplotlib)')
    ap.add_argument('--quiet', action='store_true', help='do not print the report')
    ap.add_argument('--version', action='version', version=f'asvgen {__version__}')
    return ap.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    overrides = list(args.set) + ([f'name={args.name}'] if args.name else [])
    try:
        cfg, warnings, fingerprint = config.prepare(args.config, overrides)
    except config.ConfigError as e:
        print(f'ERROR: {e}', file=sys.stderr)
        return 2
    boat = boat_mod.build(cfg, fingerprint)
    metrics, report = analysis.analyze(boat, warnings)
    if not args.quiet:
        print(report)
    if args.check:
        print('(--check: nothing written)')
        return 0
    written = outputs.write_all(boat, metrics, report, PKG)
    if args.plot:
        png = plot.plot_layout(boat, os.path.join(PKG, 'config', f'{cfg["name"]}_layout.png'))
        if png:
            written.append(os.path.relpath(png, PKG))
    print('Wrote ' + ', '.join(written))
    return 0
