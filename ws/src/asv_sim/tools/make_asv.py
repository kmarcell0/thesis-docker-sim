#!/usr/bin/env python3
"""Generate the ASV model. Run from ws/src/asv_sim:  python3 tools/make_asv.py --help"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from asvgen.cli import main  # noqa: E402

if __name__ == '__main__':
    sys.exit(main())
