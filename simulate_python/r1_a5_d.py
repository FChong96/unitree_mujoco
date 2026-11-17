#!/usr/bin/env python3
"""Repository-root entry point for the R1-A5-D simulator."""
from pathlib import Path
import runpy
import sys

if __name__ == '__main__':
    support = Path(__file__).resolve().parent / 'r1_a5_d'
    sys.path.insert(0, str(support))
    runpy.run_path(str(support / 'run.py'), run_name='__main__')
