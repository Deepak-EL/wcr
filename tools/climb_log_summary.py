#!/usr/bin/env python3
"""Summarise a climb_test CSV: height, surface angle, adhesion and speed over time.
usage: python3 tools/climb_log_summary.py climb.csv [step_s]"""
import csv
import math
import sys

rows = list(csv.DictReader(open(sys.argv[1])))
step = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
last_t, last_p = None, None
print(f'{"t":>7s} {"phase":8s} {"z[m]":>7s} {"y[m]":>7s} {"surf[deg]":>9s} {"F[N]":>6s} {"v[m/s]":>7s}')
for r in rows:
    t = float(r['t'])
    p = [float(r[k]) for k in 'xyz']
    if last_t is None or t - last_t >= step or r is rows[-1]:
        v = math.dist(p, last_p) / (t - last_t) if last_t is not None and t > last_t else 0.0
        print(f'{t:7.1f} {r["phase"]:8s} {p[2]:7.3f} {p[1]:7.3f} {float(r["surface_deg"]):9.1f} '
              f'{float(r["adhesion_N"]):6.1f} {v:7.3f}')
        last_t, last_p = t, p
