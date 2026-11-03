#!/usr/bin/env python3
"""Check every patrol route against the threat zones, and draw them on a map.

    python3 scripts/check_routes.py                 # table for every route
    python3 scripts/check_routes.py --plot routes.png

No ROS needed - run it after editing patrol_routes.yaml to make sure each zone
demo really stays in its zone and nothing lands in the river.
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from route_loader import classify, load_config, route_names, world_waypoints  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CFG = os.path.join(HERE, '..', 'config', 'patrol_routes.yaml')

# World geometry from gen_yellowstone.py
X_MIN, X_MAX, Y_MIN, Y_MAX = -75.0, 75.0, -40.0, 235.0
RIVER_S, RIVER_N = 106.0, 126.0
BRIDGE_X, BRIDGE_HALF = 20.0, 6.5
EXPECTED = {'green_zone': 'GREEN', 'yellow_zone': 'YELLOW',
            'red_zone': 'RED', 'emergency_zone': 'EMERGENCY'}


def warnings_for(x, y):
    w = []
    if not (X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX):
        w.append('outside playable area')
    if RIVER_S <= y <= RIVER_N and abs(x - BRIDGE_X) > BRIDGE_HALF:
        w.append('in the river')
    return w


def report(cfg):
    ok = True
    for name in route_names(cfg):
        pts = world_waypoints(cfg, name)
        length = sum(math.dist(a[:2], b[:2]) for a, b in zip(pts, pts[1:]))
        print(f'\n{name}  ({len(pts)} pts, {length:.0f} m one way)')
        for i, (x, y, _z) in enumerate(pts):
            zone, bear, d = classify(cfg, x, y)
            flags = warnings_for(x, y)
            if name in EXPECTED and zone != EXPECTED[name]:
                flags.append(f'expected {EXPECTED[name]}')
            ok &= not flags
            print(f'  {i + 1:2d}  ({x:7.1f}, {y:6.1f})  {zone:9s} {d:5.1f} m from {bear:13s}'
                  + ('  !! ' + ', '.join(flags) if flags else ''))
    print('\nAll routes OK' if ok else '\nSome waypoints need attention (!!)')
    return ok


def plot(cfg, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle

    names = route_names(cfg)
    fig, ax = plt.subplots(figsize=(9, 14))
    ax.add_patch(Rectangle((X_MIN, Y_MIN), X_MAX - X_MIN, Y_MAX - Y_MIN,
                           fill=False, ls='--', color='0.5'))
    ax.add_patch(Rectangle((X_MIN, RIVER_S), X_MAX - X_MIN, RIVER_N - RIVER_S,
                           color='#4a7fa0', alpha=0.45, label='river'))
    ax.add_patch(Rectangle((BRIDGE_X - BRIDGE_HALF, RIVER_S - 2), 2 * BRIDGE_HALF,
                           RIVER_N - RIVER_S + 4, color='#8a6a45', label='bridge'))
    z = cfg['zones']
    for bear, (bx, by) in cfg['bears'].items():
        for r, c in ((z['yellow'], '#e0c040'), (z['red'], '#e07030'), (z['emergency'], '#c02020')):
            ax.add_patch(Circle((bx, by), r, color=c, alpha=0.13, lw=0))
        ax.plot(bx, by, 'k^', ms=9)
        ax.annotate(bear, (bx, by), xytext=(4, -10), textcoords='offset points', fontsize=8)

    # Zone routes drawn in their zone's colour; anything else from tab10.
    colours = {'green_zone': '#2e9e44', 'yellow_zone': '#d4a500', 'red_zone': '#e0592a',
               'emergency_zone': '#9b111e', 'short_route': '#1f77b4',
               'long_route': '#7b4fb0', 'zone_approach': '#e377c2'}
    tab = plt.get_cmap('tab10')
    def cmap(i):
        return colours.get(names[i], tab(i % 10))
    for i, name in enumerate(names):
        pts = world_waypoints(cfg, name)
        mode = cfg['routes'][name].get('mode', 'loop')
        xs = [p[0] for p in pts] + ([pts[0][0]] if mode == 'loop' else [])
        ys = [p[1] for p in pts] + ([pts[0][1]] if mode == 'loop' else [])
        ax.plot(xs, ys, '-o', color=cmap(i), ms=4, lw=1.8, label=f'{name} ({mode})')
        for k, p in enumerate(pts):
            ax.annotate(str(k + 1), p[:2], xytext=(3, 3), textcoords='offset points',
                        fontsize=7, color=cmap(i))

    ax.set_xlim(X_MIN - 10, X_MAX + 10)
    ax.set_ylim(Y_MIN - 10, Y_MAX + 10)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m, world)')
    ax.set_ylabel('y (m, world)')
    ax.set_title('Patrol routes - yellowstone world\n'
                 'shaded rings: yellow 40 m / red 15 m / emergency 10 m')
    ax.legend(loc='lower right', fontsize=8)
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print(f'wrote {out}')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default=DEFAULT_CFG)
    ap.add_argument('--plot', metavar='PNG')
    a = ap.parse_args()
    cfg = load_config(a.config)
    good = report(cfg)
    if a.plot:
        plot(cfg, a.plot)
    sys.exit(0 if good else 1)