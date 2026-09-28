#!/usr/bin/env python3
"""Walk the Yellowstone bears using Gazebo's set_pose service.

Bears only. This deliberately knows nothing about the husky or the parrot: no
ROS topics are read, no robot is commanded, and nothing here affects either
robot's perception or behaviour. It moves bear models and nothing else.

Four bears move:
  * bear_walk_*  - five pieces (body + 4 legs) cut by scripts/split_bear.py.
                   The body bobs, rolls and nods while the legs stride.
  * bear_spawn, bear_meadow, bear_treeline - single rigid meshes, so they get
                   the body motion only: bob, roll sway and pitch nod.

Why there is a C++ helper
-------------------------
Poses go through `pose_bridge`, a resident ignition-transport client built by
this package. Shelling out to `ign service` costs about 330 ms per call, so
eight poses a frame capped the animation near 1.4 Hz and looked jerky no matter
how the gait constants were tuned. Through the transport library a call costs
~0.5 ms - measured 1849 calls/s - so this runs at 30 Hz and looks smooth.

Gait notes
----------
* The bear's nose points -Y, so a leg strides fore/aft by rotating about X, not
  about Y. Rotating about Y splays the legs sideways instead.
* Each hip offset is rotated by the body's full orientation before the leg's
  own swing is applied, or the legs stay level while the body rocks and
  visibly detach.
* Stride phase is integrated from distance covered, not from wall time, so a
  bear that stops also stops moving its legs.
* Heading changes are rate-limited rather than per-tick noise, so the wander
  looks the same at any frame rate.

Everything is <static> and moved by set_pose, so no physics is involved.
"""
import argparse
import math
import os
import random
import shutil
import subprocess
import time

WORLD = 'yellowstone'
HIPS = {'lf': (-0.181, -0.355), 'rf': (0.182, -0.352),
        'lr': (-0.167, 0.353), 'rr': (0.168, 0.355)}
HIP_Z = 0.42
PHASE = {'lf': 0.0, 'rr': 0.25, 'rf': 0.5, 'lr': 0.75}   # four-beat walk

RATE = 30.0          # Hz - what the resident bridge makes possible
SWING = 0.34         # rad of leg swing
BOB, ROLL, NOD = 0.05, 0.08, 0.055
RADIUS = 11.0

# Wandering: hold a heading for a few seconds, then ease onto a new one.
TURN_RATE = 0.55     # rad/s, the fastest a bear will turn
TURN_SPREAD = 1.1    # rad, how far a new heading may differ
PICK_RANGE = (3.0, 6.5)   # seconds between heading changes
PICK_EVERY = 4.0

# A bear does not hold one speed. STRIDE_LEN ties the leg cycle to the ground
# actually covered, so the feet never skate and the legs rest when it stops.
STRIDE_LEN = 0.68    # metres covered per full gait cycle
ACCEL = 0.45         # m/s^2, how briskly a bear changes pace
GAITS = [            # (name, speed m/s, weight, min_s, max_s)
    ('stand', 0.00, 0.22, 2.0, 5.0),
    ('amble', 0.20, 0.30, 3.0, 7.0),
    ('walk',  0.50, 0.36, 4.0, 9.0),
    ('trot',  1.00, 0.12, 2.0, 4.5),
]

BEARS = {
    'bear_walk': ((-13.0, 7.0), RADIUS),
    'bear_spawn': ((11.0, 9.0), RADIUS),
    'bear_meadow': ((-34.0, 150.0), 16.0),
    'bear_treeline': ((-6.0, 84.0), 9.0),
}


def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by, aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw, aw*bw - ax*bx - ay*by - az*bz)


def q_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return (sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy)


def q_rot(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    tx, ty, tz = 2*(y*vz - z*vy), 2*(z*vx - x*vz), 2*(x*vy - y*vx)
    return (vx + w*tx + y*tz - z*ty, vy + w*ty + z*tx - x*tz, vz + w*tz + x*ty - y*tx)


def heading_to_yaw(h):
    """The mesh's nose points -Y; this is the yaw that aims it along h."""
    return math.atan2(math.cos(h), -math.sin(h))


def find_bridge():
    here = os.path.dirname(os.path.abspath(__file__))
    for c in (shutil.which('pose_bridge'),
              os.path.join(here, '..', '..', '..', 'install', '41068_ignition_bringup',
                           'lib', '41068_ignition_bringup', 'pose_bridge'),
              os.path.join(here, 'pose_bridge')):
        if c and os.path.exists(c):
            return os.path.abspath(c)
    raise SystemExit('pose_bridge not found - build the package first')


class Wanderer:
    def __init__(self, home, radius, seed):
        self.home, self.radius = home, radius
        self.x, self.y = home
        self.rng = random.Random(seed)
        self.h = self.rng.uniform(0, 2*math.pi)
        self.target = self.h
        self.until = self.rng.uniform(0.0, PICK_EVERY)
        self.phase = self.rng.uniform(0, 2*math.pi)   # stride phase, integrated
        self.speed = 0.0
        self.want = 0.0
        self.gait_until = 0.0
        self.gait = 'stand'

    def _pick_gait(self, t):
        g = self.rng.choices(range(len(GAITS)), weights=[x[2] for x in GAITS])[0]
        name, spd, _w, lo, hi = GAITS[g]
        self.gait = name
        self.want = spd * self.rng.uniform(0.85, 1.15)
        self.gait_until = t + self.rng.uniform(lo, hi)

    def step(self, dt, t):
        if t >= self.gait_until:
            self._pick_gait(t)

        d = self.want - self.speed
        self.speed += max(-ACCEL*dt, min(ACCEL*dt, d))

        if math.hypot(self.x - self.home[0], self.y - self.home[1]) > self.radius:
            self.target = math.atan2(self.home[1] - self.y, self.home[0] - self.x)
            self.until = t + PICK_EVERY
            if self.want == 0.0:          # do not stand still outside the range
                self.want, self.gait, self.gait_until = 0.5, 'walk', t + 4.0
        elif t >= self.until:
            self.target = self.h + self.rng.uniform(-TURN_SPREAD, TURN_SPREAD)
            self.until = t + self.rng.uniform(*PICK_RANGE)

        err = (self.target - self.h + math.pi) % (2*math.pi) - math.pi
        self.h += max(-TURN_RATE*dt, min(TURN_RATE*dt, err))

        step = self.speed*dt
        self.x += step*math.cos(self.h)
        self.y += step*math.sin(self.h)
        self.phase += 2*math.pi*step/STRIDE_LEN
        return self.x, self.y, self.h


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=600.0)
    ap.add_argument('--only', default=None, help='comma-separated bear names')
    a = ap.parse_args()

    names = a.only.split(',') if a.only else list(BEARS)
    walkers = {n: Wanderer(*BEARS[n], seed=hash(n) & 0xffff) for n in names}

    bridge = subprocess.Popen([find_bridge(), WORLD], stdin=subprocess.PIPE,
                              text=True, bufsize=1)
    time.sleep(1.5)                      # let transport discovery settle
    out = bridge.stdin
    dt, n = 1.0/RATE, int(a.duration*RATE)
    t_start = time.time()
    try:
        for i in range(n):
            t = i*dt
            for name, w in walkers.items():
                x, y, h = w.step(dt, t)
                cyc = w.phase
                amp = min(1.0, w.speed/0.5)      # no bobbing when stood still
                z = BOB*amp*abs(math.sin(cyc))
                qb = q_rpy(ROLL*amp*math.sin(cyc), NOD*amp*math.sin(2*cyc),
                           heading_to_yaw(h))
                if name == 'bear_walk':
                    out.write(f'bear_walk_body {x:.4f} {y:.4f} {z:.4f} '
                              f'{qb[0]:.6f} {qb[1]:.6f} {qb[2]:.6f} {qb[3]:.6f}\n')
                    for part, (hx, hy) in HIPS.items():
                        ox, oy, oz = q_rot(qb, (hx, hy, HIP_Z))
                        sw = SWING*amp*math.sin(cyc + 2*math.pi*PHASE[part])
                        ql = q_mul(qb, q_rpy(sw, 0, 0))
                        out.write(f'bear_walk_{part} {x+ox:.4f} {y+oy:.4f} {z+oz:.4f} '
                                  f'{ql[0]:.6f} {ql[1]:.6f} {ql[2]:.6f} {ql[3]:.6f}\n')
                else:
                    out.write(f'{name} {x:.4f} {y:.4f} {z:.4f} '
                              f'{qb[0]:.6f} {qb[1]:.6f} {qb[2]:.6f} {qb[3]:.6f}\n')
            out.flush()
            time.sleep(dt)
            if i % (int(RATE)*10) == 0:
                real = time.time() - t_start
                gaits = ' '.join(f'{n.split("_")[-1]}:{w.gait}' for n, w in walkers.items())
                print(f'  t={t:6.1f}s (wall {real:6.1f}s, {i/max(real,1e-6):4.1f} Hz)  {gaits}',
                      flush=True)
    finally:
        try:
            out.close()
        except Exception:
            pass
        bridge.terminate()
    print('  done', flush=True)


if __name__ == '__main__':
    main()
