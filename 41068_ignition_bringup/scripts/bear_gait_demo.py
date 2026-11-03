#!/usr/bin/env python3
"""Walk the Yellowstone bears using Gazebo's set_pose service.

Bears only. This deliberately knows nothing about the husky or the parrot: no
robot is commanded, and nothing here affects either robot's perception or
behaviour. It moves bear models and nothing else. The one thing it listens to
is /bear_deterrent (see Deterrents below).

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
* Heading changes are rate-limited rather than per-tick noise, so the walk
  looks the same at any frame rate.

Goals
-----
Each bear walks a loop of goal points (BEARS below), pausing at each one before
heading for the next. Override a bear's goals from the command line, e.g. send
bear_meadow at the town to exercise the threat zones:

  bear_gait_demo.py --goals 'bear_meadow:-34,150;0,15'

Deterrents
----------
When the drone deters a bear, the bear drops its loop and heads straight away
from town until it is SAFE_MARGIN past the green line, rests, and carries on
with its loop moved out there, so it does not wander straight back in. Sound
sends it off at a brisk walk, spray at a trot. Trigger it with a std_msgs/String
on /bear_deterrent holding the bear's name and optionally the deterrent:

  ros2 topic pub --once /bear_deterrent std_msgs/msg/String "data: 'bear_spawn spray'"

or, without ROS, at a set time: bear_gait_demo.py --deter 'bear_spawn:spray@30'

Everything is <static> and moved by set_pose, so no physics is involved.
"""
import argparse
import math
import os
import random
import shutil
import subprocess
import threading
import time
from collections import deque

WORLD = 'yellowstone'
HIPS = {'lf': (-0.181, -0.355), 'rf': (0.182, -0.352),
        'lr': (-0.167, 0.353), 'rr': (0.168, 0.355)}
HIP_Z = 0.42
PHASE = {'lf': 0.0, 'rr': 0.25, 'rf': 0.5, 'lr': 0.75}   # four-beat walk

RATE = 30.0          # Hz - what the resident bridge makes possible
SWING = 0.34         # rad of leg swing
BOB, ROLL, NOD = 0.05, 0.08, 0.055

TURN_RATE = 0.55     # rad/s, the fastest a bear will turn
WOBBLE = 0.18        # rad, slow side-to-side drift so paths are not ruler-straight
ARRIVE_R = 0.8       # m, close enough to count as at the goal
PAUSE = (2.0, 6.0)   # seconds stood at each goal

# A bear does not hold one speed. STRIDE_LEN ties the leg cycle to the ground
# actually covered, so the feet never skate and the legs rest when it stops.
STRIDE_LEN = 0.68    # metres covered per full gait cycle
ACCEL = 0.45         # m/s^2, how briskly a bear changes pace
GAITS = [            # (name, speed m/s, weight, min_s, max_s)
    ('stand', 0.00, 0.10, 1.0, 3.0),
    ('amble', 0.20, 0.30, 3.0, 7.0),
    ('walk',  0.50, 0.45, 4.0, 9.0),
    ('trot',  1.00, 0.15, 2.0, 4.5),
]

# Start position, then the goals walked in a loop. The first entry is also a
# goal, so each bear comes back round to where it started.
BEARS = {
    'bear_walk': [(-13.0, 7.0), (-20.0, 14.0), (-8.0, 16.0), (-17.0, 0.0)],
    'bear_spawn': [(11.0, 9.0), (18.0, 15.0), (6.0, 18.0), (15.0, 2.0)],
    'bear_meadow': [(-34.0, 150.0), (-22.0, 162.0), (-45.0, 160.0), (-30.0, 136.0)],
    'bear_treeline': [(-6.0, 84.0), (1.0, 90.0), (-12.0, 89.0), (-4.0, 77.0)],
}

# Deterrents. Town and the green line match grizzlyguard_decision/threat_zones.py.
TOWN = (0.0, 0.0)
GREEN_RADIUS = 40.0  # m from town; beyond this the threat is green
SAFE_MARGIN = 5.0    # m past the green line before the bear settles again
FLEE = {'sound': ('walk', 0.7), 'spray': ('trot', 1.1)}   # gait, m/s
WORLD_BOUNDS = (-72.0, 72.0, -37.0, 232.0)                # x0, x1, y0, y1, 3 m in from the edge


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


class GoalWalker:
    """Walks a loop of goal points, pausing at each one."""

    def __init__(self, goals, seed):
        self.goals = list(goals)
        self.gi = 1 % len(self.goals)          # goals[0] is the start position
        self.x, self.y = self.goals[0]
        self.rng = random.Random(seed)
        self.h = self.rng.uniform(0, 2*math.pi)
        self.phase = self.rng.uniform(0, 2*math.pi)   # stride phase, integrated
        self.wobble_f = self.rng.uniform(0.08, 0.15)  # Hz
        self.speed = 0.0
        self.want = 0.0
        self.gait_until = 0.0
        self.gait = 'stand'
        self.paused_until = 0.0
        self.flee = None          # (x, y) it is running to after a deterrent
        self.flee_gait = ('walk', 0.0)
        self.settled = False      # set for one step when a flee ends

    @property
    def goal(self):
        return self.flee or self.goals[self.gi]

    def deter(self, kind):
        """Head away from town until safely in green, taking the loop along."""
        cx = sum(g[0] for g in self.goals) / len(self.goals)
        cy = sum(g[1] for g in self.goals) / len(self.goals)
        loop_r = max(math.hypot(g[0] - cx, g[1] - cy) for g in self.goals)
        now = math.hypot(self.x - TOWN[0], self.y - TOWN[1])
        reach = max(GREEN_RADIUS + SAFE_MARGIN + loop_r, now + 10.0)
        away = math.atan2(self.y - TOWN[1], self.x - TOWN[0])
        x0, x1, y0, y1 = WORLD_BOUNDS
        # Straight away from town, or the nearest direction that keeps the
        # whole moved loop inside the map.
        for off in sorted((k*0.2 for k in range(-15, 16)), key=abs):
            px = TOWN[0] + reach*math.cos(away + off)
            py = TOWN[1] + reach*math.sin(away + off)
            moved = [(g[0] - cx + px, g[1] - cy + py) for g in self.goals]
            if all(x0 <= x <= x1 and y0 <= y <= y1 for x, y in moved):
                break
        self.flee, self._moved = (px, py), moved
        self.flee_gait = FLEE.get(kind, FLEE['sound'])
        self.paused_until = 0.0
        return self.flee

    def _pick_gait(self, t):
        g = self.rng.choices(range(len(GAITS)), weights=[x[2] for x in GAITS])[0]
        name, spd, _w, lo, hi = GAITS[g]
        self.gait = name
        self.want = spd * self.rng.uniform(0.85, 1.15)
        self.gait_until = t + self.rng.uniform(lo, hi)

    def step(self, dt, t):
        gx, gy = self.goal
        dist = math.hypot(gx - self.x, gy - self.y)

        self.settled = False
        if t < self.paused_until:
            want = 0.0
        elif dist < ARRIVE_R:
            self.paused_until = t + self.rng.uniform(*PAUSE)
            if self.flee:
                # Safe: the loop now lives out here; start it from the nearest spot.
                self.goals, self.flee, self.settled = self._moved, None, True
                self.gi = min(range(len(self.goals)),
                              key=lambda i: math.hypot(self.goals[i][0] - self.x, self.goals[i][1] - self.y))
            else:
                self.gi = (self.gi + 1) % len(self.goals)
            self.gait, self.gait_until, want = 'stand', self.paused_until, 0.0
        else:
            if self.flee:
                self.gait, self.want = self.flee_gait
                self.gait_until = t
            elif t >= self.gait_until:
                self._pick_gait(t)
            target = math.atan2(gy - self.y, gx - self.x)
            target += WOBBLE*min(1.0, dist/5.0)*math.sin(2*math.pi*self.wobble_f*t)
            err = (target - self.h + math.pi) % (2*math.pi) - math.pi
            self.h += max(-TURN_RATE*dt, min(TURN_RATE*dt, err))
            # Slow for sharp turns and on the final approach, so the bear
            # neither circles the goal nor overshoots it. The 0.15 m/s floor
            # turns tighter than ARRIVE_R, so it always gets there; a
            # mid-route 'stand' gait (want 0) is left alone.
            want = min(self.want * math.cos(err), math.sqrt(2*ACCEL*dist))
            if self.want > 0.0:
                want = max(want, 0.15)

        d = want - self.speed
        self.speed += max(-ACCEL*dt, min(ACCEL*dt, d))

        step = self.speed*dt
        self.x += step*math.cos(self.h)
        self.y += step*math.sin(self.h)
        self.phase += 2*math.pi*step/STRIDE_LEN
        return self.x, self.y, self.h


def parse_goals(spec):
    """'name:x,y;x,y name2:x,y' -> {name: [(x, y), ...]}"""
    out = {}
    for part in spec.split():
        name, pts = part.split(':', 1)
        out[name] = [tuple(float(c) for c in p.split(',')) for p in pts.split(';') if p]
        if not out[name] or any(len(p) != 2 for p in out[name]):
            raise SystemExit(f'bad --goals entry: {part}')
    return out


def parse_deter(spec):
    """'name:kind@t,name@t' -> [(t, 'name kind'), ...]; kind defaults to sound."""
    out = []
    for part in filter(None, spec.split(',')):
        who, _, at = part.partition('@')
        if not at:
            raise SystemExit(f'bad --deter entry (needs @seconds): {part}')
        out.append((float(at), who.replace(':', ' ')))
    return sorted(out)


def listen_for_deterrents(inbox):
    """Push each /bear_deterrent message into inbox. Optional: no ROS, no topic."""
    try:
        import rclpy
        from std_msgs.msg import String
    except ImportError:
        print('  rclpy not found: /bear_deterrent disabled (--deter still works)', flush=True)
        return None
    rclpy.init()
    node = rclpy.create_node('bear_deterrent_listener')
    node.create_subscription(String, '/bear_deterrent', lambda m: inbox.append(m.data), 10)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    return node


def apply_deterrent(text, walkers):
    """'bear_spawn spray' / 'bear_spawn' / 'all sound' -> make those bears flee."""
    words = text.split()
    if not words:
        return
    who, kind = words[0], (words[1] if len(words) > 1 else 'sound')
    targets = list(walkers) if who == 'all' else [who]
    for name in targets:
        if name not in walkers:
            print(f'  deterrent for unknown bear {name!r} ignored', flush=True)
            continue
        fx, fy = walkers[name].deter(kind)
        print(f'  {name}: {kind}! heading away from town to ({fx:.0f}, {fy:.0f})', flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=600.0)
    ap.add_argument('--only', default=None, help='comma-separated bear names')
    ap.add_argument('--goals', default='',
                    help="override goals: 'name:x,y;x,y name2:x,y' (first point is the start)")
    ap.add_argument('--seed', type=int, default=0, help='change for a different walk')
    ap.add_argument('--deter', default='',
                    help="deter bears at set times: 'bear_spawn:spray@30,bear_walk@60'")
    a = ap.parse_args()

    bears = {**BEARS, **parse_goals(a.goals)}
    names = a.only.split(',') if a.only else list(BEARS)
    walkers = {n: GoalWalker(bears[n], seed=a.seed*1000 + i) for i, n in enumerate(names)}
    scheduled = deque(parse_deter(a.deter))
    inbox = deque()
    ros_node = listen_for_deterrents(inbox)

    bridge = subprocess.Popen([find_bridge(), WORLD], stdin=subprocess.PIPE,
                              text=True, bufsize=1)
    time.sleep(1.5)                      # let transport discovery settle
    out = bridge.stdin
    dt, n = 1.0/RATE, int(a.duration*RATE)
    t_start = time.time()
    try:
        for i in range(n):
            t = i*dt
            while scheduled and scheduled[0][0] <= t:
                inbox.append(scheduled.popleft()[1])
            while inbox:
                apply_deterrent(inbox.popleft(), walkers)
            for name, w in walkers.items():
                x, y, h = w.step(dt, t)
                if w.settled:
                    print(f'  {name}: safe in green, {math.hypot(x - TOWN[0], y - TOWN[1]):.0f} m '
                          f'from town', flush=True)
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
                gaits = ' '.join(f'{n.split("_")[-1]}:{w.gait}->({w.goal[0]:.0f},{w.goal[1]:.0f})'
                                 for n, w in walkers.items())
                print(f'  t={t:6.1f}s (wall {real:6.1f}s, {i/max(real,1e-6):4.1f} Hz)  {gaits}',
                      flush=True)
    finally:
        try:
            out.close()
        except Exception:
            pass
        bridge.terminate()
        if ros_node is not None:
            ros_node.destroy_node()
    print('  done', flush=True)


if __name__ == '__main__':
    main()
