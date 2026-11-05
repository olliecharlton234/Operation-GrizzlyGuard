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

Scenarios
---------
--scenario sets up one test case: one bear walking inside the zone under test
(SCENARIOS below), and every other bear hidden underground so nothing else can
be detected. no_bear and not_a_bear hide them all; the hikers are the only thing
left to see in not_a_bear.

  bear_gait_demo.py --scenario yellow

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

# =============================================================================
# SETTINGS - almost every change to the bears' behaviour is a number in here
# =============================================================================

# Which Gazebo world to send poses to.
WORLD = 'yellowstone'
# Where each leg joins the body of bear_walk, in metres from the body's centre.
# lf/rf/lr/rr = left front, right front, left rear, right rear.
# The nose points along -Y, so the front legs have a negative second number.
HIPS = {'lf': (-0.181, -0.355), 'rf': (0.182, -0.352),
        'lr': (-0.167, 0.353), 'rr': (0.168, 0.355)}
# Height of the hips above the ground, in metres.
HIP_Z = 0.42
# When each leg swings, as a fraction of one stride. They are a quarter of a
# stride apart, so the feet land one at a time.
PHASE = {'lf': 0.0, 'rr': 0.25, 'rf': 0.5, 'lr': 0.75}   # four-beat walk

# How many times a second every bear is updated. This is the 'tick'.
RATE = 30.0          # Hz - what the resident bridge makes possible
SWING = 0.34         # rad of leg swing
# Body motion while walking: up and down (m), side-to-side sway (rad), nod (rad).
BOB, ROLL, NOD = 0.05, 0.08, 0.055

# How a bear steers and stops.
TURN_RATE = 0.55     # rad/s, the fastest a bear will turn
WOBBLE = 0.18        # rad, slow side-to-side drift so paths are not ruler-straight
ARRIVE_R = 0.8       # m, close enough to count as at the goal
PAUSE = (2.0, 6.0)   # seconds stood at each goal

# A bear does not hold one speed. STRIDE_LEN ties the leg cycle to the ground
# actually covered, so the feet never skate and the legs rest when it stops.
STRIDE_LEN = 0.68    # metres covered per full gait cycle
ACCEL = 0.45         # m/s^2, how briskly a bear changes pace
# The four paces. 'weight' is how likely each one is to be picked, and
# min_s/max_s is how long the bear keeps that pace before picking again.
GAITS = [            # (name, speed m/s, weight, min_s, max_s)
    ('stand', 0.00, 0.10, 1.0, 3.0),
    ('amble', 0.20, 0.30, 3.0, 7.0),
    ('walk',  0.50, 0.45, 4.0, 9.0),
    ('trot',  1.00, 0.15, 2.0, 4.5),
]

# Start position, then the goals walked in a loop. The first entry is also a
# goal, so each bear comes back round to where it started.
# Each position is (x, y) in metres on the Gazebo map, with town at (0, 0).
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
# Spray scares a bear more than sound, so it leaves faster.
FLEE = {'sound': ('walk', 0.7), 'spray': ('trot', 1.1)}   # gait, m/s
WORLD_BOUNDS = (-72.0, 72.0, -37.0, 232.0)                # x0, x1, y0, y1, 3 m in from the edge

# Test cases: which bears walk, and where. Each loop stays inside one threat zone
# (distance from town), on the east side where short_route passes, and keeps
# clear of trees. yellow also has a clear lane out to green, or the bear goes out
# of sight behind a tree. Any bear not listed is hidden underground.
SCENARIOS = {
    'no_bear': {},
    'not_a_bear': {},
    'green': {'bear_walk': [(32.0, 32.0), (28.0, 36.0), (32.0, 37.0)]},       # 44-49 m
    'yellow': {'bear_walk': [(18.4, 15.4), (23.5, 18.7), (20.2, 18.0)]},  # 24-30 m
    'red': {'bear_walk': [(11.5, 6.0), (12.5, -2.0), (13.0, 2.0)]},           # 11-14 m
    'emergency': {'bear_walk': [(6.0, 1.0), (8.0, -2.0), (7.0, 3.0)]},        # 6-9 m, in front of the drone's start
}
HIDE_Z = -50.0       # m, far enough under the ground that no sensor can see it


# =============================================================================
# 3D ROTATION HELPERS
# Gazebo describes a rotation as a quaternion, four numbers (x, y, z, w).
# These are standard formulas. Nothing about bear behaviour lives here.
# =============================================================================

# Combine two rotations into one: first b, then a.
def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by, aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw, aw*bw - ax*bx - ay*by - az*bz)


# Turn roll, pitch and yaw angles into a quaternion.
def q_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return (sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy)


# Rotate the point v by the rotation q.
def q_rot(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    tx, ty, tz = 2*(y*vz - z*vy), 2*(z*vx - x*vz), 2*(x*vy - y*vx)
    return (vx + w*tx + y*tz - z*ty, vy + w*ty + z*tx - x*tz, vz + w*tz + x*ty - y*tx)


def heading_to_yaw(h):
    """The mesh's nose points -Y; this is the yaw that aims it along h."""
    return math.atan2(math.cos(h), -math.sin(h))


# Find the pose_bridge program, the small C++ helper that sends poses to Gazebo.
# It keeps one connection open, which is what makes 30 updates a second possible.
def find_bridge():
    here = os.path.dirname(os.path.abspath(__file__))
    for c in (shutil.which('pose_bridge'),
              os.path.join(here, '..', '..', '..', 'install', '41068_ignition_bringup',
                           'lib', '41068_ignition_bringup', 'pose_bridge'),
              os.path.join(here, 'pose_bridge')):
        if c and os.path.exists(c):
            return os.path.abspath(c)
    raise SystemExit('pose_bridge not found - build the package first')


# =============================================================================
# THE BEAR'S BRAIN - one GoalWalker per bear
# It only does the maths for where the bear should be. main() moves the model.
# =============================================================================
class GoalWalker:
    """Walks a loop of goal points, pausing at each one."""

    def __init__(self, goals, seed):
        # The loop of spots, and which one it is walking to (gi = goal index).
        self.goals = list(goals)
        self.gi = 1 % len(self.goals)          # goals[0] is the start position
        # Position, starting at the first spot.
        self.x, self.y = self.goals[0]
        # Each bear has its own random numbers, so the same seed repeats the same walk.
        self.rng = random.Random(seed)
        # h = heading, the direction it faces in radians. It starts facing a random way.
        self.h = self.rng.uniform(0, 2*math.pi)
        self.phase = self.rng.uniform(0, 2*math.pi)   # stride phase, integrated
        self.wobble_f = self.rng.uniform(0.08, 0.15)  # Hz
        # speed = how fast it is moving now. want = the speed its current pace asks for.
        self.speed = 0.0
        self.want = 0.0
        # When the current pace runs out, so a new one gets picked straight away.
        self.gait_until = 0.0
        self.gait = 'stand'
        self.paused_until = 0.0
        self.flee = None          # (x, y) it is running to after a deterrent
        self.flee_gait = ('walk', 0.0)
        self.settled = False      # set for one step when a flee ends

    # Where it is heading: the safe spot if it has been scared, otherwise its next
    # spot. This is how being scared takes priority over its normal walk.
    @property
    def goal(self):
        return self.flee or self.goals[self.gi]

    # Called once when a scare message arrives. It only chooses where to run to;
    # step() does the actual walking there.
    def deter(self, kind):
        """Head away from town until safely in green, taking the loop along."""
        # Find the middle of the bear's loop, and how far its spots reach from there.
        cx = sum(g[0] for g in self.goals) / len(self.goals)
        cy = sum(g[1] for g in self.goals) / len(self.goals)
        loop_r = max(math.hypot(g[0] - cx, g[1] - cy) for g in self.goals)
        # How far from town to go: past the green line by the margin, plus the size of
        # the loop so every spot ends up in green. A bear already further out than
        # that still moves 10 m further away.
        now = math.hypot(self.x - TOWN[0], self.y - TOWN[1])
        reach = max(GREEN_RADIUS + SAFE_MARGIN + loop_r, now + 10.0)
        # The direction pointing directly away from town.
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
        # Remember the safe spot and the moved loop. The loop swaps in on arrival.
        self.flee, self._moved = (px, py), moved
        # Use the pace for this deterrent. An unknown deterrent counts as sound.
        self.flee_gait = FLEE.get(kind, FLEE['sound'])
        self.paused_until = 0.0
        return self.flee

    # Pick a new pace at random, using the weights in GAITS.
    def _pick_gait(self, t):
        g = self.rng.choices(range(len(GAITS)), weights=[x[2] for x in GAITS])[0]
        name, spd, _w, lo, hi = GAITS[g]
        self.gait = name
        # Vary the speed by up to 15% either way so bears don't move identically.
        self.want = spd * self.rng.uniform(0.85, 1.15)
        self.gait_until = t + self.rng.uniform(lo, hi)

    # THE MAIN LOGIC. Runs once per tick (30 times a second) and moves the bear a
    # tiny bit. dt = seconds since the last tick, t = seconds since the start.
    def step(self, dt, t):
        # 1. WHERE AM I GOING? How far is it?
        gx, gy = self.goal
        dist = math.hypot(gx - self.x, gy - self.y)

        # 2. AM I RESTING, OR HAVE I JUST ARRIVED?
        self.settled = False
        if t < self.paused_until:
            want = 0.0
        elif dist < ARRIVE_R:
            # Arrived: start a rest of random length.
            self.paused_until = t + self.rng.uniform(*PAUSE)
            if self.flee:
                # Safe: the loop now lives out here; start it from the nearest spot.
                self.goals, self.flee, self.settled = self._moved, None, True
                self.gi = min(range(len(self.goals)),
                              key=lambda i: math.hypot(self.goals[i][0] - self.x, self.goals[i][1] - self.y))
            else:
                # Normal arrival: next spot. The % wraps from the last back to the first.
                self.gi = (self.gi + 1) % len(self.goals)
            self.gait, self.gait_until, want = 'stand', self.paused_until, 0.0
        else:
            # 3. PICK A PACE. A scared bear always uses its scare pace; otherwise pick a
            # new one whenever the current one has run out.
            if self.flee:
                self.gait, self.want = self.flee_gait
                self.gait_until = t
            elif t >= self.gait_until:
                self._pick_gait(t)
            # 4. TURN towards the target, plus a slow wobble that fades out near the
            # spot. err is the turn still needed, kept between -pi and +pi so the bear
            # always turns the short way round. Each tick it turns by at most TURN_RATE.
            target = math.atan2(gy - self.y, gx - self.x)
            target += WOBBLE*min(1.0, dist/5.0)*math.sin(2*math.pi*self.wobble_f*t)
            err = (target - self.h + math.pi) % (2*math.pi) - math.pi
            self.h += max(-TURN_RATE*dt, min(TURN_RATE*dt, err))
            # 5. PICK A SPEED. cos(err) is 1 facing the target and falls as the turn gets
            # sharper. sqrt(2*ACCEL*dist) is the speed it can still brake from in time.
            # Slow for sharp turns and on the final approach, so the bear
            # neither circles the goal nor overshoots it. The 0.15 m/s floor
            # turns tighter than ARRIVE_R, so it always gets there; a
            # mid-route 'stand' gait (want 0) is left alone.
            want = min(self.want * math.cos(err), math.sqrt(2*ACCEL*dist))
            if self.want > 0.0:
                want = max(want, 0.15)

        # 6. MOVE. Change speed gently, limited by ACCEL, so it never jumps from
        # standing to trotting.
        d = want - self.speed
        self.speed += max(-ACCEL*dt, min(ACCEL*dt, d))

        # Step forward along the heading. distance = speed x time.
        step = self.speed*dt
        self.x += step*math.cos(self.h)
        self.y += step*math.sin(self.h)
        # Advance the leg cycle by the distance walked, not by the clock, so the feet
        # don't slide and the legs stop when the bear stops.
        self.phase += 2*math.pi*step/STRIDE_LEN
        return self.x, self.y, self.h


# =============================================================================
# COMMAND-LINE OPTIONS AND SCARE MESSAGES
# =============================================================================

# Read the --goals option.
def parse_goals(spec):
    """'name:x,y;x,y name2:x,y' -> {name: [(x, y), ...]}"""
    out = {}
    for part in spec.split():
        name, pts = part.split(':', 1)
        out[name] = [tuple(float(c) for c in p.split(',')) for p in pts.split(';') if p]
        if not out[name] or any(len(p) != 2 for p in out[name]):
            raise SystemExit(f'bad --goals entry: {part}')
    return out


# Read the --deter option, which schedules a scare at a set time.
def parse_deter(spec):
    """'name:kind@t,name@t' -> [(t, 'name kind'), ...]; kind defaults to sound."""
    out = []
    for part in filter(None, spec.split(',')):
        who, _, at = part.partition('@')
        if not at:
            raise SystemExit(f'bad --deter entry (needs @seconds): {part}')
        out.append((float(at), who.replace(':', ' ')))
    return sorted(out)


# Listen on the ROS topic /bear_deterrent in a background thread. Messages only go
# into inbox; main() acts on them between ticks, so a bear is never changed halfway
# through a step. Returns a function that shuts the listener down.
def listen_for_deterrents(inbox):
    """Push each /bear_deterrent message into inbox. Optional: no ROS, no topic."""
    try:
        import rclpy
        import rclpy.executors
        import rclpy.signals
        from std_msgs.msg import String
    except ImportError:
        print('  rclpy not found: /bear_deterrent disabled (--deter still works)', flush=True)
        return None
    # Leave Ctrl-C and kill to Python: rclpy's own handlers would swallow them and
    # the bear loop, which never checks rclpy.ok(), would keep running.
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = rclpy.create_node('bear_deterrent_listener')
    node.create_subscription(String, '/bear_deterrent', lambda m: inbox.append(m.data), 10)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    def stop():
        # Join the spin thread before Python exits, or the process aborts on the way out.
        executor.shutdown()
        spinner.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
    return stop


# Act on one scare message.
def apply_deterrent(text, walkers):
    """'bear_spawn spray' / 'bear_spawn' / 'all sound' -> make those bears flee."""
    words = text.split()
    if not words:
        return
    who, kind = words[0], (words[1] if len(words) > 1 else 'sound')
    # 'all' means every bear that is walking.
    targets = list(walkers) if who == 'all' else [who]
    for name in targets:
        if name not in walkers:
            print(f'  deterrent for unknown bear {name!r} ignored', flush=True)
            continue
        fx, fy = walkers[name].deter(kind)
        print(f'  {name}: {kind}! heading away from town to ({fx:.0f}, {fy:.0f})', flush=True)


# Used by --scenario, so the only bear the drone can see is the one being tested.
def hide_bears(names, out):
    """Park bears underground. Re-sent regularly in case Gazebo started late."""
    for name in names:
        parts = [f'bear_walk_{p}' for p in ('body', *HIPS)] if name == 'bear_walk' else [name]
        for part in parts:
            out.write(f'{part} 0 0 {HIDE_Z} 0 0 0 1\n')


# =============================================================================
# MAIN PROGRAM - sets everything up, then runs the 30-times-a-second loop
# =============================================================================
def main():
    # The command-line options.
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario', default=None, choices=sorted(SCENARIOS),
                    help='one test case: one bear in that zone, the others hidden')
    ap.add_argument('--duration', type=float, default=600.0)
    ap.add_argument('--only', default=None, help='comma-separated bear names')
    ap.add_argument('--goals', default='',
                    help="override goals: 'name:x,y;x,y name2:x,y' (first point is the start)")
    ap.add_argument('--seed', type=int, default=0, help='change for a different walk')
    ap.add_argument('--deter', default='',
                    help="deter bears at set times: 'bear_spawn:spray@30,bear_walk@60'")
    a = ap.parse_args()

    # Work out which bears walk and where. Later entries win, so --goals overrides
    # the scenario, which overrides the normal BEARS list.
    scenario = SCENARIOS.get(a.scenario, {})
    bears = {**BEARS, **scenario, **parse_goals(a.goals)}
    names = a.only.split(',') if a.only else list(scenario if a.scenario else BEARS)
    hidden = [b for b in BEARS if b not in names] if a.scenario else []
    walkers = {n: GoalWalker(bears[n], seed=a.seed*1000 + i) for i, n in enumerate(names)}
    # Two sources of scares: timed ones from --deter, and live ROS messages.
    scheduled = deque(parse_deter(a.deter))
    inbox = deque()
    stop_listening = listen_for_deterrents(inbox)

    # Start the C++ helper. Anything written to 'out' becomes a pose in Gazebo.
    bridge = subprocess.Popen([find_bridge(), WORLD], stdin=subprocess.PIPE,
                              text=True, bufsize=1)
    time.sleep(1.5)                      # let transport discovery settle
    out = bridge.stdin
    dt, n = 1.0/RATE, int(a.duration*RATE)
    t_start = time.time()
    try:
        # THE LOOP. Each pass is one tick.
        for i in range(n):
            t = i*dt
            # Once a second, make sure hidden bears are still hidden.
            if i % int(RATE) == 0:
                hide_bears(hidden, out)
            # Handle any scares that are due, before the bears take their step.
            while scheduled and scheduled[0][0] <= t:
                inbox.append(scheduled.popleft()[1])
            while inbox:
                apply_deterrent(inbox.popleft(), walkers)
            # Move every bear.
            for name, w in walkers.items():
                # Ask the brain where this bear should be now.
                x, y, h = w.step(dt, t)
                if w.settled:
                    print(f'  {name}: safe in green, {math.hypot(x - TOWN[0], y - TOWN[1]):.0f} m '
                          f'from town', flush=True)
                # Work out the body motion. amp scales it with speed, up to full at
                # 0.5 m/s, so a bear that is standing still doesn't move.
                cyc = w.phase
                amp = min(1.0, w.speed/0.5)      # no bobbing when stood still
                # z = the bob. qb = the body's sway, nod and direction, as one rotation.
                z = BOB*amp*abs(math.sin(cyc))
                qb = q_rpy(ROLL*amp*math.sin(cyc), NOD*amp*math.sin(2*cyc),
                           heading_to_yaw(h))
                # bear_walk is five models (body + 4 legs), so it needs five poses.
                if name == 'bear_walk':
                    out.write(f'bear_walk_body {x:.4f} {y:.4f} {z:.4f} '
                              f'{qb[0]:.6f} {qb[1]:.6f} {qb[2]:.6f} {qb[3]:.6f}\n')
                    for part, (hx, hy) in HIPS.items():
                        # Turn the hip with the body so the leg stays attached as it rocks.
                        ox, oy, oz = q_rot(qb, (hx, hy, HIP_Z))
                        # Swing this leg, offset by its PHASE, on top of the body's rotation.
                        sw = SWING*amp*math.sin(cyc + 2*math.pi*PHASE[part])
                        ql = q_mul(qb, q_rpy(sw, 0, 0))
                        out.write(f'bear_walk_{part} {x+ox:.4f} {y+oy:.4f} {z+oz:.4f} '
                                  f'{ql[0]:.6f} {ql[1]:.6f} {ql[2]:.6f} {ql[3]:.6f}\n')
                else:
                    # The other bears are one solid model, so they need one pose each.
                    out.write(f'{name} {x:.4f} {y:.4f} {z:.4f} '
                              f'{qb[0]:.6f} {qb[1]:.6f} {qb[2]:.6f} {qb[3]:.6f}\n')
            # Send this tick's poses to Gazebo, then wait for the next tick.
            out.flush()
            time.sleep(dt)
            # Every 10 seconds, print each bear's pace and where it's heading.
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
        # Clean up, even if the program was stopped with Ctrl-C.
        bridge.terminate()
        if stop_listening is not None:
            stop_listening()
    print('  done', flush=True)


if __name__ == '__main__':
    main()
