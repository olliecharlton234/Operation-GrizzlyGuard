"""Load patrol routes from config/patrol_routes.yaml.

Shared by patrol_node.py (to fly them) and check_routes.py (to check/plot them),
so both always agree on where the waypoints are. No ROS imports here.
"""
import math

import yaml


def load_config(path):
    with open(path) as f:
        return yaml.safe_load(f)


def route_names(cfg):
    return list(cfg['routes'].keys())


def world_waypoints(cfg, name):
    """Return [(x, y, z), ...] in Gazebo WORLD coordinates for route `name`."""
    routes = cfg['routes']
    if name not in routes:
        raise KeyError(f"Unknown route '{name}'. Choose one of: {', '.join(routes)}")
    r = routes[name]
    dz = float(cfg.get('default_z', 0.0))

    if 'waypoints' in r:
        pts = []
        for p in r['waypoints']:
            x, y = float(p[0]), float(p[1])
            z = float(p[2]) if len(p) > 2 else dz
            pts.append((x, y, z))
        return pts

    if 'ring' in r:
        ring = r['ring']
        cx, cy = cfg['bears'][ring['center']] if isinstance(ring['center'], str) else ring['center']
        rad = float(ring['radius'])
        n = int(ring['points'])
        a0, a1 = float(ring['start_deg']), float(ring['end_deg'])
        step = (a1 - a0) / (n - 1) if n > 1 else 0.0
        return [(cx + rad * math.cos(math.radians(a0 + i * step)),
                 cy + rad * math.sin(math.radians(a0 + i * step)),
                 float(ring.get('z', dz))) for i in range(n)]

    raise ValueError(f"Route '{name}' needs either 'waypoints' or 'ring'")


def map_waypoints(cfg, name):
    """Same as world_waypoints, shifted into the robot's map frame."""
    sx = float(cfg['spawn']['x'])
    sy = float(cfg['spawn']['y'])
    return [(x - sx, y - sy, z) for x, y, z in world_waypoints(cfg, name)]


def route_mode(cfg, name):
    return cfg['routes'][name].get('mode', 'loop')


def classify(cfg, x, y):
    """Return (zone, nearest_bear, distance) for a WORLD-coordinate point."""
    bear, d = min(((b, math.hypot(x - bx, y - by)) for b, (bx, by) in cfg['bears'].items()),
                  key=lambda t: t[1])
    z = cfg['zones']
    if d <= z['emergency']:
        zone = 'EMERGENCY'
    elif d < z['red']:
        zone = 'RED'
    elif d <= z['yellow']:
        zone = 'YELLOW'
    else:
        zone = 'GREEN'
    return zone, bear, d