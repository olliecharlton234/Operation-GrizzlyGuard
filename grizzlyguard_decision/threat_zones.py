#!/usr/bin/env python3
"""Threat-zone definitions and classification logic.

World boundaries:
  x: -75.0 .. 75.0
  y: -40.0 .. 235.0

The operational border is represented as a circular border around the map
centre at (0.0, 0.0). Zones are concentric circular bands based on the Euclidean
radius from that border centre.

Real-world policy values (conceptual only, not world-space sim coordinates):
  - GREEN:     radius > 2000.0 m
  - YELLOW:    500.0 < radius <= 2000.0 m
  - RED:       0.0 < radius < 500.0 m
  - EMERGENCY: radius <= 0.0 m

Actual sim-space zone table to be used for the world geometry:
  - GREEN:     radius > 40.0 m
  - YELLOW:    15.0 < radius <= 40.0 m
  - RED:       10.0 < radius < 15.0 m
  - EMERGENCY: radius <= 10.0 m

This is a circular ring model for the actual Gazebo map. The emergency band is
kept as a finite circular region around the border centre because the town and
people are spread across an area, not concentrated at a single point. The
real-world 500 m / 2000 m values are recorded as documentation and should not be
used as direct world coordinates inside the simulation.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum


WORLD_X_MIN = -75.0
WORLD_X_MAX = 75.0
WORLD_Y_MIN = -40.0
WORLD_Y_MAX = 235.0
BORDER_X = 0.0
BORDER_Y = 0.0


class ThreatLevel(str, Enum):
    GREEN = "green"
    YELLOW = "yellow"
    RED = "red"
    EMERGENCY = "emergency"


@dataclass(frozen=True)
class ThreatZone:
    name: ThreatLevel
    min_distance_m: float
    max_distance_m: float | None = None
    deterrent: str = ""
    gui_alert: str = ""


THREAT_ZONES = [
    ThreatZone(
        name=ThreatLevel.GREEN,
        min_distance_m=40.0,
        max_distance_m=None,
        deterrent="None",
        gui_alert="Normal",
    ),
    ThreatZone(
        name=ThreatLevel.YELLOW,
        min_distance_m=15.0,
        max_distance_m=40.0,
        deterrent="Sound",
        gui_alert="Yellow",
    ),
    ThreatZone(
        name=ThreatLevel.RED,
        min_distance_m=0.0,
        max_distance_m=15.0,
        deterrent="Spray & Sound",
        gui_alert="High Priority",
    ),
    ThreatZone(
        name=ThreatLevel.EMERGENCY,
        min_distance_m=-1.0,
        max_distance_m=0.0,
        deterrent="Critical Monitor",
        gui_alert="Critical",
    ),
]


def distance_to_border_m(x: float, y: float, border_x: float = BORDER_X, border_y: float = BORDER_Y) -> float:
    """Euclidean distance from a world point to the circular border centre."""
    return math.hypot(x - border_x, y - border_y)


def classify_threat(distance_to_border_m_value: float) -> ThreatLevel:
    """Return the threat level for a bear distance from the border centre.

    This project uses the actual sim-space circular ring model because the world
    is a map, not a single x-axis strip. The real-world 500 m / 2000 m values are
    retained only as documentation of the conceptual policy scale and not used as
    direct world-space coordinates inside the simulator.
    """
    if distance_to_border_m_value <= 10.0:
        return ThreatLevel.EMERGENCY
    if distance_to_border_m_value < 15.0:
        return ThreatLevel.RED
    if distance_to_border_m_value <= 40.0:
        return ThreatLevel.YELLOW
    return ThreatLevel.GREEN


def classify_world_position(x: float, y: float, border_x: float = BORDER_X, border_y: float = BORDER_Y) -> ThreatLevel:
    """Classify a world position according to the Yellowstone map geometry.

    Actual sim-space bands are concentric circles around the border centre:
      - GREEN:     radius > 40.0 m
      - YELLOW:    15.0 < radius <= 40.0 m
      - RED:       10.0 < radius < 15.0 m
      - EMERGENCY: radius <= 10.0 m

    The map extents are x = -75..75 and y = -40..235, but the decision logic is
    driven by the Euclidean distance to the border centre rather than a single
    x-axis line.
    """
    if not (WORLD_X_MIN <= x <= WORLD_X_MAX):
        raise ValueError(f'x={x} is outside the Yellowstone world bounds [{WORLD_X_MIN}, {WORLD_X_MAX}]')
    if not (WORLD_Y_MIN <= y <= WORLD_Y_MAX):
        raise ValueError(f'y={y} is outside the Yellowstone world bounds [{WORLD_Y_MIN}, {WORLD_Y_MAX}]')
    return classify_threat(distance_to_border_m(x, y, border_x=border_x, border_y=border_y))


def threat_summary(threat: ThreatLevel) -> str:
    zone = next(z for z in THREAT_ZONES if z.name == threat)
    return (
        f"Threat level: {threat.value.title()} | "
        f"Deterrent: {zone.deterrent} | "
        f"GUI Alert: {zone.gui_alert}"
    )
