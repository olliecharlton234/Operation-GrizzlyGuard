#!/usr/bin/env python3
"""Follow controller for the bear threat system.

This module turns a bear detection and its threat decision into a movement goal.
It is intentionally lightweight and meant to sit between perception and robot
navigation.

Behavior:
- GREEN: hold patrol / no follow
- YELLOW: move to a near-follow offset and keep the bear visible
- RED: move closer and maintain a tighter follow offset
- EMERGENCY: maintain a cautionary offset and prioritize tracking
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from decision_controller import DecisionController
from decision_logic import decision_for_world_position
from threat_zones import ThreatLevel


@dataclass
class BearFollowController:
    """Translate a bear threat decision into a follow/motion policy."""

    follow_offset_m: float = 2.0
    close_offset_m: float = 1.5
    emergency_offset_m: float = 1.0
    controller: DecisionController = None
    current_target: Optional[tuple[float, float]] = None

    def __post_init__(self):
        if self.controller is None:
            self.controller = DecisionController()

    def evaluate_bear(self, x: float, y: float) -> tuple[ThreatLevel, str, tuple[float, float]]:
        """Return threat, action, and follow target for a bear position."""
        decision = decision_for_world_position(x, y, border_x=0.0, border_y=0.0)
        threat = decision.threat_level
        self.controller.update_state_from_threat(threat)

        if threat == ThreatLevel.GREEN:
            target = (x, y)
            action = "Hold patrol and do not chase the bear"
        elif threat == ThreatLevel.YELLOW:
            target = (x + self.follow_offset_m, y)
            action = "Move to follow bear at a safe offset and keep deterrent sound active"
        elif threat == ThreatLevel.RED:
            target = (x + self.close_offset_m, y)
            action = "Move closer and maintain repellent tracking behavior"
        else:
            target = (x + self.emergency_offset_m, y)
            action = "Maintain emergency tracking and critical alert behavior"

        self.current_target = target
        return threat, action, target

    def follow_goal_for_bear(self, x: float, y: float) -> tuple[float, float, str]:
        """Return the next follow goal for the robot or camera."""
        threat, action, target = self.evaluate_bear(x, y)
        return target[0], target[1], action


if __name__ == "__main__":
    controller = BearFollowController()
    for point in [(60.0, 0.0), (25.0, 0.0), (12.0, 0.0), (0.0, 0.0)]:
        threat, action, target = controller.evaluate_bear(*point)
        print(f"bear at {point} -> {threat.value} :: {action} :: target={target}")
