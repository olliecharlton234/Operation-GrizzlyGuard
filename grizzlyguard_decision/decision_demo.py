#!/usr/bin/env python3
"""Simple threat-level decision logic for the Yellowstone bear patrol scenario.

This module is intentionally lightweight and simulator-friendly: it defines the
threat-zone boundaries and the associated drone actions, and can be run either
as a library for tests or as a ROS-like demo script.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from decision_logic import decision_for_distance, decision_for_threat
from threat_zones import ThreatLevel, classify_threat


@dataclass
class ThreatDecisionEngine:
    distance_to_border_m: float
    threat_level: Optional[ThreatLevel] = None
    action: str = ""

    def __post_init__(self) -> None:
        decision = decision_for_distance(self.distance_to_border_m)
        self.threat_level = decision.threat_level
        self.action = decision.action

    @staticmethod
    def _action_for_level(level: ThreatLevel) -> str:
        return decision_for_threat(level).action

    def monitor_until_safe(self) -> bool:
        """Return True while the bear is still in a managed threat state."""
        return self.threat_level in (ThreatLevel.YELLOW, ThreatLevel.RED, ThreatLevel.EMERGENCY)

    def step(self, new_distance_m: float) -> "ThreatDecisionEngine":
        return ThreatDecisionEngine(new_distance_m)


def demo_sequence() -> list[tuple[float, ThreatLevel, str]]:
    """Return a small set of representative scenario values for demonstration."""
    return [
        (2500.0, ThreatLevel.GREEN, "Continue patrol route"),
        (1200.0, ThreatLevel.YELLOW, "Approach to 5-10 m and play deterrent sound; send yellow alert to GUI"),
        (350.0, ThreatLevel.RED, "Approach to 5-10 m and spray repellent; send high-priority alert to GUI and play repellant sound"),
        (0.0, ThreatLevel.EMERGENCY, "Monitor at 5-10 m and send critical alert to GUI"),
    ]


if __name__ == '__main__':
    print('Threat-zone decision demo')
    for distance, level, action in demo_sequence():
        print(f'distance={distance:.1f} m -> {level.value:>10} :: {action}')
