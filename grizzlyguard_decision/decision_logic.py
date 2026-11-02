#!/usr/bin/env python3
"""Canonical decision logic for Yellowstone bear-threat handling.

This module centralizes the relationship between a threat zone, the resulting
robot decision, and the corresponding action to take. It directly mirrors the
threat bands defined in `threat_zones.py` rather than reproducing a demo-only
hardcoded map.
"""

from __future__ import annotations

from dataclasses import dataclass

from threat_zones import ThreatLevel, classify_threat, classify_world_position


@dataclass(frozen=True)
class ThreatDecision:
    """Decision metadata for a single threat zone."""

    threat_level: ThreatLevel
    zone_name: str
    action: str
    deterrent: str
    gui_alert: str

    @property
    def is_active(self) -> bool:
        """Return True while the drone must actively manage the bear."""
        return self.threat_level in (ThreatLevel.YELLOW, ThreatLevel.RED, ThreatLevel.EMERGENCY)


THREAT_DECISION_MAP: dict[ThreatLevel, ThreatDecision] = {
    ThreatLevel.GREEN: ThreatDecision(
        threat_level=ThreatLevel.GREEN,
        zone_name="Green zone",
        action="Continue patrol route",
        deterrent="None",
        gui_alert="Normal",
    ),
    ThreatLevel.YELLOW: ThreatDecision(
        threat_level=ThreatLevel.YELLOW,
        zone_name="Yellow zone",
        action="Approach to 5-10 m and play deterrent sound; send yellow alert to GUI",
        deterrent="Sound",
        gui_alert="Yellow",
    ),
    ThreatLevel.RED: ThreatDecision(
        threat_level=ThreatLevel.RED,
        zone_name="Red zone",
        action="Approach to 5-10 m and spray repellent; send high-priority alert to GUI and play repellant sound",
        deterrent="Spray & Sound",
        gui_alert="High Priority",
    ),
    ThreatLevel.EMERGENCY: ThreatDecision(
        threat_level=ThreatLevel.EMERGENCY,
        zone_name="Emergency zone",
        action="Monitor at 5-10 m and send critical alert to GUI",
        deterrent="Critical Monitor",
        gui_alert="Critical",
    ),
}


def decision_for_threat(threat_level: ThreatLevel) -> ThreatDecision:
    """Return the canonical decision for a threat level."""
    return THREAT_DECISION_MAP[threat_level]


def action_for_threat(threat_level: ThreatLevel) -> str:
    """Return the action associated with a threat level."""
    return decision_for_threat(threat_level).action


def decision_for_distance(distance_to_border_m_value: float) -> ThreatDecision:
    """Classify a distance and return the matching decision record."""
    threat = classify_threat(distance_to_border_m_value)
    return decision_for_threat(threat)


def decision_for_world_position(x: float, y: float, border_x: float = 0.0, border_y: float = 0.0) -> ThreatDecision:
    """Classify a map position and return the matching decision record."""
    threat = classify_world_position(x, y, border_x=border_x, border_y=border_y)
    return decision_for_threat(threat)


def summarize_decision(decision: ThreatDecision) -> str:
    """Create a readable summary of the zone and action."""
    return (
        f"Threat zone: {decision.zone_name} | "
        f"Threat level: {decision.threat_level.value.title()} | "
        f"Action: {decision.action} | "
        f"Deterrent: {decision.deterrent} | "
        f"GUI Alert: {decision.gui_alert}"
    )


if __name__ == '__main__':
    for distance in (2500.0, 1200.0, 350.0, 0.0):
        decision = decision_for_distance(distance)
        print(f'distance={distance:.1f} m -> {decision.threat_level.value:>10} :: {decision.action}')
