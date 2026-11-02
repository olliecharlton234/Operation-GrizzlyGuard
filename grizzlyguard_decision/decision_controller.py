#!/usr/bin/env python3
"""Minimal decision controller for Yellowstone bear-threat handling.

This module preserves the same behavior pattern as the prior controller:

- classify the bear's threat from its distance to the border
- map each threat level to the canonical action
- maintain a simple finite-state response loop
- expose the decision and action so higher-level code can use it

It intentionally does not depend on visualization or test scaffolding.
"""

from __future__ import annotations

from dataclasses import dataclass

from decision_logic import THREAT_DECISION_MAP, ThreatDecision, decision_for_distance, decision_for_world_position
from threat_zones import ThreatLevel


@dataclass
class DecisionController:
    """Simple rule-based controller that translates threat state into actions."""

    state: str = "patrol"
    last_threat: ThreatLevel = ThreatLevel.GREEN
    follow_offset_m: float = 2.0

    def evaluate_distance(self, distance_to_border_m: float) -> ThreatLevel:
        """Classify the bear threat and store the latest threat state."""
        threat = decision_for_distance(distance_to_border_m).threat_level
        self.last_threat = threat
        return threat

    def action_for_distance(self, distance_to_border_m: float) -> str:
        """Return the canonical action for the given distance."""
        return decision_for_distance(distance_to_border_m).action

    def evaluate_world_position(self, x: float, y: float, border_x: float = 0.0, border_y: float = 0.0) -> ThreatLevel:
        """Classify a bear's world location and update the controller state."""
        decision = decision_for_world_position(x, y, border_x=border_x, border_y=border_y)
        threat = decision.threat_level
        self.last_threat = threat
        self.update_state_from_threat(threat)
        return threat

    def update_state_from_threat(self, threat: ThreatLevel) -> ThreatLevel:
        """Apply the same state machine logic directly from a threat level."""
        if self.state == "patrol":
            if threat == ThreatLevel.GREEN:
                self.state = "patrol"
            else:
                self.state = "yellow_response"
            return threat

        if self.state == "yellow_response":
            if threat == ThreatLevel.GREEN:
                self.state = "returning_to_green"
            elif threat == ThreatLevel.RED:
                self.state = "red_response"
            elif threat == ThreatLevel.EMERGENCY:
                self.state = "emergency_monitor"
            return threat

        if self.state == "red_response":
            if threat == ThreatLevel.GREEN:
                self.state = "returning_to_green"
            elif threat == ThreatLevel.EMERGENCY:
                self.state = "emergency_monitor"
            return threat

        if self.state == "emergency_monitor":
            if threat == ThreatLevel.GREEN:
                self.state = "returning_to_green"
            return threat

        if self.state == "returning_to_green":
            if threat == ThreatLevel.GREEN:
                self.state = "patrol"
            return threat

        return threat

    def update_state(self, distance_to_border_m: float) -> ThreatLevel:
        """Apply the state machine response for the current bear distance."""
        threat = self.evaluate_distance(distance_to_border_m)

        return self.update_state_from_threat(threat)

    def process_world_position(self, x: float, y: float, border_x: float = 0.0, border_y: float = 0.0) -> ThreatDecision:
        """Return the decision generated from bear world coordinates and update controller state."""
        decision = decision_for_world_position(x, y, border_x=border_x, border_y=border_y)
        self.last_threat = decision.threat_level
        self.update_state_from_threat(decision.threat_level)
        return decision

    def current_action(self, distance_to_border_m: float) -> str:
        """Return the action for the current distance and current state."""
        threat = self.evaluate_distance(distance_to_border_m)
        if threat == ThreatLevel.GREEN:
            return THREAT_DECISION_MAP[ThreatLevel.GREEN].action
        if threat == ThreatLevel.YELLOW:
            return THREAT_DECISION_MAP[ThreatLevel.YELLOW].action
        if threat == ThreatLevel.RED:
            return THREAT_DECISION_MAP[ThreatLevel.RED].action
        return THREAT_DECISION_MAP[ThreatLevel.EMERGENCY].action


if __name__ == "__main__":
    controller = DecisionController()
    for distance in (2500.0, 1200.0, 350.0, 0.0, 2600.0):
        threat = controller.update_state(distance)
        print(f"distance={distance:.1f} m -> {threat.value} :: {controller.current_action(distance)} | state={controller.state}")
