import os
import sys
import unittest
from unittest.mock import patch

import rclpy

sys.path.insert(0, os.path.dirname(__file__))

from decision_logic import decision_for_distance, decision_for_threat, THREAT_DECISION_MAP
from threat_zones import ThreatLevel, classify_threat, classify_world_position

import real_bear_controller


class ThreatLogicTests(unittest.TestCase):
    def test_green_zone(self):
        self.assertEqual(classify_threat(60.0), ThreatLevel.GREEN)

    def test_yellow_zone(self):
        self.assertEqual(classify_threat(25.0), ThreatLevel.YELLOW)

    def test_red_zone(self):
        self.assertEqual(classify_threat(12.0), ThreatLevel.RED)

    def test_emergency_zone(self):
        self.assertEqual(classify_threat(0.0), ThreatLevel.EMERGENCY)
        self.assertEqual(classify_threat(8.0), ThreatLevel.EMERGENCY)
        self.assertEqual(classify_threat(12.0), ThreatLevel.RED)
        self.assertEqual(classify_threat(-10.0), ThreatLevel.EMERGENCY)

    def test_circular_distance_zone(self):
        self.assertEqual(classify_world_position(-56.0, 42.0), ThreatLevel.GREEN)
        self.assertEqual(classify_world_position(0.0, 0.0), ThreatLevel.EMERGENCY)
        self.assertEqual(classify_world_position(12.0, 0.0), ThreatLevel.RED)
        self.assertEqual(classify_world_position(25.0, 0.0), ThreatLevel.YELLOW)
        self.assertEqual(classify_world_position(50.0, 0.0), ThreatLevel.GREEN)

    def test_decision_engine_actions(self):
        green = decision_for_distance(60.0)
        yellow = decision_for_distance(25.0)
        red = decision_for_distance(12.0)
        emergency = decision_for_distance(0.0)

        self.assertEqual(green.threat_level, ThreatLevel.GREEN)
        self.assertEqual(green.action, 'Continue patrol route')

        self.assertEqual(yellow.threat_level, ThreatLevel.YELLOW)
        self.assertEqual(yellow.action, 'Approach to 5-10 m and play deterrent sound; send yellow alert to GUI')

        self.assertEqual(red.threat_level, ThreatLevel.RED)
        self.assertEqual(red.action, 'Approach to 5-10 m and spray repellent; send high-priority alert to GUI and play repellant sound')

        self.assertEqual(emergency.threat_level, ThreatLevel.EMERGENCY)
        self.assertEqual(emergency.action, 'Monitor at 5-10 m and send critical alert to GUI')

    def test_all_threat_levels_map_to_a_decision(self):
        for threat in ThreatLevel:
            decision = decision_for_threat(threat)
            self.assertEqual(decision.threat_level, threat)
            self.assertIn(decision.action, THREAT_DECISION_MAP[threat].action)

    def test_real_controller_skips_amcl_wait(self):
        class DummyNavigator:
            def __init__(self, namespace):
                self.namespace = namespace

            def waitUntilNav2Active(self):
                raise AssertionError('AMCL wait should not be used in this stack')

        rclpy.init(args=None)
        try:
            with patch.object(real_bear_controller, 'BasicNavigator', DummyNavigator):
                node = real_bear_controller.BearThreatGazeboController()
                self.assertEqual(node.robot_namespace, 'parrot1')
                self.assertIsNotNone(node.navigator)
                node.destroy_node()
        finally:
            rclpy.shutdown()

    def test_visual_simulation_green_hold_then_resume(self):
        from visual_simulation import BearThreatVisualSimulation

        sim = BearThreatVisualSimulation()
        sim.update(17.5)
        self.assertIn(sim.state, {'returning_to_green', 'green_hold'})
        self.assertLess(sim.bear_pos[0], 0.0)

        sim.update(19.5)
        self.assertEqual(sim.state, 'green_hold')
        self.assertAlmostEqual(sim.bear_target[0], -50.0, delta=10.0)
        self.assertAlmostEqual(sim.bear_target[1], 90.0, delta=10.0)

        sim.update(23.0)
        self.assertEqual(sim.state, 'patrol')

    def test_visual_simulation_drone_moves_smoothly_toward_bear_in_yellow(self):
        from visual_simulation import BearThreatVisualSimulation

        sim = BearThreatVisualSimulation()
        sim.update(3.2)
        previous = sim.drone_pos[:]
        desired = [sim.bear_target[0] + 120.0, sim.bear_target[1] + 20.0]
        sim.update(3.4)

        distance_from_desired = (sim.drone_pos[0] - desired[0]) ** 2 + (sim.drone_pos[1] - desired[1]) ** 2
        previous_distance = (previous[0] - desired[0]) ** 2 + (previous[1] - desired[1]) ** 2
        self.assertGreater(previous_distance, distance_from_desired)
        self.assertGreater(distance_from_desired, 0.0)


if __name__ == '__main__':
    unittest.main()
