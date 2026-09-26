import unittest

import cv2
import numpy as np

from backend.detectors import SimulatedBeaconDetector
from backend.simulation import FSOCSimulation, _segment_intersects_aabb
from backend.schemas import ControlMessage
from control.ptz_controller import PIDPanTiltController


class SimulationTests(unittest.TestCase):
    def setUp(self):
        self.simulation = FSOCSimulation()

    def test_trajectory_moves_target(self):
        first = self.simulation._target_position(0.0)
        second = self.simulation._target_position(4.0)
        self.assertGreater(float(((first - second) ** 2).sum()), 1.0)

    def test_distance_and_angle_are_valid(self):
        pan, tilt, distance = self.simulation._target_angles()
        self.assertGreater(distance, 10.0)
        self.assertGreaterEqual(abs(pan), 0.0)
        self.assertGreaterEqual(abs(tilt), 0.0)

    def test_fov_projection_centres_aligned_target(self):
        pan, tilt, _ = self.simulation._target_angles()
        image_point, _, _, angular_error = self.simulation._project(pan, tilt)
        self.assertIsNotNone(image_point)
        assert image_point is not None
        self.assertAlmostEqual(image_point[0], 320.0, places=3)
        self.assertAlmostEqual(image_point[1], 240.0, places=3)
        self.assertAlmostEqual(angular_error, 0.0, places=3)

    def test_los_obstacle_blocks_segment(self):
        self.assertTrue(_segment_intersects_aabb(
            self.simulation.uav1,
            self.simulation.uav2,
            self.simulation.uav1 * 0.5 + self.simulation.uav2 * 0.5,
            [10.0, 80.0, 40.0],
        ))

    def test_pid_integral_is_bounded(self):
        controller = PIDPanTiltController(integral_limit=2.0)
        for _ in range(100):
            controller.update(10.0, -10.0, 0.1)
        self.assertLessEqual(abs(controller._integral[0]), 2.0)
        self.assertLessEqual(abs(controller._integral[1]), 2.0)

    def test_detector_centroid_is_image_derived(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.circle(frame, (390, 125), 8, (255, 255, 255), -1)
        position, confidence, source = SimulatedBeaconDetector().detect(frame)
        self.assertEqual(source, "simulated")
        self.assertIsNotNone(position)
        assert position is not None
        self.assertAlmostEqual(position[0], 390, delta=1)
        self.assertAlmostEqual(position[1], 125, delta=1)
        self.assertGreater(confidence, 0.5)

    def test_turbulence_changes_formed_camera_image(self):
        self.simulation.config["disturbances"]["camera_noise"] = 0.0
        self.simulation.config["disturbances"]["turbulence"] = 0.0
        calm = self.simulation._render_observation((320.0, 240.0), True)
        self.simulation.config["disturbances"]["turbulence"] = 1.0
        disturbed = self.simulation._render_observation((320.0, 240.0), True)
        self.assertGreater(float(np.abs(calm.astype(float) - disturbed.astype(float)).mean()), 0.1)

    def test_alignment_requires_stable_tracking(self):
        self.simulation.set_running(True)
        pan, tilt, _ = self.simulation._target_angles()
        self.simulation.pan_actual = self.simulation.pan_command = pan
        self.simulation.tilt_actual = self.simulation.tilt_command = tilt
        self.simulation.config["disturbances"]["vibration"]["level"] = "OFF"
        for _ in range(75):
            state = self.simulation.step(1 / 30)
        self.assertIn(state["tracking"]["state"], {"COARSE_ALIGNED", "LINK_READY"})

    def test_state_schema_has_required_sections(self):
        state = self.simulation.step(1 / 30)
        for name in ("uav1", "uav2", "camera", "beacon", "tracking", "disturbances", "fsoc"):
            self.assertIn(name, state)

    def test_websocket_command_schema(self):
        message = ControlMessage.model_validate({"action": "configure", "config": {"camera": {"fov_horizontal": 32}}})
        self.assertEqual(message.action, "configure")
        self.assertEqual(message.config["camera"]["fov_horizontal"], 32)


if __name__ == "__main__":
    unittest.main()
