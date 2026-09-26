"""Authoritative Python FSOC coarse-alignment simulation.

The model is deliberately lightweight enough to stream at 30 Hz.  It models
geometric line of sight, an image-forming virtual camera, detector and Kalman
updates, PID pointing commands, gimbal dynamics, and a link-state machine.
Atmospheric effects are approximations for a software simulator, not a full
wave-optics propagation model.
"""

from __future__ import annotations

import base64
import copy
import math
import time
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np

from control.ptz_controller import PIDPanTiltController
from disturbance.effects import add_gaussian_noise
from sim.sky import render_sky
from vision.kalman_tracker import BeaconKalmanTracker

from .detectors import DetectorInterface, create_detector


FRAME_WIDTH = 640
FRAME_HEIGHT = 480
STATE_NAMES = (
    "TARGET_NOT_VISIBLE",
    "SEARCHING",
    "ACQUIRED",
    "TRACKING",
    "COARSE_ALIGNED",
    "LINK_READY",
    "TARGET_LOST",
)


DEFAULT_CONFIG: dict[str, Any] = {
    "simulation": {"dt": 1 / 30, "speed": 1.0, "duration": 0},
    "target": {"trajectory": "figure8", "velocity": 1.0, "amplitude": 1.0},
    "camera": {
        "fov_horizontal": 28.0,
        "fov_vertical": 21.0,
        "pan_limit": 110.0,
        "tilt_limit": 55.0,
        "max_rate": 80.0,
    },
    "tracking": {
        "detector": "simulated",
        "prediction": True,
        "alignment_threshold": 1.15,
        "stable_duration": 1.5,
        "detector_hz": 15.0,
    },
    "disturbances": {
        "vibration": {"level": "LOW", "amplitude": 0.10, "frequency": 13.0},
        "camera_noise": 0.08,
        "turbulence": 0.10,
        "camera_motion": 0.18,
    },
    "environment": {
        "obstacles": True,
        "obstacle": {"enabled": False, "center": [85.0, 28.0, 8.0], "size": [16.0, 56.0, 32.0]},
        "atmospheric_attenuation": 0.08,
    },
}


@dataclass
class Observation:
    actual: tuple[float, float] | None
    detected: tuple[float, float] | None
    confidence: float
    source: str
    visible: bool
    frame: np.ndarray


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _wrap_degrees(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def _vec(value: list[float] | np.ndarray) -> np.ndarray:
    return np.asarray(value, dtype=float)


def _segment_intersects_aabb(start: np.ndarray, end: np.ndarray, center: np.ndarray, size: np.ndarray) -> bool:
    """Slab-method segment/AABB test used for simplified LOS obstruction."""

    low, high = center - size / 2.0, center + size / 2.0
    direction = end - start
    t_min, t_max = 0.0, 1.0
    for axis in range(3):
        if abs(direction[axis]) < 1e-8:
            if start[axis] < low[axis] or start[axis] > high[axis]:
                return False
            continue
        inv = 1.0 / direction[axis]
        enter, leave = (low[axis] - start[axis]) * inv, (high[axis] - start[axis]) * inv
        if enter > leave:
            enter, leave = leave, enter
        t_min, t_max = max(t_min, enter), min(t_max, leave)
        if t_min > t_max:
            return False
    return True


class FSOCSimulation:
    def __init__(self) -> None:
        self.config = copy.deepcopy(DEFAULT_CONFIG)
        self.rng = np.random.default_rng(41)
        self.detector: DetectorInterface = create_detector("simulated")
        self.reset()

    def reset(self) -> None:
        self.time = 0.0
        self.running = False
        self.uav1 = np.array([0.0, 34.0, 0.0])
        self.uav2 = np.array([138.0, 40.0, 14.0])
        self.previous_uav2 = self.uav2.copy()
        self.uav2_velocity = np.zeros(3)
        self.pan_command = -54.0
        self.tilt_command = 0.0
        self.pan_actual = -54.0
        self.tilt_actual = 0.0
        self.pan_velocity = 0.0
        self.tilt_velocity = 0.0
        self.last_measurement: tuple[float, float] | None = None
        self.last_prediction: tuple[float, float] | None = None
        self.last_detection_time = -999.0
        self.first_acquisition_time: float | None = None
        self.aligned_time = 0.0
        self.state = "TARGET_NOT_VISIBLE"
        self.target_losses = 0
        self.was_tracking = False
        self.detector_elapsed = 0.0
        self.tracker = BeaconKalmanTracker(max_coast_frames=12)
        self.controller = PIDPanTiltController()
        self.history: dict[str, list[float]] = {"error": [], "confidence": [], "distance": [], "time": []}
        self.target_path: list[list[float]] = [self.uav2.round(3).tolist()]
        self.total_error = 0.0
        self.error_samples = 0
        self.max_error = 0.0
        self.last_frame = self._empty_camera_frame()
        self.last_observation = Observation(None, None, 0.0, "simulated", False, self.last_frame)
        self.last_tick_wall = time.perf_counter()

    def set_running(self, running: bool) -> None:
        self.running = running
        if running and self.state == "TARGET_NOT_VISIBLE":
            self.state = "SEARCHING"

    def configure(self, updates: dict[str, Any]) -> None:
        """Deep-merge safe UI configuration updates into the central config."""

        def merge(destination: dict[str, Any], source: dict[str, Any]) -> None:
            for key, value in source.items():
                if key not in destination:
                    continue
                if isinstance(destination[key], dict) and isinstance(value, dict):
                    merge(destination[key], value)
                else:
                    destination[key] = value

        previous_detector = self.config["tracking"]["detector"]
        merge(self.config, updates)
        selected = str(self.config["tracking"]["detector"])
        if selected != previous_detector:
            try:
                self.detector = create_detector(selected)
            except Exception:
                # Missing optional Ultralytics dependencies should not stop the
                # real-time application; report a clean fallback in telemetry.
                self.config["tracking"]["detector"] = "simulated"
                self.detector = create_detector("simulated")

    def _target_position(self, t: float) -> np.ndarray:
        target = self.config["target"]
        speed = float(target["velocity"]) * float(self.config["simulation"]["speed"])
        amplitude = float(target["amplitude"])
        phase = t * 0.32 * speed
        trajectory = target["trajectory"]
        if trajectory == "circle":
            return np.array([138 + 32 * amplitude * math.cos(phase), 41 + 7 * math.sin(phase * 1.7), 22 + 32 * amplitude * math.sin(phase)])
        if trajectory == "straight":
            return np.array([138 + 32 * amplitude * math.sin(phase), 39 + 2 * math.sin(phase * 2), 17.0])
        if trajectory == "random":
            return np.array([138 + 34 * math.sin(phase * 0.91) + 13 * math.sin(phase * 2.17), 40 + 8 * math.sin(phase * 1.31), 15 + 27 * math.sin(phase * 0.73)])
        # figure-eight is the default and produces a recognisable 3-D track.
        return np.array([142 + 38 * amplitude * math.sin(phase), 41 + 8 * math.sin(phase * 1.45), 18 + 27 * amplitude * math.sin(phase * 2)])

    def _line_of_sight(self) -> bool:
        environment = self.config["environment"]
        obstacle = environment["obstacle"]
        if not environment["obstacles"] or not obstacle["enabled"]:
            return True
        return not _segment_intersects_aabb(self.uav1, self.uav2, _vec(obstacle["center"]), _vec(obstacle["size"]))

    def _target_angles(self) -> tuple[float, float, float]:
        delta = self.uav2 - self.uav1
        horizontal = math.hypot(delta[0], delta[2])
        return math.degrees(math.atan2(delta[2], delta[0])), math.degrees(math.atan2(delta[1], horizontal)), float(np.linalg.norm(delta))

    def _vibration(self) -> tuple[float, float]:
        vibration = self.config["disturbances"]["vibration"]
        levels = {"OFF": 0.0, "LOW": 0.55, "MEDIUM": 1.0, "HIGH": 2.25}
        scale = levels.get(str(vibration["level"]).upper(), 0.0)
        amplitude = float(vibration["amplitude"]) * scale
        frequency = float(vibration["frequency"])
        if amplitude == 0.0:
            return 0.0, 0.0
        base = 2 * math.pi * frequency * self.time
        pan = amplitude * (math.sin(base) + 0.32 * math.sin(base * 1.73))
        tilt = amplitude * (0.74 * math.sin(base * 1.29 + 1.2) + 0.18 * self.rng.normal())
        return pan, tilt

    def _update_gimbal(self, dt: float) -> tuple[float, float]:
        camera = self.config["camera"]
        motion = float(self.config["disturbances"]["camera_motion"])
        response = 34.0 * (1.0 - motion * 0.45)
        damping = 9.0 - motion * 2.4
        max_rate = float(camera["max_rate"])
        for axis in ("pan", "tilt"):
            actual = getattr(self, f"{axis}_actual")
            command = getattr(self, f"{axis}_command")
            velocity = getattr(self, f"{axis}_velocity")
            velocity += (response * (command - actual) - damping * velocity) * dt
            velocity = _clamp(velocity, -max_rate, max_rate)
            actual += velocity * dt
            limit = float(camera[f"{axis}_limit"])
            actual = _clamp(actual, -limit, limit)
            setattr(self, f"{axis}_velocity", velocity)
            setattr(self, f"{axis}_actual", actual)
        vibration = self._vibration()
        return self.pan_actual + vibration[0], self.tilt_actual + vibration[1]

    def _project(self, pan: float, tilt: float) -> tuple[tuple[float, float] | None, float, float, float]:
        target_pan, target_tilt, _ = self._target_angles()
        pan_error = _wrap_degrees(target_pan - pan)
        tilt_error = target_tilt - tilt
        camera = self.config["camera"]
        h_fov, v_fov = float(camera["fov_horizontal"]), float(camera["fov_vertical"])
        if abs(pan_error) > h_fov / 2 or abs(tilt_error) > v_fov / 2:
            return None, pan_error, tilt_error, math.hypot(pan_error, tilt_error)
        x = FRAME_WIDTH / 2 + pan_error / (h_fov / 2) * FRAME_WIDTH / 2
        y = FRAME_HEIGHT / 2 - tilt_error / (v_fov / 2) * FRAME_HEIGHT / 2
        return (float(x), float(y)), pan_error, tilt_error, math.hypot(pan_error, tilt_error)

    def _empty_camera_frame(self) -> np.ndarray:
        frame = render_sky(FRAME_WIDTH, FRAME_HEIGHT, mode="dusk").copy()
        frame = cv2.GaussianBlur(frame, (0, 0), 1.3)
        return frame

    def _render_observation(self, actual: tuple[float, float] | None, visible: bool) -> np.ndarray:
        frame = self._empty_camera_frame()
        turbulence = float(self.config["disturbances"]["turbulence"])
        noise = float(self.config["disturbances"]["camera_noise"])
        if visible and actual is not None:
            displacement = 12.0 * turbulence * np.array([
                math.sin(self.time * 7.0) + 0.35 * math.sin(self.time * 17.0),
                math.cos(self.time * 6.2) + 0.35 * math.sin(self.time * 14.0),
            ])
            displacement += self.rng.normal(0.0, 1.7 * turbulence, 2)
            position = np.array(actual) + displacement
            scintillation = _clamp(1.0 - 0.52 * turbulence + 0.22 * turbulence * math.sin(self.time * 8.4), 0.22, 1.0)
            center = (int(_clamp(position[0], 0, FRAME_WIDTH - 1)), int(_clamp(position[1], 0, FRAME_HEIGHT - 1)))
            core = max(2, int(5 * scintillation))
            cv2.circle(frame, center, int(core * 3.2), (70, 115, 170), 1, cv2.LINE_AA)
            cv2.circle(frame, center, int(core * 1.85), (170, 205, 235), 1, cv2.LINE_AA)
            cv2.circle(frame, center, core, (255, 255, 255), -1, cv2.LINE_AA)
        if turbulence > 0.08:
            blur = int(1 + turbulence * 5) * 2 + 1
            frame = cv2.GaussianBlur(frame, (blur, blur), 0)
        if noise > 0:
            # Existing repository image noise is applied to the formed image,
            # before detector execution, rather than to UI-only coordinates.
            frame = add_gaussian_noise(frame, sigma=2.0 + noise * 28.0)
            shot = self.rng.poisson(np.maximum(frame.astype(float), 1.0) * (0.08 + noise * 0.18))
            frame = np.clip(frame.astype(float) * (1.0 - noise * 0.18) + shot * noise * 0.18, 0, 255).astype(np.uint8)
        return frame

    def _observe(self, pan: float, tilt: float, should_detect: bool) -> tuple[Observation, float, float, float]:
        actual, pan_error, tilt_error, angular_error = self._project(pan, tilt)
        los = self._line_of_sight()
        visible = actual is not None and los
        frame = self._render_observation(actual, visible)
        detected: tuple[float, float] | None = None
        confidence, source = 0.0, self.detector.name
        turbulence = float(self.config["disturbances"]["turbulence"])
        if visible and should_detect:
            detected, confidence, source = self.detector.detect(frame)
            # Turbulence makes temporary detector loss plausible even when the
            # beacon remains geometrically inside the FOV.
            dropout = turbulence * 0.26
            if self.rng.random() < dropout:
                detected, confidence = None, 0.0
        return Observation(actual, detected, confidence, source, visible, frame), pan_error, tilt_error, angular_error

    def _update_tracking(self, observation: Observation, dt: float, pan_error: float, tilt_error: float) -> tuple[tuple[float, float] | None, float]:
        measurement = observation.detected
        tracked, kalman_confidence = self.tracker.update(measurement)
        confidence = observation.confidence if measurement is not None else kalman_confidence
        self.last_measurement = measurement
        self.last_prediction = tracked
        if measurement is not None:
            self.last_detection_time = self.time
            if self.first_acquisition_time is None:
                self.first_acquisition_time = self.time
            # PID error is measured in optical angles, not image pixels.
            command_pan, command_tilt = self.controller.update(pan_error, tilt_error, dt)
            camera = self.config["camera"]
            self.pan_command = _clamp(self.pan_command + command_pan * dt, -camera["pan_limit"], camera["pan_limit"])
            self.tilt_command = _clamp(self.tilt_command + command_tilt * dt, -camera["tilt_limit"], camera["tilt_limit"])
        else:
            # Search is independent of ground-truth target coordinates.
            self.pan_command = 92.0 * math.sin(self.time * 0.34)
            self.tilt_command = 14.0 * math.sin(self.time * 0.21)
            self.controller.reset()
        return tracked, float(confidence)

    def _advance_state(self, observation: Observation, angular_error: float, los: bool, dt: float) -> tuple[bool, bool, str, float]:
        recent_detection = observation.detected is not None
        tracking_recently = self.time - self.last_detection_time < 0.55
        if not self.running:
            self.state = "TARGET_NOT_VISIBLE"
            self.aligned_time = 0.0
        elif recent_detection and self.state in ("SEARCHING", "TARGET_NOT_VISIBLE", "TARGET_LOST"):
            self.state = "ACQUIRED"
        elif tracking_recently:
            self.state = "TRACKING"
        elif self.was_tracking:
            self.state = "TARGET_LOST"
            self.aligned_time = 0.0
        else:
            self.state = "SEARCHING"

        threshold = float(self.config["tracking"]["alignment_threshold"])
        stable = tracking_recently and los and angular_error <= threshold
        self.aligned_time = self.aligned_time + dt if stable else 0.0
        alignment_ok = self.aligned_time >= float(self.config["tracking"]["stable_duration"])
        if alignment_ok:
            self.state = "COARSE_ALIGNED"
        # A deliberately simple range-dependent atmospheric attenuation term.
        # It keeps link state conditional on the configured environment without
        # claiming a full FSOC propagation or link-budget calculation.
        _, _, distance = self._target_angles()
        atmospheric_loss = float(self.config["environment"]["atmospheric_attenuation"]) * distance / 100.0
        attenuation_ok = atmospheric_loss <= 1.0
        link = alignment_ok and observation.visible and los and attenuation_ok
        if link:
            self.state = "LINK_READY"
        if self.was_tracking and not tracking_recently:
            self.target_losses += 1
            self.was_tracking = False
        elif tracking_recently:
            self.was_tracking = True
        if link:
            reason = "LINK ACTIVE"
        elif not los:
            reason = "LOS obstruction"
        elif not observation.visible:
            reason = "Beacon outside camera FOV"
        elif not attenuation_ok:
            reason = "Atmospheric attenuation above link limit"
        elif not tracking_recently:
            reason = "No validated beacon detection"
        elif angular_error > threshold:
            reason = "Angular error above alignment threshold"
        else:
            reason = "Stabilizing coarse alignment"
        return alignment_ok, link, reason, atmospheric_loss

    def step(self, dt: float | None = None) -> dict[str, Any]:
        dt = float(dt or self.config["simulation"]["dt"])
        dt = _clamp(dt, 0.005, 0.08)
        started = time.perf_counter()
        if self.running:
            self.time += dt * float(self.config["simulation"]["speed"])
            self.previous_uav2 = self.uav2.copy()
            self.uav2 = self._target_position(self.time)
            self.uav2_velocity = (self.uav2 - self.previous_uav2) / dt
            self.target_path.append(self.uav2.round(3).tolist())
            if len(self.target_path) > 180:
                self.target_path.pop(0)
        pan, tilt = self._update_gimbal(dt)
        detector_period = 1.0 / max(1.0, float(self.config["tracking"]["detector_hz"]))
        self.detector_elapsed += dt
        should_detect = self.running and self.detector_elapsed >= detector_period
        if should_detect:
            self.detector_elapsed = 0.0
        observation, pan_error, tilt_error, angular_error = self._observe(pan, tilt, should_detect)
        tracked, confidence = self._update_tracking(observation, dt, pan_error, tilt_error) if should_detect else (self.last_prediction, self.last_observation.confidence)
        if not should_detect:
            observation.detected = self.last_measurement
            observation.confidence = self.last_observation.confidence
            observation.source = self.last_observation.source
        self.last_observation = observation
        self.last_frame = observation.frame
        target_pan, target_tilt, distance = self._target_angles()
        los = self._line_of_sight()
        alignment_ok, link, reason, atmospheric_loss = self._advance_state(observation, angular_error, los, dt)
        if self.running:
            self.total_error += angular_error
            self.error_samples += 1
            self.max_error = max(self.max_error, angular_error)
            for name, value in (("error", angular_error), ("confidence", confidence), ("distance", distance), ("time", self.time)):
                self.history[name].append(float(value))
                if len(self.history[name]) > 180:
                    self.history[name].pop(0)
        elapsed = (time.perf_counter() - started) * 1000.0
        return self._state_payload(
            pan, tilt, target_pan, target_tilt, distance, angular_error, confidence, tracked, los, alignment_ok, link, reason, atmospheric_loss, elapsed
        )

    def _frame_data_url(self) -> str:
        ok, encoded = cv2.imencode(".jpg", self.last_frame, [cv2.IMWRITE_JPEG_QUALITY, 76])
        if not ok:
            return ""
        return "data:image/jpeg;base64," + base64.b64encode(encoded.tobytes()).decode("ascii")

    def _state_payload(self, pan: float, tilt: float, target_pan: float, target_tilt: float, distance: float, angular_error: float, confidence: float, tracked: tuple[float, float] | None, los: bool, alignment_ok: bool, link: bool, reason: str, atmospheric_loss: float, latency_ms: float) -> dict[str, Any]:
        vibration = self.config["disturbances"]["vibration"]
        return {
            "type": "state",
            "timestamp": round(self.time, 3),
            "running": self.running,
            "uav1": {"position": self.uav1.round(3).tolist(), "rotation": [0.0, 0.0, 0.0]},
            "uav2": {"position": self.uav2.round(3).tolist(), "velocity": self.uav2_velocity.round(3).tolist(), "rotation": [0.0, round(math.degrees(math.atan2(self.uav2_velocity[2], self.uav2_velocity[0])) if np.linalg.norm(self.uav2_velocity[[0, 2]]) else 0.0, 2), 0.0]},
            "camera": {"pan": round(pan, 3), "tilt": round(tilt, 3), "commanded_pan": round(self.pan_command, 3), "commanded_tilt": round(self.tilt_command, 3), "fov_horizontal": self.config["camera"]["fov_horizontal"], "fov_vertical": self.config["camera"]["fov_vertical"]},
            "beacon": {"visible": self.last_observation.visible, "actual": list(self.last_observation.actual) if self.last_observation.actual else None, "x": self.last_measurement[0] if self.last_measurement else None, "y": self.last_measurement[1] if self.last_measurement else None, "confidence": round(confidence, 3), "source": self.last_observation.source},
            "tracking": {"error_x": round(_wrap_degrees(target_pan - pan), 3), "error_y": round(target_tilt - tilt, 3), "angular_error": round(angular_error, 3), "predicted_x": tracked[0] if tracked else None, "predicted_y": tracked[1] if tracked else None, "state": self.state},
            "disturbances": {"platform_vibration": float(vibration["amplitude"]), "vibration_level": vibration["level"], "camera_noise": self.config["disturbances"]["camera_noise"], "atmospheric_turbulence": self.config["disturbances"]["turbulence"], "camera_motion": self.config["disturbances"]["camera_motion"]},
            "target": {"distance": round(distance, 2), "azimuth": round(target_pan, 2), "elevation": round(target_tilt, 2)},
            "environment": {"los": los, "atmospheric_loss": round(atmospheric_loss, 3), "obstacle": self.config["environment"]["obstacle"]},
            "fsoc": {"fov_ok": self.last_observation.actual is not None, "los_ok": los, "alignment_ok": alignment_ok, "link": link, "reason": reason},
            "metrics": {"acquisition_time": round(self.first_acquisition_time, 2) if self.first_acquisition_time is not None else None, "average_angular_error": round(self.total_error / self.error_samples, 3) if self.error_samples else 0.0, "maximum_angular_error": round(self.max_error, 3), "lock_retention": round(100.0 * (sum(1 for value in self.history["error"] if value < self.config["tracking"]["alignment_threshold"]) / max(1, len(self.history["error"]))), 1), "fps": round(1.0 / max(self.config["simulation"]["dt"], 1e-6), 1), "processing_latency": round(latency_ms, 2), "target_losses": self.target_losses},
            "history": self.history,
            "target_path": self.target_path,
            "camera_view": {"width": FRAME_WIDTH, "height": FRAME_HEIGHT, "image": self._frame_data_url()},
            "config": self.config,
        }
