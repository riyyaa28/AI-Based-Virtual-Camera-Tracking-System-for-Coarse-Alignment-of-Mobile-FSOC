"""Detector adapters that preserve the repository's existing vision stack."""

from __future__ import annotations

from abc import ABC, abstractmethod
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from vision.classical_detector import detect_beacon_classical


Detection = tuple[Optional[tuple[float, float]], float, str]


class DetectorInterface(ABC):
    """A detector produces an image-derived beacon centroid and confidence."""

    name: str

    @abstractmethod
    def detect(self, frame: np.ndarray) -> Detection:
        raise NotImplementedError


class SimulatedBeaconDetector(DetectorInterface):
    """Fast compact-highlight detector for the rendered virtual camera frame.

    This intentionally works from pixels, rather than from the target's world
    position, so noise and turbulence influence the observation as they would
    in the browser's displayed camera image.
    """

    name = "simulated"

    def detect(self, frame: np.ndarray) -> Detection:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, 235, 255, cv2.THRESH_BINARY)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, 0.0, self.name
        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        if area < 2.0:
            return None, 0.0, self.name
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            return None, 0.0, self.name
        position = (moments["m10"] / moments["m00"], moments["m01"] / moments["m00"])
        confidence = min(0.98, 0.52 + min(area, 80.0) / 180.0)
        return position, confidence, self.name


class ClassicalDetectorAdapter(DetectorInterface):
    """Adapter for the project's ring-signature beacon detector."""

    name = "classical"

    def detect(self, frame: np.ndarray) -> Detection:
        position, confidence = detect_beacon_classical(frame, match_threshold=0.28)
        if position is None:
            return None, 0.0, self.name
        return (float(position[0]), float(position[1])), float(confidence), self.name


class YoloDetectorAdapter(DetectorInterface):
    """Lazy YOLO adapter; weights are not loaded unless the user selects it."""

    name = "yolo"

    def __init__(self, weights_path: str | Path = "beacon_yolo.pt"):
        from vision.yolo_detector import YoloBeaconDetector

        self._detector = YoloBeaconDetector(str(weights_path))

    def detect(self, frame: np.ndarray) -> Detection:
        position, confidence = self._detector.detect(frame)
        if position is None:
            return None, 0.0, self.name
        return (float(position[0]), float(position[1])), float(confidence), self.name


def create_detector(name: str) -> DetectorInterface:
    if name == "classical":
        return ClassicalDetectorAdapter()
    if name == "yolo":
        return YoloDetectorAdapter()
    return SimulatedBeaconDetector()
