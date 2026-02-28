#!/usr/bin/env python3
"""Photo verification routines for inspection completion gating."""

import os
from typing import Any, Dict

try:
    import cv2
except Exception:
    cv2 = None


class PhotoVerifier:
    def __init__(
        self,
        min_file_bytes: int = 50 * 1024,
        min_edges: int = 400,
        min_keypoints: int = 30,
        min_overlap: float = 0.6,
        min_blur_var: float = 80.0,
        min_center_coverage: float = 0.2,
    ) -> None:
        self.min_file_bytes = min_file_bytes
        self.min_edges = min_edges
        self.min_keypoints = min_keypoints
        self.min_overlap = min_overlap
        self.min_blur_var = min_blur_var
        self.min_center_coverage = min_center_coverage

    @staticmethod
    def _metadata_ok(metadata: Dict[str, Any]) -> bool:
        required = ("vehicle_id", "tire_id", "timestamp", "pose", "camera_intrinsics")
        return all(k in metadata for k in required)

    def verify(self, file_path: str, metadata: Dict[str, Any]) -> bool:
        if not os.path.isfile(file_path):
            return False
        if os.path.getsize(file_path) < self.min_file_bytes:
            return False
        if not self._metadata_ok(metadata):
            return False
        overlap = float(metadata.get("projection_overlap", 0.0))
        if overlap < self.min_overlap:
            return False
        center_cov = float(metadata.get("tire_center_coverage", 1.0))
        if center_cov < self.min_center_coverage:
            return False
        if cv2 is None:
            return True
        img = cv2.imread(file_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            return False
        edges = cv2.Canny(img, 80, 160)
        if int((edges > 0).sum()) < self.min_edges:
            return False
        corners = cv2.goodFeaturesToTrack(img, 200, 0.01, 8)
        kp_count = 0 if corners is None else len(corners)
        if kp_count < self.min_keypoints:
            return False
        blur_var = float(cv2.Laplacian(img, cv2.CV_64F).var())
        if blur_var < self.min_blur_var:
            return False
        return True
