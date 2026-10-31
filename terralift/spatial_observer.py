"""Sensor-backed, proximity-limited observation interpretation.

No method in this module reads ``GazeboWorldManager`` ground truth. Passage
status comes only from the latest bridged LaserScan after Nav2 reaches the
configured inspection pose. Semantic marker readings are similarly limited to
entities within range of the latest robot pose.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from .spatial_models import Pose2D, Scenario


@dataclass
class ScanSnapshot:
    ranges: Sequence[float]
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float


class ProximityObservationAdapter:
    """Converts scan/pose readings into explicit, logged evidence."""

    # Each inspection pose is two metres before the fixed candidate barrier
    # location. The scanner is 0.55 m forward of base_link and Nav2 permits a
    # 0.35 m goal tolerance, so a 2.0 m forward-ray threshold has margin while
    # staying well below the distance to an open corridor's far wall.
    def __init__(self, scenario: Scenario, inspection_tolerance_m: float = 1.5, blocked_distance_m: float = 2.0, marker_range_m: float = 2.5, passage_scan_half_angle_rad: float = 0.22) -> None:
        self.scenario = scenario
        self.inspection_tolerance_m = inspection_tolerance_m
        self.blocked_distance_m = blocked_distance_m
        self.marker_range_m = marker_range_m
        self.passage_scan_half_angle_rad = passage_scan_half_angle_rad
        self.pose: Optional[Pose2D] = None
        self.scan: Optional[ScanSnapshot] = None
        self._observation_index = 0

    def update_pose(self, pose: Pose2D) -> None:
        self.pose = pose

    def update_scan(self, scan: ScanSnapshot) -> None:
        self.scan = scan

    def _next_id(self) -> str:
        self._observation_index += 1
        return f"observation_{self._observation_index:03d}"

    @staticmethod
    def _distance(first: Pose2D, second: Pose2D) -> float:
        return math.hypot(first.x - second.x, first.y - second.y)

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _scan_minimum_toward(self, target: Pose2D) -> Optional[float]:
        """Return the nearest LiDAR range in a cone toward one doorway.

        A room/corridor has nearby side walls by design, so using the minimum
        value from the entire 360-degree scan would falsely report every
        narrow passage as blocked. The target direction is fixed calibration
        geometry, not dynamic truth: only the measured range decides whether
        the spawned physical blocker is present.
        """
        if self.scan is None or self.pose is None:
            return None
        bearing_world = math.atan2(target.y - self.pose.y, target.x - self.pose.x)
        bearing_sensor = self._wrap_angle(bearing_world - self.pose.yaw)
        candidates: List[float] = []
        for index, value in enumerate(self.scan.ranges):
            if not math.isfinite(value):
                continue
            angle = self.scan.angle_min + index * self.scan.angle_increment
            if (abs(self._wrap_angle(angle - bearing_sensor)) <= self.passage_scan_half_angle_rad
                    and self.scan.range_min <= value <= self.scan.range_max):
                candidates.append(float(value))
        return min(candidates) if candidates else None

    def inspect_passage(self, passage_id: str) -> Dict[str, Any]:
        if passage_id not in self.scenario.passages:
            raise ValueError(f"Unknown passage: {passage_id}")
        passage = self.scenario.passages[passage_id]
        if self.pose is None:
            raise RuntimeError("No robot pose has been received")
        pose_distance = self._distance(self.pose, passage.inspection_pose)
        if pose_distance > self.inspection_tolerance_m:
            return {
                "evidence_id": self._next_id(), "kind": "passage", "passage_id": passage_id,
                "observed_status": "unobserved", "source": "pose_guard",
                "confidence": 0.0, "robot_pose": self.pose.to_dict(), "distance_to_inspection_pose_m": pose_distance,
            }
        nearest_range = self._scan_minimum_toward(passage.blocker_pose)
        if nearest_range is None:
            return {
                "evidence_id": self._next_id(), "kind": "passage", "passage_id": passage_id,
                "observed_status": "unobserved", "source": "lidar", "confidence": 0.0,
                "robot_pose": self.pose.to_dict(), "distance_to_inspection_pose_m": pose_distance,
                "reason": "no_valid_lidar_returns_in_passage_cone",
            }
        blocked = nearest_range <= self.blocked_distance_m
        return {
            "evidence_id": self._next_id(), "kind": "passage", "passage_id": passage_id,
            "observed_status": "blocked" if blocked else "open", "source": "nearby_lidar",
            "confidence": 0.9 if blocked else 0.75, "robot_pose": self.pose.to_dict(),
            "distance_to_inspection_pose_m": pose_distance, "nearest_lidar_range_m": nearest_range,
            "threshold_m": self.blocked_distance_m, "passage_scan_half_angle_rad": self.passage_scan_half_angle_rad,
        }

    def inspect_location(self, room_id: str, entities: Mapping[str, Mapping[str, Any]]) -> Dict[str, Any]:
        if room_id not in self.scenario.rooms:
            raise ValueError(f"Unknown room: {room_id}")
        if self.pose is None:
            raise RuntimeError("No robot pose has been received")
        visible = []
        for entity_id, entity in entities.items():
            entity_pose = Pose2D.from_value(entity["pose"])
            if self._distance(self.pose, entity_pose) <= self.marker_range_m:
                visible.append({"entity_id": entity_id, "type": entity["type"], "room_id": entity["room_id"]})
        return {
            "evidence_id": self._next_id(), "kind": "location", "room_id": room_id,
            "source": "proximity_marker_sensor", "confidence": 0.8,
            "robot_pose": self.pose.to_dict(), "visible_entities": sorted(visible, key=lambda item: item["entity_id"]),
            "range_m": self.marker_range_m,
        }
