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

    # Nav2's configured 0.35 m goal tolerance plus the 0.55 m sensor mount
    # makes a grounded blocker appear up to roughly 1.4 m away at the A_E
    # inspection pose.  1.6 m preserves a small measurement margin.
    def __init__(self, scenario: Scenario, inspection_tolerance_m: float = 1.5, blocked_distance_m: float = 1.6, marker_range_m: float = 2.5) -> None:
        self.scenario = scenario
        self.inspection_tolerance_m = inspection_tolerance_m
        self.blocked_distance_m = blocked_distance_m
        self.marker_range_m = marker_range_m
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

    def _scan_minimum(self) -> Optional[float]:
        """Return the nearest real LiDAR return at an inspection pose.

        Nav2 may complete at the requested position with either final yaw
        inside its tolerance.  At a passage inspection point, a nearby return
        anywhere in the 360-degree scan is therefore a more reliable physical
        indicator than assuming the robot faces one fixed world direction.
        The pose guard still prevents unrelated distant obstacles from being
        treated as passage evidence.
        """
        if self.scan is None:
            return None
        candidates: List[float] = []
        for value in self.scan.ranges:
            if not math.isfinite(value):
                continue
            if self.scan.range_min <= value <= self.scan.range_max:
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
        nearest_range = self._scan_minimum()
        if nearest_range is None:
            return {
                "evidence_id": self._next_id(), "kind": "passage", "passage_id": passage_id,
                "observed_status": "unobserved", "source": "lidar", "confidence": 0.0,
                "robot_pose": self.pose.to_dict(), "distance_to_inspection_pose_m": pose_distance,
                "reason": "no_valid_lidar_returns",
            }
        blocked = nearest_range <= self.blocked_distance_m
        return {
            "evidence_id": self._next_id(), "kind": "passage", "passage_id": passage_id,
            "observed_status": "blocked" if blocked else "open", "source": "nearby_lidar",
            "confidence": 0.9 if blocked else 0.75, "robot_pose": self.pose.to_dict(),
            "distance_to_inspection_pose_m": pose_distance, "nearest_lidar_range_m": nearest_range,
            "threshold_m": self.blocked_distance_m,
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
