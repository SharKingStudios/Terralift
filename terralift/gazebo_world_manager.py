"""Supported Gazebo Harmonic entity operations for scenario changes.

The manager uses ros_gz_sim's supported command-line clients for the bridged
``/world/<name>/create``, ``remove``, and ``set_pose`` services. It owns the
ground truth but exposes it only to the controller/evaluator, never Claude.
"""

from __future__ import annotations

import copy
from pathlib import Path
import subprocess
import time
from typing import Any, Dict, Optional

from .spatial_models import Pose2D, Scenario


class GazeboWorldManager:
    def __init__(self, scenario: Scenario, resource_root: str | Path) -> None:
        self.scenario = scenario
        self.resource_root = Path(resource_root)
        self._ground_truth = scenario.initial_state()
        self._created: set[str] = set()
        self._sim_time: Optional[float] = None

    def update_sim_time(self, seconds: float) -> None:
        self._sim_time = seconds

    def get_sim_time(self) -> Optional[float]:
        return self._sim_time

    def get_ground_truth(self) -> Dict[str, Any]:
        return copy.deepcopy(self._ground_truth)

    def _run(self, args: list[str], timeout: float = 15.0) -> None:
        completed = subprocess.run(args, text=True, capture_output=True, timeout=timeout)
        if completed.returncode != 0:
            raise RuntimeError(f"Gazebo entity command failed: {' '.join(args)}\n{completed.stderr}")

    def _run_with_compatibility_fallback(self, preferred: list[str], legacy: list[str]) -> None:
        """Support the current Jazzy ``create`` CLI and its demo-era alias."""
        try:
            self._run(preferred)
        except RuntimeError:
            self._run(legacy)

    def _blocker_name(self, passage_id: str) -> str:
        return f"passage_blocker_{passage_id}"

    def spawn_obstacle(self, passage_id: str) -> None:
        if passage_id not in self.scenario.passages:
            raise ValueError(f"Unknown passage: {passage_id}")
        name = self._blocker_name(passage_id)
        if name in self._created:
            return
        pose = self.scenario.passages[passage_id].blocker_pose
        sdf = self.resource_root / "models" / "passage_blocker" / "model.sdf"
        self._run_with_compatibility_fallback(
            ["ros2", "run", "ros_gz_sim", "create", "--world", self.scenario.world_name, "--name", name,
             "--file", str(sdf), "--x", str(pose.x), "--y", str(pose.y), "--z", "0.0", "--yaw", str(pose.yaw)],
            ["ros2", "run", "ros_gz_sim", "spawn_entity", "--name", name,
             "--sdf_filename", str(sdf), "--pos", str(pose.x), str(pose.y), "0.0", "--euler", "0", "0", str(pose.yaw)],
        )
        self._created.add(name)
        self._ground_truth["passages"][passage_id]["traversable"] = False

    def remove_obstacle(self, passage_id: str) -> None:
        name = self._blocker_name(passage_id)
        if name not in self._created:
            return
        self._run_with_compatibility_fallback(
            ["ros2", "run", "ros_gz_sim", "remove", "--world", self.scenario.world_name, "--entity", name],
            ["ros2", "run", "ros_gz_sim", "delete_entity", "--name", name],
        )
        self._created.remove(name)
        self._ground_truth["passages"][passage_id]["traversable"] = True

    def move_entity(self, entity_id: str, room_id: str) -> None:
        if entity_id not in self.scenario.entities or room_id not in self.scenario.rooms:
            raise ValueError("Unknown entity or room")
        pose = self.scenario.rooms[room_id].pose
        self._run_with_compatibility_fallback(
            ["ros2", "run", "ros_gz_sim", "set_entity_pose", "--world", self.scenario.world_name, "--entity", entity_id,
             "--x", str(pose.x), "--y", str(pose.y), "--z", "0.0", "--yaw", str(pose.yaw)],
            ["ros2", "run", "ros_gz_sim", "set_entity_pose", "--name", entity_id,
             "--pos", str(pose.x), str(pose.y), "0.0", "--euler", "0", "0", str(pose.yaw)],
        )
        self._ground_truth["entities"][entity_id]["room_id"] = room_id
        self._ground_truth["entities"][entity_id]["pose"] = pose.to_dict()

    def reset_robot_pose(self) -> None:
        pose = self.scenario.robot_start
        self._run_with_compatibility_fallback(
            ["ros2", "run", "ros_gz_sim", "set_entity_pose", "--world", self.scenario.world_name, "--entity", "terralift_mecanum",
             "--x", str(pose.x), "--y", str(pose.y), "--z", "0.25", "--yaw", str(pose.yaw)],
            ["ros2", "run", "ros_gz_sim", "set_entity_pose", "--name", "terralift_mecanum",
             "--pos", str(pose.x), str(pose.y), "0.25", "--euler", "0", "0", str(pose.yaw)],
        )

    def reset_world(self) -> None:
        for passage in self.scenario.passages:
            self.remove_obstacle(passage)
        for entity in self.scenario.entities.values():
            self.move_entity(entity.entity_id, entity.room_id)
        self._ground_truth = self.scenario.initial_state()
        self.reset_robot_pose()

    def apply_hidden_changes(self) -> None:
        """Controller-only method called before the LLM receives any message."""
        for change in self.scenario.hidden_changes:
            if change["type"] == "block_passage":
                self.spawn_obstacle(change["passage_id"])
            elif change["type"] == "open_passage":
                self.remove_obstacle(change["passage_id"])
            elif change["type"] == "move_entity":
                self.move_entity(change["entity_id"], change["room_id"])
        # Let Gazebo publish the new collision state before the first inspection.
        time.sleep(1.0)
