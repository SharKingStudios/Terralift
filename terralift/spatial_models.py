"""Pure, deterministic models for spatial-belief experiment scenarios.

This module deliberately has no ROS or Anthropic dependency.  It is used by
the controller, evaluator, and unit tests so that scoring does not depend on a
language model or a running simulator.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import copy
import json
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Tuple

import networkx as nx


class ScenarioError(ValueError):
    """Raised when a scenario does not meet the experiment contract."""


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0

    @classmethod
    def from_value(cls, value: Mapping[str, Any] | Iterable[float]) -> "Pose2D":
        if isinstance(value, Mapping):
            return cls(float(value["x"]), float(value["y"]), float(value.get("yaw", 0.0)))
        values = list(value)
        if len(values) not in (2, 3):
            raise ScenarioError("A pose must have two or three numeric values")
        return cls(float(values[0]), float(values[1]), float(values[2]) if len(values) == 3 else 0.0)

    def to_dict(self) -> Dict[str, float]:
        return {"x": self.x, "y": self.y, "yaw": self.yaw}


@dataclass(frozen=True)
class Room:
    room_id: str
    pose: Pose2D
    geometry: Dict[str, float] = field(default_factory=dict)


@dataclass(frozen=True)
class Passage:
    passage_id: str
    rooms: Tuple[str, str]
    cost: float
    inspection_pose: Pose2D
    blocker_pose: Pose2D
    initial_traversable: bool = True


@dataclass(frozen=True)
class SemanticEntity:
    entity_id: str
    entity_type: str
    room_id: str
    pose: Pose2D


@dataclass(frozen=True)
class Question:
    question_id: str
    family: str
    text: str
    source_entity_type: str
    candidate_entity_type: str
    constraints: Dict[str, Any] = field(default_factory=dict)


@dataclass
class Scenario:
    """A full scenario, including controller-only changes.

    ``to_agent_seed`` is intentionally the sole supported route for creating
    the model-visible initial state.  It excludes all controller-only fields.
    """

    scenario_id: str
    title: str
    rooms: Dict[str, Room]
    passages: Dict[str, Passage]
    entities: Dict[str, SemanticEntity]
    question: Question
    hidden_changes: List[Dict[str, Any]]
    robot_start: Pose2D
    world_name: str = "spatial_warehouse"
    study_control: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "Scenario":
        required = {"scenario_id", "rooms", "passages", "entities", "question", "hidden_changes", "robot_start"}
        missing = required.difference(data)
        if missing:
            raise ScenarioError(f"Scenario is missing required keys: {sorted(missing)}")

        rooms: Dict[str, Room] = {}
        for raw in data["rooms"]:
            room_id = str(raw["id"])
            if room_id in rooms:
                raise ScenarioError(f"Duplicate room id: {room_id}")
            rooms[room_id] = Room(room_id, Pose2D.from_value(raw["pose"]), dict(raw.get("geometry", {})))

        passages: Dict[str, Passage] = {}
        for raw in data["passages"]:
            passage_id = str(raw["id"])
            linked = tuple(str(room) for room in raw["rooms"])
            if passage_id in passages or len(linked) != 2 or any(room not in rooms for room in linked):
                raise ScenarioError(f"Invalid passage {passage_id}")
            cost = float(raw.get("cost", 1.0))
            if cost <= 0:
                raise ScenarioError(f"Passage {passage_id} needs a positive cost")
            passages[passage_id] = Passage(
                passage_id,
                linked,  # type: ignore[arg-type]
                cost,
                Pose2D.from_value(raw["inspection_pose"]),
                Pose2D.from_value(raw["blocker_pose"]),
                bool(raw.get("initial_traversable", True)),
            )

        entities: Dict[str, SemanticEntity] = {}
        for raw in data["entities"]:
            entity_id = str(raw["id"])
            room_id = str(raw["room_id"])
            if entity_id in entities or room_id not in rooms:
                raise ScenarioError(f"Invalid entity {entity_id}")
            entities[entity_id] = SemanticEntity(
                entity_id,
                str(raw["type"]),
                room_id,
                Pose2D.from_value(raw.get("pose", rooms[room_id].pose.to_dict())),
            )

        raw_question = data["question"]
        question = Question(
            str(raw_question["id"]),
            str(raw_question["family"]),
            str(raw_question["text"]),
            str(raw_question.get("source_entity_type", "A")),
            str(raw_question.get("candidate_entity_type", "B")),
            dict(raw_question.get("constraints", {})),
        )
        scenario = cls(
            str(data["scenario_id"]),
            str(data.get("title", data["scenario_id"])),
            rooms,
            passages,
            entities,
            question,
            [dict(change) for change in data["hidden_changes"]],
            Pose2D.from_value(data["robot_start"]),
            str(data.get("world_name", "spatial_warehouse")),
            dict(data.get("study_control", {})),
        )
        scenario.validate()
        return scenario

    def validate(self) -> None:
        if self.question.family != "semantic_distance_ranking":
            raise ScenarioError(f"Unsupported question family: {self.question.family}")
        if not any(e.entity_type == self.question.source_entity_type for e in self.entities.values()):
            raise ScenarioError("Question has no source semantic entity")
        if not any(e.entity_type == self.question.candidate_entity_type for e in self.entities.values()):
            raise ScenarioError("Question has no candidate semantic entity")
        for change in self.hidden_changes:
            change_type = change.get("type")
            if change_type == "block_passage" and change.get("passage_id") not in self.passages:
                raise ScenarioError("Hidden change references an unknown passage")
            if change_type == "move_entity":
                if change.get("entity_id") not in self.entities or change.get("room_id") not in self.rooms:
                    raise ScenarioError("Hidden entity move references an unknown entity or room")
            if change_type not in {"block_passage", "open_passage", "move_entity"}:
                raise ScenarioError(f"Unsupported hidden change: {change_type}")
        declared_kind = self.study_control.get("change_kind")
        actual_kinds = {
            "topological" if change["type"] in {"block_passage", "open_passage"} else "semantic"
            for change in self.hidden_changes
        }
        actual_kind = "none" if not actual_kinds else next(iter(actual_kinds)) if len(actual_kinds) == 1 else "mixed"
        if declared_kind is not None and declared_kind != actual_kind:
            raise ScenarioError(
                f"study_control.change_kind={declared_kind!r} disagrees with changes ({actual_kind!r})"
            )
        declared_effect = self.study_control.get("answer_effect")
        answer_changed = self.answer_for_state(self.initial_state())["answer"] != self.answer_for_state(self.modified_state())["answer"]
        actual_effect = "flip" if answer_changed else "invariant"
        if declared_effect is not None and declared_effect != actual_effect:
            raise ScenarioError(
                f"study_control.answer_effect={declared_effect!r} disagrees with deterministic answer ({actual_effect!r})"
            )

    def initial_state(self) -> Dict[str, Any]:
        return {
            "passages": {p.passage_id: {"traversable": p.initial_traversable, "cost": p.cost} for p in self.passages.values()},
            "entities": {e.entity_id: {"type": e.entity_type, "room_id": e.room_id, "pose": e.pose.to_dict()} for e in self.entities.values()},
        }

    def modified_state(self) -> Dict[str, Any]:
        state = copy.deepcopy(self.initial_state())
        for change in self.hidden_changes:
            if change["type"] == "block_passage":
                state["passages"][change["passage_id"]]["traversable"] = False
            elif change["type"] == "open_passage":
                state["passages"][change["passage_id"]]["traversable"] = True
            elif change["type"] == "move_entity":
                state["entities"][change["entity_id"]]["room_id"] = change["room_id"]
                state["entities"][change["entity_id"]]["pose"] = self.rooms[change["room_id"]].pose.to_dict()
        return state

    def graph_for_state(self, state: Mapping[str, Any]) -> nx.Graph:
        graph = nx.Graph()
        graph.add_nodes_from(self.rooms)
        for passage in self.passages.values():
            passage_state = state["passages"][passage.passage_id]
            if passage_state["traversable"]:
                graph.add_edge(*passage.rooms, passage_id=passage.passage_id, weight=float(passage_state.get("cost", passage.cost)))
        return graph

    def answer_for_state(self, state: Mapping[str, Any]) -> Dict[str, Any]:
        """Compute a deterministic answer to the scenario question.

        Ties are returned in room-id order. Disconnected candidate rooms are
        excluded unless a future question family explicitly opts into them.
        """
        graph = self.graph_for_state(state)
        source_rooms = sorted(
            item["room_id"] for item in state["entities"].values()
            if item["type"] == self.question.source_entity_type
        )
        if len(source_rooms) != 1:
            raise ScenarioError("semantic_distance_ranking currently requires exactly one source entity")
        source = source_rooms[0]
        candidates = sorted({
            item["room_id"] for item in state["entities"].values()
            if item["type"] == self.question.candidate_entity_type
        })
        distances: Dict[str, Optional[float]] = {}
        for candidate in candidates:
            try:
                distances[candidate] = float(nx.shortest_path_length(graph, source, candidate, weight="weight"))
            except nx.NetworkXNoPath:
                distances[candidate] = None
        reachable = {room: distance for room, distance in distances.items() if distance is not None}
        if not reachable:
            return {"answer": None, "ties": [], "distances": distances, "source_room": source}
        maximum = max(reachable.values())
        ties = sorted(room for room, distance in reachable.items() if distance == maximum)
        return {"answer": ties[0] if len(ties) == 1 else ties, "ties": ties, "distances": distances, "source_room": source}

    def to_agent_seed(self) -> Dict[str, Any]:
        """Return only original belief data allowed in an LLM prompt.

        It intentionally omits ``hidden_changes``, obstacle poses, and the
        modified answer. Keep this contract small and testable.
        """
        return {
            "rooms": {room_id: {"pose": room.pose.to_dict()} for room_id, room in self.rooms.items()},
            "passages": {
                p.passage_id: {
                    "rooms": list(p.rooms),
                    "cost": p.cost,
                    "traversable": p.initial_traversable,
                    "provenance": "previously_recorded",
                }
                for p in self.passages.values()
            },
            "entities": {
                e.entity_id: {"type": e.entity_type, "room_id": e.room_id, "provenance": "previously_recorded"}
                for e in self.entities.values()
            },
            "question": {
                "id": self.question.question_id,
                "family": self.question.family,
                "text": self.question.text,
            },
        }

    def ground_truth_record(self) -> Dict[str, Any]:
        return {
            "scenario_id": self.scenario_id,
            "original_state": self.initial_state(),
            "hidden_changes": copy.deepcopy(self.hidden_changes),
            "modified_state": self.modified_state(),
            "original_answer": self.answer_for_state(self.initial_state()),
            "modified_answer": self.answer_for_state(self.modified_state()),
            "study_control": copy.deepcopy(self.study_control),
        }

    def changed_components(self) -> List[Dict[str, str]]:
        """Controller/evaluator-only identifiers for changed world facts."""
        components: List[Dict[str, str]] = []
        for change in self.hidden_changes:
            if change["type"] in {"block_passage", "open_passage"}:
                components.append({"kind": "passage", "id": str(change["passage_id"])})
            elif change["type"] == "move_entity":
                components.append({"kind": "entity", "id": str(change["entity_id"])})
        return components


def load_scenario(path: str | Path) -> Scenario:
    with Path(path).open("r", encoding="utf-8") as stream:
        return Scenario.from_dict(json.load(stream))


def default_scenario_path() -> Path:
    source_tree_path = Path(__file__).resolve().parents[1] / "config" / "spatial_scenarios" / "distance_ranking_flip.json"
    if source_tree_path.is_file():
        return source_tree_path
    try:  # Installed ROS packages keep data outside site-packages.
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("terralift")) / "config" / "spatial_scenarios" / "distance_ranking_flip.json"
    except ImportError:
        return source_tree_path
